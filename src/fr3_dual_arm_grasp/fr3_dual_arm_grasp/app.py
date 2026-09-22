"""Teaching application backend; one operation worker and a live ROS executor."""
from copy import deepcopy
import json
from pathlib import Path
import threading
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionFK
from controller_manager_msgs.srv import SwitchController, ListControllers

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .teach_model import (Feedback, TeachBook, captured_point, model_fingerprint,
                          finite_vector, pose_vector, SIDES)
from .motion import DualArmMoveIt, pose_values
from .demo_scene import DemoScene
from .workflow import run_workflow
from .display_geometry import VIEWS, matrix, vector, camera_neutral, object_path
from .teaching import TeachingMode, robot_mode


class DemoApp(Node):
    def __init__(self):
        super().__init__('fr3_teach_demo')
        for key, value in [('arms_file', ''), ('scene_file', ''), ('demo_config', ''),
                           ('points_file', '~/.ros/fr3_demo/teach_points.json'),
                           ('enable_execution', False), ('speed', 0.1), ('mode', 'mock'), ('hardware', '')]:
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.arms_file, self.scene_file = param('arms_file'), param('scene_file')
        arms = yaml.safe_load(Path(self.arms_file).read_text(encoding='utf-8'))
        scene = yaml.safe_load(Path(self.scene_file).read_text(encoding='utf-8'))
        config = yaml.safe_load(Path(param('demo_config')).read_text(encoding='utf-8'))
        self.mode = param('mode')
        self.robot_ips = {}
        if self.mode == 'real':
            from fr3_dual_arm_description.model import validate_hardware
            hardware = validate_hardware(yaml.safe_load(Path(param('hardware')).expanduser().read_text(encoding='utf-8')))
            self.robot_ips = {side: hardware[side]['robot_ip'] for side in SIDES}
        self.open_gap = float(arms['gripper']['open_gap'])
        self.feedback = Feedback(max_age=1.0)
        self.stop_event, self.confirm_event = threading.Event(), threading.Event()
        self.operation_lock = threading.Lock()
        self.worker = None
        self.status_text, self.awaiting_confirmation = '就绪：等待机器人反馈', ''
        self.recovery_required = False
        self.motion = DualArmMoveIt(self, self.feedback, self.stop_event, param('enable_execution'), param('speed'))
        self.switch_clients = {side: self.create_client(SwitchController,
            f'/{side}_controller_manager/switch_controller') for side in SIDES}
        self.list_clients = {side: self.create_client(ListControllers,
            f'/{side}_controller_manager/list_controllers') for side in SIDES}
        self.teaching = TeachingMode(self.switch_controllers, self.teach_command)
        self.live_poses, self.live_pose_error = {}, '等待实时 TCP'
        self.live_stamp, self.live_pending = 0.0, None
        self.create_timer(0.2, self.update_live_pose)
        dimensions = finite_vector(config['workpiece']['dimensions_m'], 3, 'workpiece dimensions')
        if any(x <= 0 or x > 1 for x in dimensions):
            raise ValueError('Workpiece box dimensions must be in (0, 1] m')
        offset = pose_vector(config['workpiece']['right_tcp_to_object'])
        self.scene = DemoScene(self, self.motion, scene, dimensions, offset)
        self.dwell = float(config.get('display_dwell_seconds', 2.0))
        if not 0 <= self.dwell <= 60:
            raise ValueError('Display dwell must be 0..60 seconds')
        self.retreat_distance = float(config.get('retreat_distance_m', 0.06))
        self.place_clearance = float(config.get('place_clearance_m', 0.08))
        if not all(math.isfinite(x) and 0.01 <= x <= 0.3 for x in (self.retreat_distance, self.place_clearance)):
            raise ValueError('Retreat/placement clearance must be 0.01..0.3 m')
        self.commissioned = config.get('scene_and_object_verified', False) is True
        self.book = TeachBook(model_fingerprint(self.arms_file, self.scene_file), self.open_gap)
        self.points_file = str(Path(param('points_file')).expanduser())
        self.load_error = ''
        if Path(self.points_file).exists():
            try:
                self.book.load(self.points_file)
            except Exception as exc:
                self.load_error = str(exc)
                self.status_text = '示教文件未载入：' + self.load_error
        self.latest_capture = None
        # UI-selectable policy for ordinary taught points. Display and
        # Cartesian approach/retreat segments remain TCP-based in both modes.
        self.keypoint_motion_mode = 'joints'
        self.perception_candidate = None
        self.status_pub = self.create_publisher(String, '/grasp/status_text', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        # Future perception adapter publishes an already transformed world TCP.
        # Incoming messages are candidates only; they cannot start a motion.
        self.create_subscription(PoseStamped, '/grasp/perception/grasp_tcp', self.on_candidate, 10)
        for name, callback in [('start', self.start_service), ('stop', self.stop_service),
                               ('status', self.status_service), ('continue', self.continue_service)]:
            self.create_service(Trigger, '/grasp/' + name, callback)
        self.create_timer(0.5, lambda: self.status_pub.publish(String(data=self.status_json())))

    @property
    def busy(self):
        return self.operation_lock.locked()

    def on_joints(self, msg):
        self.feedback.update(msg.name, msg.position)

    def update_live_pose(self):
        # Non-blocking and independent of motion cancellation/operation lock.
        if self.live_pending is not None and not self.live_pending.done():
            if time.monotonic() - self.live_request_time > 2:
                self.live_pending.cancel()
                self.live_pending = None
                self.live_pose_error = 'FK 反馈超时'
            return
        try:
            values = self.feedback.snapshot()
            if not self.motion.fk.service_is_ready():
                raise RuntimeError('等待 /compute_fk')
            request = GetPositionFK.Request()
            request.header.frame_id = 'world'
            request.fk_link_names = [side + '_gripper_tcp' for side in SIDES]
            request.robot_state = self.motion.state(values)
            self.live_request_time = time.monotonic()
            self.live_pending = self.motion.fk.call_async(request)
            stamp = self.live_request_time
            def receive(future):
                try:
                    result = future.result()
                    if result.error_code.val != 1:
                        raise RuntimeError('实时 FK 失败')
                    poses = dict(zip(result.fk_link_names, result.pose_stamped))
                    self.live_poses = {side: pose_values(poses[side + '_gripper_tcp'].pose) for side in SIDES}
                    self.live_stamp, self.live_pose_error = stamp, ''
                except Exception as exc:
                    self.live_pose_error = str(exc)
            self.live_pending.add_done_callback(receive)
        except Exception as exc:
            self.live_pose_error = str(exc)

    def set_speed(self, percent):
        value = float(percent) / 100
        if not math.isfinite(value) or not 0.01 <= value <= 0.3:
            raise ValueError('速度范围为 1%–30%')
        self.motion.speed = value
        self.publish(f'速度 {percent:.0f}%：下一段规划生效，当前运动不突变')

    def set_keypoint_motion_mode(self, mode):
        if mode not in ('joints', 'tcp'):
            raise ValueError('关键点模式必须是 joints 或 tcp')
        if self.busy:
            raise RuntimeError('流程运行中不能切换关键点模式')
        self.keypoint_motion_mode = mode
        self.publish('关键点运行模式：' + ('保存的 TCP 位姿反解' if mode == 'tcp' else '保存的关节角'))

    def switch_controllers(self, side, activate):
        if activate and not self.motion.enabled:
            return
        client = self.list_clients[side]
        response = self.motion.service(client, ListControllers.Request())
        states = {c.name: c.state for c in response.controller}
        names = [side + '_arm_controller', side + '_gripper_controller']
        if any(name not in states for name in names):
            raise RuntimeError(f'{side} 控制器未加载')
        changed = [name for name in names if (states[name] != 'active') == activate]
        if not changed:
            return
        request = SwitchController.Request()
        # Humble and later field names; deprecated aliases are only fallback.
        on = 'activate_controllers' if hasattr(request, 'activate_controllers') else 'start_controllers'
        off = 'deactivate_controllers' if hasattr(request, 'deactivate_controllers') else 'stop_controllers'
        setattr(request, on if activate else off, changed)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        if not self.motion.service(self.switch_clients[side], request).ok:
            raise RuntimeError(f'{side} 控制器切换失败')

    def teach_command(self, side, method, value):
        if self.mode == 'real':
            robot_mode(self.robot_ips[side], method, value)

    def set_teach_mode(self, enabled):
        if self.motion.fault:
            raise RuntimeError('运动结果不确定，请检查实机并重启后再切换')
        if enabled and (self.scene.owner or self.recovery_required):
            raise RuntimeError('请先人工处理零件并清除任务状态，再进入拖动示教')
        (self.teaching.enter if enabled else self.teaching.leave)()
        self.publish('拖动示教已启用，可拖动后采集' if enabled else '运动控制已恢复')

    def on_candidate(self, msg):
        if msg.header.frame_id != 'world':
            self.get_logger().warning('Perception candidate rejected: transform TCP to world first')
            return
        try:
            values = pose_vector(pose_values(msg.pose))
        except ValueError:
            return
        self.perception_candidate = dict(tcp=values, stamp_sec=msg.header.stamp.sec,
                                         stamp_nanosec=msg.header.stamp.nanosec)

    def publish(self, text):
        self.status_text = text
        self.get_logger().info(text)

    def status_json(self):
        return json.dumps(dict(message=self.status_text, busy=self.busy,
                               execution_enabled=self.motion.enabled,
                               awaiting_confirmation=self.awaiting_confirmation,
                               owner=self.scene.owner, recovery_required=self.recovery_required,
                               motion_fault=self.motion.fault,
                               keypoint_motion_mode=self.keypoint_motion_mode), ensure_ascii=False)

    def submit(self, label, function):
        if not self.operation_lock.acquire(blocking=False):
            raise RuntimeError('另一项操作正在运行')
        self.stop_event.clear()

        def work():
            try:
                self.publish(label)
                function()
                if not self.status_text.startswith('完成：'):
                    self.publish(label + '：完成')
            except Exception as exc:
                self.publish('停止 / 失败：' + str(exc))
            finally:
                self.operation_lock.release()

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def initialize_scene(self):
        if self.teaching.state != 'motion':
            raise RuntimeError('请先恢复运动控制；当前模式：' + self.teaching.state)
        if not self.scene.ready:
            self.scene.initialize()

    def capture(self, name=None):
        values = self.feedback.snapshot()
        poses = self.motion.tcp_poses(values)
        after = self.feedback.snapshot()
        if any(abs(values[j]-after[j]) > (0.0003 if 'finger' in j else 0.003) for j in values):
            raise RuntimeError('采集时机器人正在移动，请停止后重试')
        point = captured_point(values, poses, self.open_gap)
        self.latest_capture = point
        if name:
            if self.load_error:
                raise RuntimeError('已有文件未能载入，请先选择新的保存路径，避免覆盖：' + self.load_error)
            self.book.record(name, point)
            self.book.save(self.points_file)
        return point

    def move_point(self, name, side, execute=False):
        self.initialize_scene()
        if name == 'right_display':
            if side == 'both':
                raise ValueError('展示中心由单臂占用，请选择左手或右手')
            return self.move_display_neutral(side, execute)
        point = deepcopy(self.book.points[name])
        self.book.validate_point(point)
        sides = SIDES if side == 'both' else (side,)
        if self.keypoint_motion_mode == 'tcp':
            targets = {s: point[s]['tcp'] for s in sides}
            return self.motion.poses(
                targets, 'both_arms' if side == 'both' else side + '_arm', execute)
        targets = {f'{s}_j{i}': q for s in sides for i, q in enumerate(point[s]['joints'], 1)}
        return self.motion.joints(targets, 'both_arms' if side == 'both' else side + '_arm', execute)

    def tcp_object(self, side):
        if self.scene.owner == side and self.scene.local_pose is not None:
            return matrix(self.scene.local_pose)
        right = matrix(self.scene.offset)
        if side == 'right':
            return right
        receive = self.book.points['left_receive']
        return np.linalg.inv(matrix(receive['left']['tcp'])) @ matrix(receive['right']['tcp']) @ right

    def display_neutral(self):
        # The stored measured joints/TCP remain unchanged; derive targets at use time.
        return camera_neutral(self.scene.scene['camera'],
            self.book.points['right_display']['right']['tcp'], matrix(self.scene.offset))

    def move_display_neutral(self, side, execute=False):
        target = vector(self.display_neutral() @ np.linalg.inv(self.tcp_object(side)))
        return self.motion.pose(side, target, execute=execute)

    def scan_display(self, side):
        neutral, offset = self.display_neutral(), self.tcp_object(side)
        self.move_display_neutral(side, True)
        for index, angles in enumerate(VIEWS, 1):
            self.motion.guard(True)
            self.publish(f'{side} 展示 {index}/{len(VIEWS)}，零件 RPY {angles}°')
            target = neutral.copy()
            target[:3, :3] = neutral[:3, :3] @ Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
            if any(angles):
                self.motion.cartesian(side, object_path(neutral, target, offset), True)
            if self.stop_event.wait(self.dwell):
                raise RuntimeError('展示已停止')
            if any(angles):
                self.motion.cartesian(side, object_path(target, neutral, offset), True)

    def retreat_donor(self):
        tcp = matrix(self.motion.tcp_poses()['right'])
        tcp[:3, 3] -= tcp[:3, 2] * self.retreat_distance
        self.motion.pose('right', vector(tcp), execute=True, linear=True)

    def place_move(self, above):
        pose = list(self.book.points['left_place']['left']['tcp'])
        pose[2] += self.place_clearance
        if above:
            # Before placement use global planning; after detach rise linearly.
            self.motion.pose('left', pose, True, linear=self.scene.owner is None)
        else:
            self.motion.pose('left', self.book.points['left_place']['left']['tcp'], True, linear=True)

    def manual_joints(self, side, joints, execute):
        self.initialize_scene()
        targets = {f'{side}_j{i}': q for i, q in enumerate(finite_vector(joints, 6, 'joints'), 1)}
        self.motion.joints(targets, side + '_arm', execute)

    def manual_pose(self, side, pose, execute, linear):
        self.initialize_scene()
        self.motion.pose(side, pose_vector(pose), execute, linear)

    def manual_gripper(self, side, gap, execute):
        if not 0 <= gap <= self.open_gap:
            raise ValueError('夹爪开口越界')
        self.initialize_scene()
        self.motion.gripper(side, gap, execute)

    def start_demo(self):
        if not self.commissioned:
            raise RuntimeError('请核对工作台 / 零件尺寸和 TCP 偏移，并在 demo.yaml 设置 scene_and_object_verified: true')
        run_workflow(self)

    def continue_demo(self):
        if not self.awaiting_confirmation:
            raise RuntimeError('当前没有等待确认的步骤')
        self.confirm_event.set()

    def recover(self):
        self.scene.clear_after_manual_recovery()
        self.recovery_required = False

    def start_service(self, request, response):
        try:
            self.submit('启动完整 demo', self.start_demo)
            response.success, response.message = True, 'Started; monitor /grasp/status'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def stop_service(self, request, response):
        self.motion.cancel()
        response.success, response.message = True, 'Cancel requested; wait for terminal status; grippers retained'
        return response

    def continue_service(self, request, response):
        try:
            self.continue_demo()
            response.success, response.message = True, 'Operator confirmed'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def status_service(self, request, response):
        response.success, response.message = True, self.status_json()
        return response


def main():
    rclpy.init()
    node = DemoApp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motion.cancel()
        if node.worker:
            # Keep callbacks spinning during cancellation acknowledgement.
            import time
            deadline = time.monotonic() + 6
            while node.worker.is_alive() and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
