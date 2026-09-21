"""Teaching application backend; one operation worker and a live ROS executor."""
from copy import deepcopy
import json
from pathlib import Path
import threading

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


class DemoApp(Node):
    def __init__(self):
        super().__init__('fr3_teach_demo')
        for key, value in [('arms_file', ''), ('scene_file', ''), ('demo_config', ''),
                           ('points_file', '~/.ros/fr3_demo/teach_points.json'),
                           ('enable_execution', False), ('speed', 0.1)]:
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.arms_file, self.scene_file = param('arms_file'), param('scene_file')
        arms = yaml.safe_load(Path(self.arms_file).read_text(encoding='utf-8'))
        scene = yaml.safe_load(Path(self.scene_file).read_text(encoding='utf-8'))
        config = yaml.safe_load(Path(param('demo_config')).read_text(encoding='utf-8'))
        self.open_gap = float(arms['gripper']['open_gap'])
        self.feedback = Feedback(max_age=1.0)
        self.stop_event, self.confirm_event = threading.Event(), threading.Event()
        self.operation_lock = threading.Lock()
        self.worker = None
        self.status_text, self.awaiting_confirmation = '就绪：等待机器人反馈', ''
        self.recovery_required = False
        self.motion = DualArmMoveIt(self, self.feedback, self.stop_event, param('enable_execution'), param('speed'))
        dimensions = finite_vector(config['workpiece']['dimensions_m'], 3, 'workpiece dimensions')
        if any(x <= 0 or x > 1 for x in dimensions):
            raise ValueError('Workpiece box dimensions must be in (0, 1] m')
        offset = pose_vector(config['workpiece']['right_tcp_to_object'])
        self.scene = DemoScene(self, self.motion, scene, dimensions, offset)
        self.dwell = float(config.get('display_dwell_seconds', 2.0))
        if not 0 <= self.dwell <= 60:
            raise ValueError('Display dwell must be 0..60 seconds')
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
                               motion_fault=self.motion.fault), ensure_ascii=False)

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
        point = deepcopy(self.book.points[name])
        self.book.validate_point(point)
        sides = SIDES if side == 'both' else (side,)
        targets = {f'{s}_j{i}': q for s in sides for i, q in enumerate(point[s]['joints'], 1)}
        return self.motion.joints(targets, 'both_arms' if side == 'both' else side + '_arm', execute)

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
