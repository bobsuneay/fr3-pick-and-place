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
from ament_index_python.packages import get_package_share_directory

from .teach_model import (Feedback, TeachBook, captured_point, model_fingerprint,
                          finite_vector, pose_vector, validate_handover_centerline,
                          SIDES)
from .motion import DualArmMoveIt, pose_values
from .demo_scene import DemoScene
from .workflow import run_workflow
from .display_geometry import matrix, vector, camera_neutral, object_path, display_views
from .teaching import TeachingMode, robot_mode
from .kinematics import build_kinematics
from fr3_dual_arm_description.model import build_model


class DemoApp(Node):
    def __init__(self):
        super().__init__('fr3_teach_demo')
        share = Path(get_package_share_directory('fr3_dual_arm_grasp'))
        default_points = share / 'config/teach_points.json'
        for parent in share.parents:
            source_config = parent / 'src/fr3_dual_arm_grasp/config'
            if source_config.is_dir():
                default_points = source_config / 'teach_points.json'
                break
        for key, value in [('arms_file', ''), ('scene_file', ''), ('demo_config', ''),
                           ('points_file', str(default_points)),
                           ('enable_execution', False), ('speed', 0.1), ('mode', 'mock'), ('hardware', '')]:
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.arms_file, self.scene_file = param('arms_file'), param('scene_file')
        arms = yaml.safe_load(Path(self.arms_file).read_text(encoding='utf-8'))
        scene = yaml.safe_load(Path(self.scene_file).read_text(encoding='utf-8'))
        config = yaml.safe_load(Path(param('demo_config')).read_text(encoding='utf-8'))
        self.demo_config = config
        self.mode = param('mode')
        self.robot_ips = {}
        if self.mode == 'real':
            from fr3_dual_arm_description.model import validate_hardware
            hardware = validate_hardware(yaml.safe_load(Path(param('hardware')).expanduser().read_text(encoding='utf-8')))
            self.robot_ips = {side: hardware[side]['robot_ip'] for side in SIDES}
        description_share = get_package_share_directory('fr3_dual_arm_description')
        self.kinematics = build_kinematics(
            build_model(description_share, self.scene_file, arms, mode='mock'),
            arms,
        )
        self.open_gap = float(arms['gripper']['open_gap'])
        self.feedback = Feedback(max_age=1.0)
        self.stop_event = threading.Event()
        self.operation_lock = threading.Lock()
        self.worker = None
        self.status_text = '就绪：等待机器人反馈'
        self.recovery_required = False
        self.motion = DualArmMoveIt(
            self, self.feedback, self.stop_event, param('enable_execution'), param('speed'),
            kinematics=self.kinematics,
        )
        self.switch_clients = {side: self.create_client(SwitchController,
            f'/{side}_controller_manager/switch_controller') for side in SIDES}
        self.list_clients = {side: self.create_client(ListControllers,
            f'/{side}_controller_manager/list_controllers') for side in SIDES}
        self.teaching = TeachingMode(self.switch_controllers, self.teach_command)
        self.live_poses, self.live_pose_error = {}, '等待实时 TCP'
        self.live_stamp, self.live_pending = 0.0, None
        self.create_timer(0.2, self.update_live_pose)
        workpiece = config['workpiece']
        shape = workpiece.get('shape', 'box')
        if shape == 'cylinder':
            radius = float(workpiece['radius_m'])
            height = float(workpiece['height_m'])
            if not 0 < radius <= 0.5 or not 0 < height <= 1.0:
                raise ValueError('Cylinder workpiece radius/height outside range')
            dimensions = [height, 2.0 * radius, 2.0 * radius]
        else:
            dimensions = finite_vector(workpiece['dimensions_m'], 3, 'workpiece dimensions')
        if any(x <= 0 or x > 1 for x in dimensions):
            raise ValueError('Workpiece box dimensions must be in (0, 1] m')
        offset = pose_vector(config['workpiece']['right_tcp_to_object'])
        self.grasp_gap_min = float(config['workpiece'].get('grasp_gap_min_m', 0.005))
        self.grasp_gap_max = float(config['workpiece'].get('grasp_gap_max_m', 0.04))
        self.grasp_gap = float(config['workpiece'].get('grasp_gap_m', 0.015))
        if not 0 < self.grasp_gap_min < self.grasp_gap_max < self.open_gap:
            raise ValueError('Workpiece grasp gap range must be inside the calibrated gripper opening')
        if not self.grasp_gap_min <= self.grasp_gap <= self.grasp_gap_max:
            raise ValueError('Workpiece grasp gap must lie inside the grasp verification range')
        self.handover_line_tolerance = float(
            config.get('handover_centerline_tolerance_m', 0.005))
        self.handover_angle_tolerance = math.radians(float(
            config.get('handover_centerline_angle_deg', 5.0)))
        if not 0.001 <= self.handover_line_tolerance <= 0.02:
            raise ValueError('Handover centerline tolerance must be 1..20 mm')
        if not math.radians(1) <= self.handover_angle_tolerance <= math.radians(15):
            raise ValueError('Handover centerline angle tolerance must be 1..15 deg')
        self.scene = DemoScene(
            self, self.motion, scene, dimensions, offset,
            show_workpiece=config.get('show_workpiece_in_rviz', False), shape=shape)
        self.dwell = float(config.get('display_dwell_seconds', 2.0))
        if not 0 <= self.dwell <= 60:
            raise ValueError('Display dwell must be 0..60 seconds')
        self.display_distance = float(config.get('display_distance_m', 0.30))
        if not 0.1 <= self.display_distance <= 1.0:
            raise ValueError('Display distance must be 0.1..1.0 m')
        self.display_pose_joints_deg = config.get('display_pose_joints_deg')
        if self.display_pose_joints_deg is not None:
            self.display_pose_joints_deg = finite_vector(
                self.display_pose_joints_deg, 6, 'display_pose_joints_deg')
        self.display_turn_step_deg = float(config.get('display_turn_step_deg', 15.0))
        if not 1.0 <= self.display_turn_step_deg <= 45.0:
            raise ValueError('Display turn step must be 1..45 deg')
        self.display_turn_direction = 1 if config.get('display_turn_direction', 1) >= 0 else -1
        self.display_x_tilts_deg = config.get('display_x_tilts_deg', [30.0, -30.0])
        if (not isinstance(self.display_x_tilts_deg, list) or
                any(not isinstance(v, (int, float)) or not np.isfinite(v)
                    for v in self.display_x_tilts_deg)):
            raise ValueError('display_x_tilts_deg must be a list of finite angles')
        self.display_x_tilts_deg = [float(v) for v in self.display_x_tilts_deg]
        self.skip_event = threading.Event()
        self.awaiting_skip = False
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
        self._refresh_kinematics_seed()
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
                               ('skip', self.skip_service), ('status', self.status_service)]:
            self.create_service(Trigger, '/grasp/' + name, callback)
        self.create_timer(0.5, lambda: self.status_pub.publish(String(data=self.status_json())))

    @property
    def busy(self):
        return self.operation_lock.locked()

    def on_joints(self, msg):
        self.feedback.update(msg.name, msg.position)

    def _refresh_kinematics_seed(self):
        """Seed the numeric IK with the taught, guaranteed-reachable ready pose."""
        ready = self.book.points.get('ready')
        if ready is None:
            self.motion.seed_joints = {}
            return
        self.motion.seed_joints = {
            side: list(ready[side]['joints']) for side in SIDES
        }

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
        if not math.isfinite(value) or not 0.01 <= value <= 0.8:
            raise ValueError('速度范围为 1%–80%')
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
                               owner=self.scene.owner, recovery_required=self.recovery_required,
                               motion_fault=self.motion.fault,
                               awaiting_skip=self.awaiting_skip,
                               keypoint_motion_mode=self.keypoint_motion_mode), ensure_ascii=False)

    def skip_step(self):
        """Signal the running workflow to skip the step that just failed."""
        if not self.busy:
            raise RuntimeError('当前没有正在运行的流程')
        self.skip_event.set()
        self.publish('已请求跳过当前失败步骤')

    def _wait_skip_or_stop(self, label, error):
        """Block the worker until the operator skips or stops after a failure."""
        self.awaiting_skip = True
        self.publish(f'{label} 失败：{error}')
        self.publish('等待操作：点“跳过本步”继续，或“停止流程”结束')
        self.skip_event.clear()
        try:
            while not self.skip_event.wait(0.1):
                if self.stop_event.is_set():
                    raise RuntimeError('Cancelled; gripper ownership retained')
            self.publish('已跳过当前失败步骤，继续')
        finally:
            self.awaiting_skip = False

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
            self._refresh_kinematics_seed()
        return point

    def _generated_point(self, tcp, gap=0.0):
        ready = self.book.points['ready']
        return dict(frame='world', captured_at='generated',
                    left=dict(joints=ready['left']['joints'], tcp=tcp,
                              tcp_link='left_gripper_tcp', gap_m=gap),
                    right=dict(joints=ready['right']['joints'], tcp=tcp,
                               tcp_link='right_gripper_tcp', gap_m=gap))

    def generated_targets(self, name):
        pre = self.book.points['right_pregrasp']['right']['tcp']
        if name in ('right_orient', 'right_grasp'):
            target = matrix(pre)
            if name == 'right_grasp':
                target[2, 3] = float(self.scene.scene['table']['top_z']) + 0.015
            # Keep the taught yaw/roll, but force TCP Z vertically downward.
            self._force_tcp_down(target)
            return self._generated_point(vector(target), 0.0)
        if name == 'handover_ready':
            center = np.asarray(self.demo_config.get('handover_center_xyz', [0.35, 0.0, 1.0]), dtype=float)
            separation = float(self.demo_config.get('handover_separation_m', 0.16))
            # Hand over along world Y: both wrists can reach opposing +Y/-Y
            # approach axes here, whereas the X axis leaves the left arm out of
            # its reachable orientation space (IK fails).
            axis = np.array([0.0, 1.0, 0.0])
            z_right, z_left = axis, -axis
            # The two approach axes are collinear (+Y / -Y); roll the left
            # fingers 90 degrees about Z so the hands interleave without
            # colliding when both close on the same part.
            y_right = np.array([0.0, 0.0, 1.0])
            y_left = np.array([1.0, 0.0, 0.0])
            right_r = np.column_stack((np.cross(y_right, z_right), y_right, z_right))
            left_r = np.column_stack((np.cross(y_left, z_left), y_left, z_left))
            right = np.eye(4); left = np.eye(4)
            right[:3, :3], left[:3, :3] = right_r, left_r
            # Keep the right hand on its natural -Y side and the left hand on
            # its +Y side so the two arms never cross each other; both approach
            # the shared part from their own side.
            right[:3, 3] = center - axis * separation / 2.0
            left[:3, 3] = center + axis * separation / 2.0
            return {
                'right': dict(tcp=vector(right), joints=self.book.points['ready']['right']['joints'], gap_m=0.0),
                'left': dict(tcp=vector(left), joints=self.book.points['ready']['left']['joints'], gap_m=0.0),
            }
        raise ValueError('Unknown generated target: ' + name)

    @staticmethod
    def _force_tcp_down(target):
        # Force the TCP +Z axis to point along world -Z (vertical down) while
        # keeping the taught X direction projected into the horizontal plane.
        z_axis = np.array([0.0, 0.0, -1.0])
        x_axis = target[:3, 0] - np.dot(target[:3, 0], z_axis) * z_axis
        norm = np.linalg.norm(x_axis)
        if norm < 1e-9:
            x_axis = np.array([1.0, 0.0, 0.0])
        else:
            x_axis /= norm
        y_axis = np.cross(z_axis, x_axis)
        target[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
        return target

    def grasp_target(self):
        return self.generated_targets('right_grasp')['right']['tcp']

    def move_point(self, name, side, execute=False, linear=False):
        self.initialize_scene()
        if name == 'right_display':
            if side == 'both':
                raise ValueError('展示中心由单臂占用，请选择左手或右手')
            return self.move_display_neutral(side, execute)
        if name in ('right_orient', 'right_grasp', 'handover_ready'):
            generated = self.generated_targets(name)
            if name in ('right_orient', 'right_grasp'):
                # Orient in joint space, descend straight with MoveL; a pure
                # vertical translation completes the Cartesian path reliably.
                return self.motion.pose('right', generated['right']['tcp'], execute, linear=linear)
            # Move the two hands one at a time so they never plan as a single
            # both_arms goal; the right hand goes first, then the left.
            self.motion.pose('right', generated['right']['tcp'], execute)
            return self.motion.pose('left', generated['left']['tcp'], execute)
        point = deepcopy(self.book.points[name])
        self.book.validate_point(point)
        sides = SIDES if side == 'both' else (side,)
        if linear:
            if side == 'both':
                raise ValueError('Cartesian keypoint motion supports one arm at a time')
            return self.motion.pose(side, point[side]['tcp'], execute, linear=True)
        if self.keypoint_motion_mode == 'tcp':
            targets = {s: point[s]['tcp'] for s in sides}
            return self.motion.poses(
                targets, 'both_arms' if side == 'both' else side + '_arm', execute)
        targets = {f'{s}_j{i}': q for s in sides for i, q in enumerate(point[s]['joints'], 1)}
        return self.motion.joints(targets, 'both_arms' if side == 'both' else side + '_arm', execute)

    def tcp_object(self, side):
        if self.scene.owner == side and self.scene.local_pose is not None:
            return matrix(self.scene.local_pose)
        if side == 'left':
            raise RuntimeError('左手展示仅能在交接完成后进行')
        return matrix(self.scene.offset)

    def _display_seed(self, side):
        if side == 'right' and self.display_pose_joints_deg is not None:
            return [math.radians(value) for value in self.display_pose_joints_deg]
        return None

    def _camera_optical_transform(self):
        camera = self.scene.scene['camera']
        optical = (Rotation.from_euler('xyz', camera['rpy']) *
                   Rotation.from_euler('xyz', [-math.pi / 2, 0, -math.pi / 2]))
        result = np.eye(4)
        result[:3, :3] = optical.as_matrix()
        result[:3, 3] = np.asarray(camera['xyz'])
        return result

    def display_neutral(self, side='right'):
        # Place the part centre on the head-camera optical axis at the
        # configured distance (within the reachable 0.12-0.36 m window for a
        # face-on view), keeping the reference posture's side-on orientation.
        # The left hand mirrors the right about the X-Z plane.
        optical = self._camera_optical_transform()
        obj = np.eye(4)
        obj[:3, 3] = optical[:3, 3] + optical[:3, 2] * self.display_distance
        seed = self._display_seed('right')
        if seed is not None and 'right' in self.kinematics:
            tcp = self.kinematics['right'].fk(seed)
            base = (tcp @ matrix(self.scene.offset))[:3, :3]
        else:
            taught = matrix(self.book.points['right_pregrasp']['right']['tcp'])
            base = (taught @ matrix(self.scene.offset))[:3, :3]
        if side == 'left':
            mirror = np.diag([1.0, -1.0, 1.0])
            base = mirror @ base @ mirror
            obj[:3, 3] = [obj[0, 3], -obj[1, 3], obj[2, 3]]
        obj[:3, :3] = base
        return obj

    def move_display_neutral(self, side, execute=False):
        target = vector(self.display_neutral(side) @ np.linalg.inv(self.tcp_object(side)))
        return self.motion.pose(side, target, execute=execute, seed=self._display_seed(side))

    def scan_display(self, side):
        neutral, offset = self.display_neutral(side), self.tcp_object(side)
        self.move_display_neutral(side, True)
        seed = self._display_seed(side)
        z_axis = neutral[:3, 2]
        x_axis = neutral[:3, 0]

        # 1) Turn the part 180 degrees about its own Z axis in one motion.
        # A continuous Cartesian roll is not feasible this close to the camera
        # (the wrist hits its limit after a fraction of a degree), so resolve
        # the whole turn with one IK/OMPL move to the opposite orientation.
        self.motion.guard(True)
        turn = int(round(self.display_turn_direction * 180.0))
        self.publish(f'{side} 展示：绕零件 Z 轴一次转 {turn}°')
        target = neutral.copy()
        target[:3, :3] = (Rotation.from_rotvec(z_axis * math.radians(turn)).as_matrix()
                          @ neutral[:3, :3])
        self.motion.pose(side, vector(target @ np.linalg.inv(offset)), execute=True, seed=seed)
        if self.stop_event.wait(self.dwell):
            raise RuntimeError('展示已停止')

        # 2) Fixed tilts about the part's X axis.
        for tilt in self.display_x_tilts_deg:
            self.motion.guard(True)
            self.publish(f'{side} 展示：绕零件 X 轴 {tilt}°')
            target = neutral.copy()
            target[:3, :3] = (Rotation.from_rotvec(x_axis * math.radians(tilt)).as_matrix()
                              @ neutral[:3, :3])
            # Pitching about the TCP X axis is not a pure wrist roll, so a
            # Cartesian path only covers a few percent.  Resolve with IK seeded
            # from the reference posture and return to neutral between tilts.
            self.motion.pose(side, vector(target @ np.linalg.inv(offset)), execute=True, seed=seed)
            if self.stop_event.wait(self.dwell):
                raise RuntimeError('展示已停止')
            self.motion.pose(side, vector(neutral @ np.linalg.inv(offset)), execute=True, seed=seed)

        # 3) Show the part's bottom face toward the camera.
        self.motion.guard(True)
        self.publish(f'{side} 展示：零件底部朝向相机')
        base = self._bottom_view(neutral)
        axis = base[:3, 2]
        settled = False
        for roll in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
            self.motion.guard(True)
            target = base.copy()
            target[:3, :3] = (Rotation.from_rotvec(axis * math.radians(roll)).as_matrix()
                              @ base[:3, :3])
            try:
                # A failed IK raises before any motion is sent, so retrying a
                # rolled target is safe (the mirrored left hand needs this).
                self.motion.pose(side, vector(target @ np.linalg.inv(offset)), execute=True, seed=seed)
            except RuntimeError:
                continue
            settled = True
            break
        if not settled:
            raise RuntimeError('零件底部朝向相机无解')
        if self.stop_event.wait(self.dwell):
            raise RuntimeError('展示已停止')

    def _bottom_view(self, neutral):
        """Point the part's +Z axis directly back toward the head camera."""
        optical = self._camera_optical_transform()
        to_camera = optical[:3, 3] - neutral[:3, 3]
        distance = float(np.linalg.norm(to_camera))
        if distance < 1e-9:
            return neutral
        to_camera /= distance
        from_axis = neutral[:3, 2] / np.linalg.norm(neutral[:3, 2])
        cross = np.cross(from_axis, to_camera)
        dot = float(np.dot(from_axis, to_camera))
        if np.linalg.norm(cross) < 1e-9:
            if dot < 0:
                rotation = Rotation.from_rotvec(neutral[:3, 0] * math.pi)
            else:
                rotation = Rotation.identity()
        else:
            angle = math.atan2(float(np.linalg.norm(cross)), dot)
            rotation = Rotation.from_rotvec(cross / np.linalg.norm(cross) * angle)
        target = neutral.copy()
        target[:3, :3] = rotation.as_matrix() @ neutral[:3, :3]
        return target

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

    def verify_grasp(self, side, timeout=3.0):
        """Accept a grasp only when the measured opening is stable and plausible."""
        # The grippers close to a fixed part-sized gap (not fully closed), so
        # the measured opening is meaningful in mock as well as real mode.
        joint = side + '_left_finger_joint'
        deadline = time.monotonic() + timeout
        stable = []
        last_gap = None
        while time.monotonic() < deadline:
            gap = 2.0 * float(self.motion.guard(True)[joint])
            if self.grasp_gap_min <= gap <= self.grasp_gap_max:
                if last_gap is not None and abs(gap - last_gap) <= 0.0005:
                    stable.append(gap)
                else:
                    stable = [gap]
                if len(stable) >= 5:
                    self.publish(
                        f'{side} 自动夹持成功：实际开度 {gap*1000:.1f} mm')
                    return gap
            else:
                stable = []
            last_gap = gap
            time.sleep(0.05)
        measured = 2.0 * float(self.motion.guard(True)[joint])
        raise RuntimeError(
            f'{side} 自动夹持失败：实际开度 {measured*1000:.1f} mm，'
            f'允许范围 {self.grasp_gap_min*1000:.1f}–{self.grasp_gap_max*1000:.1f} mm')

    def verify_handover_alignment(self):
        poses = self.motion.tcp_poses()
        lateral, angle = validate_handover_centerline(
            poses['left'], poses['right'],
            self.handover_line_tolerance, self.handover_angle_tolerance)
        self.publish(
            f'交接中心线已对准：横向偏差 {lateral*1000:.1f} mm，'
            f'角度偏差 {math.degrees(angle):.1f}°')

    def aligned_handover_target(self):
        """Project the taught receiver TCP onto the donor's live centerline."""
        generated = self.generated_targets('handover_ready')
        taught_left = matrix(generated['left']['tcp'])
        taught_right = matrix(generated['right']['tcp'])
        live_right = matrix(self.motion.tcp_poses()['right'])

        # Preserve the taught axial spacing, but remove all lateral offset.
        taught_axis = taught_right[:3, 2]
        axial_spacing = float(np.dot(
            taught_left[:3, 3] - taught_right[:3, 3], taught_axis))
        donor_axis = live_right[:3, 2]
        target = np.eye(4)
        target[:3, 3] = live_right[:3, 3] + axial_spacing * donor_axis

        # Receiver Z faces donor Z. Preserve its taught roll as closely as
        # possible by projecting the taught X axis onto the new normal plane.
        receiver_z = -donor_axis
        receiver_x = taught_left[:3, 0] - np.dot(
            taught_left[:3, 0], receiver_z) * receiver_z
        norm = float(np.linalg.norm(receiver_x))
        if norm < 1e-6:
            receiver_x = live_right[:3, 0]
            receiver_x -= np.dot(receiver_x, receiver_z) * receiver_z
            norm = float(np.linalg.norm(receiver_x))
        receiver_x /= norm
        receiver_y = np.cross(receiver_z, receiver_x)
        receiver_y /= np.linalg.norm(receiver_y)
        receiver_x = np.cross(receiver_y, receiver_z)
        target[:3, :3] = np.column_stack((receiver_x, receiver_y, receiver_z))
        return vector(target)

    def move_handover_receive(self):
        target = self.aligned_handover_target()
        self.publish('自动对齐左手接取位姿到右手夹爪中心线')
        # The corrected TCP still goes through MoveIt IK, OMPL and collision
        # checking. The donor remains stationary throughout this move.
        self.motion.pose('left', target, execute=True)

    def start_demo(self):
        if not self.commissioned:
            raise RuntimeError('请核对工作台 / 零件尺寸和 TCP 偏移，并在 demo.yaml 设置 scene_and_object_verified: true')
        run_workflow(self)

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

    def skip_service(self, request, response):
        try:
            self.skip_step()
            response.success, response.message = True, 'Skip requested'
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
