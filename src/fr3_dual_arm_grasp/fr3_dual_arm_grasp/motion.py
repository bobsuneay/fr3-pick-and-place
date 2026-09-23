"""MoveIt-only motion. Blocking methods run in a worker, not ROS callbacks."""
from copy import deepcopy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, RobotState,
                             PositionConstraint, OrientationConstraint, DisplayTrajectory)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetCartesianPath, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


def pose_msg(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, values[:3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, values[3:])
    return pose


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


class DualArmMoveIt:
    def __init__(self, node, feedback, stop, enabled=False, speed=0.1, kinematics=None):
        if not 0 < speed <= 0.8:
            raise ValueError('Demo speed must be > 0 and <= 0.8')
        self.node, self.feedback, self.stop = node, feedback, stop
        self.enabled, self.speed = enabled, speed
        # Offline analytic chains used for a robust numeric IK fallback. The
        # value is a dict of side -> fr3_dual_arm_grasp.kinematics.ArmChain.
        self.kinematics = kinematics or {}
        self.seed_joints = {}
        self.move = ActionClient(node, MoveGroup, '/move_action')
        self.execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.cart = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.valid = node.create_client(GetStateValidity, '/check_state_validity')
        self.display = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        self.active = None
        self.lock = threading.Lock()
        self.fault = False

    def guard(self, execute=False):
        if self.stop.is_set():
            raise RuntimeError('Cancelled; gripper ownership retained')
        if self.fault:
            raise RuntimeError('Motion result uncertain; inspect robot and restart demo node')
        if execute and not self.enabled:
            raise RuntimeError('Execution disabled by launch parameter')
        return self.feedback.snapshot()

    def wait(self, future, timeout=15.0, watch_feedback=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.stop.is_set():
                raise RuntimeError('Cancelled')
            if watch_feedback:
                self.guard()
            if time.monotonic() > deadline:
                raise TimeoutError('ROS request timed out')
            time.sleep(0.02)
        if self.stop.is_set():
            raise RuntimeError('Cancelled')
        result = future.result()
        if result is None:
            raise RuntimeError('ROS request returned no result')
        return result

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('Required MoveIt service unavailable')
        if self.stop.is_set():
            raise RuntimeError('Cancelled')
        return self.wait(client.call_async(request))

    def state(self, values=None):
        values = self.guard() if values is None else values
        state = RobotState()
        state.is_diff = True  # Preserve attached bodies from the scene.
        state.joint_state = JointState()
        state.joint_state.name = list(values)
        state.joint_state.position = list(values.values())
        return state

    def check_state(self, values=None):
        req = GetStateValidity.Request()
        req.robot_state = self.state(values)
        response = self.service(self.valid, req)  # Empty group checks entire robot.
        if not response.valid:
            contacts = ', '.join(f'{c.contact_body_1}/{c.contact_body_2}' for c in response.contacts[:5])
            raise RuntimeError('Robot state in collision or out of bounds: ' + contacts)

    def tcp_poses(self, values=None):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = ['left_gripper_tcp', 'right_gripper_tcp']
        req.robot_state = self.state(values)
        result = self.service(self.fk, req)
        if result.error_code.val != 1 or len(result.pose_stamped) != 2:
            raise RuntimeError(f'FK failed: {result.error_code.val}')
        by_link = dict(zip(result.fk_link_names, result.pose_stamped))
        return {s: pose_values(by_link[s + '_gripper_tcp'].pose) for s in ('left', 'right')}

    def cancel(self):
        self.stop.set()
        with self.lock:
            handle = self.active
        if handle is not None:
            handle.cancel_goal_async()

    def _action(self, client, goal, execute):
        self.guard(execute)
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('MoveIt action server unavailable')
        self.guard(execute)  # Stop/feedback may have changed during discovery.
        pending = client.send_goal_async(goal)
        abandoned = threading.Event()

        def accepted(future):
            try:
                handle = future.result()
                if handle and handle.accepted:
                    with self.lock:
                        self.active = handle
                    if self.stop.is_set() or abandoned.is_set():
                        handle.cancel_goal_async()
            except Exception:
                self.fault = True

        pending.add_done_callback(accepted)
        result_future = None
        try:
            handle = self.wait(pending, 10.0, watch_feedback=True)
            if not handle.accepted:
                raise RuntimeError('MoveIt rejected goal')
            with self.lock:
                self.active = handle
            result_future = handle.get_result_async()
            response = self.wait(result_future, 180.0, watch_feedback=True)
            if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != 1:
                raise RuntimeError(f'MoveIt failed: action={response.status}, code={response.result.error_code.val}')
            return response.result
        except Exception:
            abandoned.set()
            with self.lock:
                handle = self.active
            if handle is not None:
                handle.cancel_goal_async()
            # Wait for terminal acknowledgement before permitting another command.
            deadline = time.monotonic() + 5.0
            while result_future is not None and not result_future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if result_future is None or not result_future.done():
                self.fault = True
            raise
        finally:
            with self.lock:
                self.active = None

    def _goal(self, group, constraints, execute):
        self.guard(execute)
        self.check_state()
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.pipeline_id, req.planner_id = 'ompl', 'RRTConnectkConfigDefault'
        req.num_planning_attempts, req.allowed_planning_time = 5, 10.0
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = self.speed
        req.start_state.is_diff = True
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self._action(self.move, goal, execute)
        if not execute:
            self.show(result.trajectory_start, result.planned_trajectory)
        return result

    def joints(self, targets, group, execute=False, verify=True):
        expected_tcp = None
        if execute and verify:
            # Some Fairino firmware reports a successful trajectory while its
            # joint feedback uses a different calibration/ordering.  Keep the
            # target TCP as the independent execution check.
            expected_state = self.guard()
            expected_state.update({name: float(value) for name, value in targets.items()})
            expected_tcp = self.tcp_poses(expected_state)
        constraint = Constraints()
        for name, position in targets.items():
            if not math.isfinite(position):
                raise ValueError('Non-finite joint target')
            jc = JointConstraint()
            jc.joint_name, jc.position, jc.weight = name, float(position), 1.0
            jc.tolerance_above = jc.tolerance_below = 0.0005 if 'finger' in name else 0.01
            constraint.joint_constraints.append(jc)
        result = self._goal(group, constraint, execute)
        if execute and verify:
            try:
                self.verify_targets(targets)
            except RuntimeError as joint_error:
                if expected_tcp is None or not self.verify_tcp_targets(expected_tcp):
                    raise
                self.node.get_logger().warning(
                    'Joint feedback differs from target, but TCP reached; '
                    'continuing: %s', joint_error)
        return result

    @staticmethod
    def _pose_error(actual, target):
        position = math.sqrt(sum((float(actual[i]) - float(target[i])) ** 2
                                 for i in range(3)))
        dot = abs(sum(float(actual[i]) * float(target[i]) for i in range(3, 7)))
        dot = min(1.0, max(0.0, dot))
        orientation = 2.0 * math.acos(dot)
        return position, orientation

    def verify_tcp_targets(self, targets, timeout=5.0):
        deadline = time.monotonic() + timeout
        while True:
            actual = self.tcp_poses()
            if all(self._pose_error(actual[side], target)[0] <= 0.02 and
                   self._pose_error(actual[side], target)[1] <= math.radians(5.0)
                   for side, target in targets.items()):
                return True
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)

    def verify_targets(self, targets):
        # The vendor controller can report trajectory completion before the
        # measured joints have finished settling. Keep the independent safety
        # check, but allow a short settling window and identify every offender.
        deadline = time.monotonic() + 5.0
        while True:
            values = self.guard()
            errors = [(name, float(target), float(values[name]),
                       abs(float(values[name])-float(target)),
                       0.0006 if 'finger' in name else 0.015)
                      for name, target in targets.items()]
            failed = [item for item in errors if item[3] > item[4]]
            if not failed:
                return
            if time.monotonic() > deadline:
                details = []
                for name, target, actual, error, limit in sorted(
                        failed, key=lambda item: item[3], reverse=True):
                    if 'finger' in name:
                        details.append(
                            f'{name}: target={target*1000:.2f} mm, '
                            f'measured={actual*1000:.2f} mm, '
                            f'error={error*1000:.2f} mm, limit={limit*1000:.2f} mm')
                    else:
                        details.append(
                            f'{name}: target={math.degrees(target):.2f} deg, '
                            f'measured={math.degrees(actual):.2f} deg, '
                            f'error={math.degrees(error):.2f} deg, '
                            f'limit={math.degrees(limit):.2f} deg')
                raise RuntimeError('Measured target not reached after 5 s; ' + '; '.join(details))
            time.sleep(0.05)

    def gripper(self, side, gap, execute=False):
        # Gripper path is planned too; there is no direct action/SDK bypass.
        # A zero-gap adaptive grasp intentionally stalls on the part, so its
        # success is checked from the measured opening by the workflow.
        return self.joints(
            {side + '_left_finger_joint': gap / 2.0},
            side + '_gripper', execute, verify=gap > 1e-6)

    def poses(self, targets, group, execute=False):
        """Plan one or both saved TCP targets through MoveIt's IK and OMPL."""
        if not targets or any(side not in ('left', 'right') for side in targets):
            raise ValueError('TCP targets must contain left and/or right')
        joint_targets = {}
        for side, values in targets.items():
            joint_targets.update(self._ik_joints(side, values))
        return self.joints(joint_targets, group, execute)

    def _fk_pose(self, side, joints):
        """Resolve one TCP pose from six joint values via MoveIt FK."""
        values = self.guard()
        for index, value in enumerate(joints, 1):
            values[f'{side}_j{index}'] = float(value)
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = [side + '_gripper_tcp']
        request.robot_state = self.state(values)
        result = self.service(self.fk, request)
        if result.error_code.val != 1 or len(result.pose_stamped) != 1:
            raise RuntimeError(f'FK failed for {side}')
        return pose_values(result.pose_stamped[0].pose)

    def _numeric_ik_joints(self, side, values, seed=None):
        """Robust numeric IK fallback; returns a joint dict or ``None``."""
        chain = getattr(self, 'kinematics', {}).get(side)
        if chain is None:
            return None
        from fr3_dual_arm_grasp.kinematics import pose_to_matrix
        target = pose_to_matrix(values)
        current = self.guard()
        seeds = []
        if seed is not None:
            seeds.append([float(x) for x in seed])
        seeds.append([current[f'{side}_j{i}'] for i in range(1, 7)])
        taught = getattr(self, 'seed_joints', {}).get(side)
        if taught is not None:
            seeds.append(taught)
        for seed in seeds:
            solution, _error = chain.solve_ik(target, seed)
            if solution is None:
                continue
            try:
                actual = self._fk_pose(side, solution)
            except Exception:
                return None
            position_error, orientation_error = self._pose_error(actual, values)
            if position_error <= 0.005 and orientation_error <= math.radians(1.0):
                return {f'{side}_j{i}': float(solution[i - 1]) for i in range(1, 7)}
        return None

    def _ik_joints(self, side, values, seed=None):
        numeric = self._numeric_ik_joints(side, values, seed=seed)
        if numeric is not None:
            try:
                self.check_state(numeric)
            except RuntimeError:
                numeric = None
            else:
                return numeric
        request = GetPositionIK.Request()
        request.ik_request.group_name = side + '_arm'
        request.ik_request.ik_link_name = side + '_gripper_tcp'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose_msg(values)
        request.ik_request.robot_state = self.state()
        request.ik_request.avoid_collisions = True
        result = self.service(self.ik, request)
        if result.error_code.val != 1:
            raise RuntimeError(f'IK failed for {side} TCP target: code={result.error_code.val}')
        joints = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        return {f'{side}_j{i}': joints[f'{side}_j{i}'] for i in range(1, 7)}

    def pose(self, side, values, execute=False, linear=False, seed=None):
        if linear:
            return self.linear(side, values, execute)
        # Resolve the TCP target with MoveIt's dedicated IK service first.
        # The resulting joint target is then planned through OMPL, so collision
        # checking remains active and IK failures are reported separately.
        return self.joints(self._ik_joints(side, values, seed=seed), side + '_arm', execute)

    def show(self, state, trajectory):
        msg = DisplayTrajectory()
        msg.trajectory_start, msg.trajectory = state, [trajectory]
        self.display.publish(msg)

    def linear(self, side, values, execute=False):
        return self.cartesian(side, [values], execute)

    def cartesian(self, side, waypoints, execute=False):
        self.guard(execute)
        self.check_state()
        req = GetCartesianPath.Request()
        req.header.frame_id, req.group_name, req.link_name = 'world', side + '_arm', side + '_gripper_tcp'
        planning_start = self.guard()
        req.start_state = self.state(planning_start)
        req.waypoints = [pose_msg(values) for values in waypoints]
        req.max_step, req.jump_threshold, req.avoid_collisions = 0.002, 2.0, True
        result = self.service(self.cart, req)
        if result.error_code.val != 1 or result.fraction < 0.999999:
            raise RuntimeError(f'Cartesian path incomplete ({result.fraction:.1%}); no execution')
        trajectory = deepcopy(result.solution)
        points = trajectory.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError('Cartesian path empty')
        speed = self.speed  # One immutable speed per planned segment.
        previous = 0.0
        for index, point in enumerate(points):
            if (len(point.positions) != len(trajectory.joint_trajectory.joint_names)
                    or any(not math.isfinite(x) for x in list(point.positions) + list(point.velocities) + list(point.accelerations))):
                raise RuntimeError('Invalid Cartesian trajectory values')
            seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if not math.isfinite(seconds) or (index and seconds <= previous):
                raise RuntimeError('Cartesian service did not return a timed trajectory')
            if index and max(abs(a-b) for a, b in zip(points[index-1].positions, point.positions)) > 0.15:
                raise RuntimeError('Cartesian joint jump > 0.15 rad')
            previous = seconds
            seconds /= speed
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds-int(seconds))*1e9)
            point.velocities = [v*speed for v in point.velocities]
            point.accelerations = [a*speed*speed for a in point.accelerations]
        self.show(result.start_state, trajectory)
        if execute:
            current = self.guard(True)
            if any(abs(current[n]-v) > (0.0005 if 'finger' in n else 0.01)
                   for n, v in planning_start.items()):
                raise RuntimeError('Robot moved during Cartesian planning; replan')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            self._action(self.execute, goal, True)
            final_joints = dict(zip(
                trajectory.joint_trajectory.joint_names,
                points[-1].positions))
            try:
                self.verify_targets(final_joints)
            except RuntimeError as joint_error:
                if not self.verify_tcp_targets({side: waypoints[-1]}):
                    raise
                self.node.get_logger().warning(
                    'Cartesian TCP reached despite joint feedback mismatch; '
                    'continuing: %s', joint_error)
        return trajectory
