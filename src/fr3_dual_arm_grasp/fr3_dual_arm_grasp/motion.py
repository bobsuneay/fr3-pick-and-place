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
from moveit_msgs.srv import GetPositionFK, GetCartesianPath, GetStateValidity
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
    def __init__(self, node, feedback, stop, enabled=False, speed=0.1):
        if not 0 < speed <= 0.3:
            raise ValueError('Demo speed must be > 0 and <= 0.3')
        self.node, self.feedback, self.stop = node, feedback, stop
        self.enabled, self.speed = enabled, speed
        self.move = ActionClient(node, MoveGroup, '/move_action')
        self.execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
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

    def joints(self, targets, group, execute=False):
        constraint = Constraints()
        for name, position in targets.items():
            if not math.isfinite(position):
                raise ValueError('Non-finite joint target')
            jc = JointConstraint()
            jc.joint_name, jc.position, jc.weight = name, float(position), 1.0
            jc.tolerance_above = jc.tolerance_below = 0.0002 if 'finger' in name else 0.002
            constraint.joint_constraints.append(jc)
        result = self._goal(group, constraint, execute)
        if execute:
            self.verify_targets(targets)
        return result

    def verify_targets(self, targets):
        deadline = time.monotonic() + 2.0
        while True:
            values = self.guard()
            if all(abs(values[n]-v) <= (0.0006 if 'finger' in n else 0.015) for n, v in targets.items()):
                return
            if time.monotonic() > deadline:
                raise RuntimeError('Controller success but measured target was not reached')
            time.sleep(0.05)

    def gripper(self, side, gap, execute=False):
        # Gripper path is planned too; there is no direct action/SDK bypass.
        return self.joints({side + '_left_finger_joint': gap / 2.0}, side + '_gripper', execute)

    def pose(self, side, values, execute=False, linear=False):
        if linear:
            return self.linear(side, values, execute)
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id, pc.link_name, pc.weight = 'world', side + '_gripper_tcp', 1.0
        region = SolidPrimitive()
        region.type, region.dimensions = SolidPrimitive.SPHERE, [0.001]
        pc.constraint_region.primitives = [region]
        pc.constraint_region.primitive_poses = [pose_msg(values)]
        oc = OrientationConstraint()
        oc.header.frame_id, oc.link_name, oc.weight = 'world', side + '_gripper_tcp', 1.0
        oc.orientation = pose_msg(values).orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01
        constraints.position_constraints, constraints.orientation_constraints = [pc], [oc]
        return self._goal(side + '_arm', constraints, execute)

    def show(self, state, trajectory):
        msg = DisplayTrajectory()
        msg.trajectory_start, msg.trajectory = state, [trajectory]
        self.display.publish(msg)

    def linear(self, side, values, execute=False):
        self.guard(execute)
        self.check_state()
        req = GetCartesianPath.Request()
        req.header.frame_id, req.group_name, req.link_name = 'world', side + '_arm', side + '_gripper_tcp'
        planning_start = self.guard()
        req.start_state = self.state(planning_start)
        req.waypoints = [pose_msg(values)]
        req.max_step, req.jump_threshold, req.avoid_collisions = 0.002, 2.0, True
        result = self.service(self.cart, req)
        if result.error_code.val != 1 or result.fraction < 0.999999:
            raise RuntimeError(f'Cartesian path incomplete ({result.fraction:.1%}); no execution')
        trajectory = deepcopy(result.solution)
        points = trajectory.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError('Cartesian path empty')
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
            seconds /= self.speed
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds-int(seconds))*1e9)
            point.velocities = [v*self.speed for v in point.velocities]
            point.accelerations = [a*self.speed*self.speed for a in point.accelerations]
        self.show(result.start_state, trajectory)
        if execute:
            current = self.guard(True)
            if any(abs(current[n]-v) > (0.0005 if 'finger' in n else 0.01)
                   for n, v in planning_start.items()):
                raise RuntimeError('Robot moved during Cartesian planning; replan')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            self._action(self.execute, goal, True)
            self.verify_targets(dict(zip(trajectory.joint_trajectory.joint_names, points[-1].positions)))
        return trajectory
