"""Read-only startup gate: require fresh real joints and (if enabled) actions."""
import argparse
import math
import time

EXPECTED = {f'{side}_j{i}' for side in ('left', 'right') for i in range(1, 7)} | {
    'left_left_finger_joint', 'right_left_finger_joint'}


def record_feedback(message, now, seen):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    if len(message.name) != len(message.position) or not -0.1 <= now - stamp <= 1.0:
        return
    for name, position in zip(message.name, message.position):
        if name in EXPECTED and math.isfinite(position):
            seen[name] = stamp
        elif name in EXPECTED:
            seen.pop(name, None)


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from sensor_msgs.msg import JointState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-actions', action='store_true')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('fr3_real_readiness')
    seen = {}
    clients = {}
    subscription = node.create_subscription(
        JointState, '/joint_states',
        lambda message: record_feedback(message, node.get_clock().now().nanoseconds * 1e-9, seen),
        qos_profile_sensor_data)
    if args.require_actions:
        for side in ('left', 'right'):
            for action, suffix in ((FollowJointTrajectory, 'arm_controller/follow_joint_trajectory'),
                                   (GripperCommand, 'gripper_controller/gripper_cmd')):
                name = f'/{side}_{suffix}'
                clients[name] = ActionClient(node, action, name)
    deadline = time.monotonic() + args.timeout
    missing, unavailable = sorted(EXPECTED), list(clients)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            missing = sorted(name for name in EXPECTED
                             if name not in seen or not -0.1 <= now - seen[name] <= 1.0)
            unavailable = [name for name, client in clients.items() if not client.server_is_ready()]
            if not missing and not unavailable:
                node.get_logger().info('READY: 12 arm + 2 gripper master joints are fresh; '
                                       'requested action endpoints are available.')
                return 0
        node.get_logger().error(f'NOT READY: missing/stale joint feedback={missing}; '
                                f'unavailable actions={unavailable}. Check sourced workspace, '
                                'SDK plugins and both controller managers.')
        return 1
    finally:
        for client in clients.values():
            client.destroy()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
