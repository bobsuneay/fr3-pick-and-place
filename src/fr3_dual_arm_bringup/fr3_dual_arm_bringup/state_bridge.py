"""Merge independent ros2_control feedback streams into global /joint_states."""
from collections import OrderedDict
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    """Forward only fresh measured joints; never fabricate or refresh stale data."""
    def __init__(self):
        super().__init__('fr3_joint_state_bridge')
        self.declare_parameter('input_topics', [
            '/left_joint_state_broadcaster/joint_states',
            '/right_joint_state_broadcaster/joint_states',
            '/left_controller_manager/left_joint_state_broadcaster/joint_states',
            '/right_controller_manager/right_joint_state_broadcaster/joint_states',
            '/left_controller_manager/joint_states',
            '/right_controller_manager/joint_states',
        ])
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('max_age_seconds', 0.5)
        self._states = OrderedDict()
        self._topics = set()
        self._input_subscriptions = []
        self._output = self.get_parameter('output_topic').value
        # Reliable output connects to both reliable and best-effort consumers.
        self._publisher = self.create_publisher(JointState, self._output, 10)
        for topic in self.get_parameter('input_topics').value:
            self._subscribe(topic)
        self.create_timer(1.0, self._discover)
        self.get_logger().info('Joint feedback bridge ready; waiting for both controller managers')

    def _subscribe(self, topic):
        if topic == self._output or topic in self._topics:
            return
        self._topics.add(topic)
        self._input_subscriptions.append(self.create_subscription(
            JointState, topic, self._update, qos_profile_sensor_data))

    def _discover(self):
        for topic, types in self.get_topic_names_and_types():
            if (topic != self._output and topic.endswith('/joint_states') and
                    'sensor_msgs/msg/JointState' in types):
                self._subscribe(topic)

    def _update(self, message):
        if len(message.name) != len(message.position):
            return
        received = time.monotonic()
        for name, position in zip(message.name, message.position):
            value = float(position)
            if name and math.isfinite(value):
                self._states[name] = (value, received)
        self._publish(received)

    def _publish(self, now=None):
        now = time.monotonic() if now is None else now
        max_age = float(self.get_parameter('max_age_seconds').value)
        fresh = {name: value for name, (value, received) in self._states.items()
                 if 0.0 <= now - received <= max_age}
        if not fresh:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(fresh)
        message.position = list(fresh.values())
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
