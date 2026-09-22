"""Unit tests for merging two controller-manager feedback streams."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / 'src/fr3_dual_arm_bringup/fr3_dual_arm_bringup/state_bridge.py'


class JointState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, _name):
        self.parameters = {}
        self.subscriptions = []
        self.publisher = FakePublisher()

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, _kind, topic, qos):
        assert topic == '/joint_states'
        assert qos == 10
        return self.publisher

    def create_subscription(self, _kind, topic, callback, _qos):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, _period, callback):
        return SimpleNamespace(callback=callback)

    def get_topic_names_and_types(self):
        return []

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))

    def get_logger(self):
        return SimpleNamespace(info=lambda _message: None)


class StateBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {
            'rclpy': ModuleType('rclpy'),
            'rclpy.node': ModuleType('rclpy.node'),
            'rclpy.qos': ModuleType('rclpy.qos'),
            'sensor_msgs': ModuleType('sensor_msgs'),
            'sensor_msgs.msg': ModuleType('sensor_msgs.msg'),
        }
        modules['rclpy.node'].Node = FakeNode
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['sensor_msgs.msg'].JointState = JointState
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('state_bridge', BRIDGE)
            cls.bridge_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.bridge_module)

    def test_merges_fresh_arm_and_gripper_feedback(self):
        bridge = self.bridge_module.JointStateBridge()
        self.assertEqual(bridge.publisher.messages, [])

        with patch.object(self.bridge_module.time, 'monotonic', return_value=10.0):
            bridge._update(SimpleNamespace(name=['left_j1', 'left_left_finger_joint'],
                                           position=[0.1, 0.02]))
        with patch.object(self.bridge_module.time, 'monotonic', return_value=10.1):
            bridge._update(SimpleNamespace(name=['right_j1', 'right_left_finger_joint'],
                                           position=[-0.1, 0.01]))

        result = bridge.publisher.messages[-1]
        self.assertEqual(result.header.stamp, 'stamp')
        self.assertEqual(dict(zip(result.name, result.position)), {
            'left_j1': 0.1,
            'left_left_finger_joint': 0.02,
            'right_j1': -0.1,
            'right_left_finger_joint': 0.01,
        })

    def test_never_refreshes_or_republishes_stale_feedback(self):
        bridge = self.bridge_module.JointStateBridge()
        bridge.parameters['max_age_seconds'] = 0.5
        with patch.object(self.bridge_module.time, 'monotonic', return_value=1.0):
            bridge._update(SimpleNamespace(name=['left_j1'], position=[0.3]))
        count = len(bridge.publisher.messages)
        bridge._publish(2.0)
        self.assertEqual(len(bridge.publisher.messages), count)

        with patch.object(self.bridge_module.time, 'monotonic', return_value=2.0):
            bridge._update(SimpleNamespace(name=['right_j1'], position=[0.4]))
        result = bridge.publisher.messages[-1]
        self.assertEqual(result.name, ['right_j1'])
        self.assertEqual(result.position, [0.4])

    def test_rejects_malformed_and_non_finite_messages(self):
        bridge = self.bridge_module.JointStateBridge()
        bridge._update(SimpleNamespace(name=['left_j1'], position=[]))
        bridge._update(SimpleNamespace(name=['left_j1'], position=[float('nan')]))
        self.assertEqual(bridge.publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
