"""Offline contract tests. No robot or ROS runtime is contacted."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / 'src/fr3_dual_arm_description'
sys.path.insert(0, str(SHARE))
from fr3_dual_arm_description.model import build_model, controllers, moveit_config, read_yaml, manager_model

spec = importlib.util.spec_from_file_location('ready', ROOT / 'src/fr3_dual_arm_bringup/launch/check_real_ready.py')
ready = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ready)


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.arms = read_yaml(SHARE / 'config/arms.yaml')
        self.hardware = read_yaml(ROOT / 'src/fr3_dual_arm_hardware/config/hardware.example.yaml')
        self.hardware.update(commissioned=True)
        self.hardware['left'].update(robot_ip='192.0.2.4', gripper_index=1)
        self.hardware['right'].update(robot_ip='192.0.2.2', gripper_index=1)

    def model(self, mode):
        return build_model(SHARE, SHARE / 'config/scene.yaml', self.arms, mode=mode, hardware=self.hardware)

    def test_real_entire_actuation_and_feedback_chain(self):
        self.hardware['gripper']['block'] = 0  # user's legacy config
        root = self.model('real')
        mapping = moveit_config(root, self.arms, 'real')['moveit_simple_controller_manager']
        self.assertEqual(len(mapping['controller_names']), 4)
        for side in ('left', 'right'):
            master, follower = f'{side}_left_finger_joint', f'{side}_right_finger_joint'
            systems = ET.fromstring(manager_model(root, side)).findall('ros2_control')
            self.assertEqual(len(systems), 2)
            interface_names = {j.get('name') for s in systems for j in s.findall('joint')}
            self.assertEqual(interface_names, {f'{side}_j{i}' for i in range(1, 7)} | {master})
            hand = next(s for s in systems if s.get('name').endswith('_gripper_system'))
            self.assertEqual(hand.find('hardware/plugin').text, 'fairino_hardware/FairinoGripperHardwareInterface')
            self.assertEqual(hand.find("hardware/param[@name='block']").text, '1')
            self.assertEqual(root.find(f"joint[@name='{follower}']/mimic").get('joint'), master)
            c = controllers('real', side)
            broadcast = c[f'{side}_joint_state_broadcaster']['ros__parameters']['joints']
            self.assertEqual(set(broadcast), interface_names)
            for name in (f'{side}_arm_controller', f'{side}_gripper_controller'):
                self.assertIn(name, c[f'{side}_controller_manager']['ros__parameters'])
                self.assertLessEqual(set(mapping[name]['joints']), interface_names)
            self.assertEqual(mapping[f'{side}_gripper_controller']['action_ns'], 'gripper_cmd')
            self.assertFalse(c[f'{side}_gripper_controller']['ros__parameters']['allow_stalling'])

    def test_mimic_not_an_independent_command_in_any_mode(self):
        for mode in ('mock', 'gazebo', 'real'):
            root = self.model(mode)
            params = moveit_config(root, self.arms, mode)
            self.assertEqual(len(params['robot_description_planning']['joint_limits']), 14)
            for side in ('left', 'right'):
                name = f'{side}_gripper_controller'
                self.assertEqual(params['moveit_simple_controller_manager'][name]['joints'], [f'{side}_left_finger_joint'])
                if mode == 'gazebo':
                    mimic = root.find(f"ros2_control/joint[@name='{side}_right_finger_joint']/param[@name='mimic']")
                    self.assertEqual(mimic.text, f'{side}_left_finger_joint')

    def test_calibrated_limits_match_sdk_range(self):
        self.arms['gripper']['open_gap'] = 0.04
        root = self.model('real')
        for joint in root.findall('joint'):
            if joint.get('type') == 'prismatic':
                self.assertEqual(float(joint.find('limit').get('upper')), 0.02)

    def test_feedback_gate_rejects_missing_stale_nan(self):
        seen = {}
        def message(names, stamp=100, positions=None):
            return SimpleNamespace(name=names, position=positions or [0.1]*len(names),
                                   header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))
        for side in ('left', 'right'):
            ready.record_feedback(message([f'{side}_j{i}' for i in range(1, 7)]), 100.0, seen)
        self.assertEqual(ready.EXPECTED - seen.keys(), {'left_left_finger_joint', 'right_left_finger_joint'})
        fingers = ['left_left_finger_joint', 'right_left_finger_joint']
        ready.record_feedback(message(fingers, stamp=90), 100.0, seen)
        self.assertEqual(len(seen), 12)
        ready.record_feedback(message(fingers, positions=[float('nan'), 0.02]), 100.0, seen)
        self.assertNotIn(fingers[0], seen)
        ready.record_feedback(message(fingers), 100.0, seen)
        self.assertEqual(ready.EXPECTED, seen.keys())

    def test_launch_waits_for_feedback_before_moveit_and_rviz(self):
        # Evaluate the real launch builder with inert ROS launch objects; no processes start.
        class Item:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.__dict__.update(kwargs)
        class Configuration:
            def __init__(self, name): self.name = name
            def perform(self, context): return context[self.name]
        modules = {}
        def module(name, **attrs): modules[name] = SimpleNamespace(**attrs)
        module('launch', LaunchDescription=Item)
        module('launch.actions', **{name: type(name, (Item,), {}) for name in (
            'DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess', 'LogInfo',
            'OpaqueFunction', 'RegisterEventHandler')})
        module('launch.conditions', IfCondition=Item)
        module('launch.event_handlers', OnProcessExit=Item, OnShutdown=Item)
        module('launch.events', Shutdown=Item)
        module('launch.substitutions', LaunchConfiguration=Configuration)
        module('launch_ros.actions', Node=Item)
        module('launch_ros.parameter_descriptions', ParameterValue=Item)
        with tempfile.TemporaryDirectory() as temp:
            driver = Path(temp)
            manifest = driver / 'fairino_hardware.xml'
            manifest.write_text('<library><class name="fairino_hardware/FairinoHardwareInterface"/>'
                                '<class name="fairino_hardware/FairinoGripperHardwareInterface"/></library>')
            hardware_file = driver / 'hardware.yaml'
            hardware_file.write_text(yaml.safe_dump(self.hardware))
            module('ament_index_python.packages',
                   get_package_share_directory=lambda name: str(driver if name.startswith('fairino_hardware') else ROOT / 'src' / name),
                   get_packages_with_prefixes=lambda: {'fairino_hardware_v3_9_7': str(driver)})
            with patch.dict(sys.modules, modules):
                launch_spec = importlib.util.spec_from_file_location('real_launch', ROOT / 'src/fr3_dual_arm_bringup/launch/real.launch.py')
                launch = importlib.util.module_from_spec(launch_spec)
                launch_spec.loader.exec_module(launch)
                for enabled in ('false', 'true'):
                    actions = launch.start({'enable_execution': enabled, 'hardware': str(hardware_file),
                                            'scene': str(SHARE / 'config/scene.yaml'),
                                            'arms': str(SHARE / 'config/arms.yaml')})
                    handlers = [item.args[0] for item in actions if type(item).__name__ == 'RegisterEventHandler']
                    try:
                        self.assertFalse(any(getattr(a, 'executable', '') in ('move_group', 'rviz2') for a in actions))
                        gate = next(h for h in handlers if hasattr(getattr(h, 'target_action', None), 'cmd'))
                        self.assertEqual('--require-actions' in gate.target_action.cmd, enabled == 'true')
                        failure = gate.on_exit(SimpleNamespace(returncode=1), None)
                        self.assertEqual(type(failure[0]).__name__, 'EmitEvent')
                        success = gate.on_exit(SimpleNamespace(returncode=0), None)
                        self.assertEqual([n.executable for n in success], ['move_group', 'rviz2'])
                        mapping = success[0].parameters[0]['moveit_simple_controller_manager']
                        self.assertEqual(len(mapping['controller_names']), 4)
                    finally:
                        next(h for h in handlers if hasattr(h, 'on_shutdown')).on_shutdown(None, None)
                manifest.write_text('<library><class name="fairino_hardware/FairinoHardwareInterface"/></library>')
                with self.assertRaisesRegex(RuntimeError, 'both arm and gripper plugins'):
                    launch.start({'enable_execution': 'true', 'hardware': str(hardware_file)})


if __name__ == '__main__':
    unittest.main()
