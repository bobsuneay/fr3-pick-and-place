"""Exercise production cancellation/result logic with futures, without ROS."""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
import threading
from types import ModuleType, SimpleNamespace
import sys

import pytest


@pytest.fixture
def motion(monkeypatch):
    exports = {
        'action_msgs.msg': ['GoalStatus'], 'geometry_msgs.msg': ['Pose'],
        'moveit_msgs.action': ['MoveGroup', 'ExecuteTrajectory'],
        'moveit_msgs.msg': ['Constraints', 'JointConstraint', 'RobotState',
                            'PositionConstraint', 'OrientationConstraint', 'DisplayTrajectory'],
        'moveit_msgs.srv': ['GetPositionFK', 'GetCartesianPath', 'GetStateValidity'],
        'rclpy.action': ['ActionClient'], 'sensor_msgs.msg': ['JointState'],
        'shape_msgs.msg': ['SolidPrimitive'],
    }
    for name, classes in exports.items():
        module = ModuleType(name)
        for cls in classes:
            setattr(module, cls, type(cls, (), {}))
        if name == 'action_msgs.msg':
            module.GoalStatus.STATUS_SUCCEEDED = 4
        monkeypatch.setitem(sys.modules, name, module)
    file = Path(__file__).resolve().parents[1] / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/motion.py'
    spec = importlib.util.spec_from_file_location('motion_under_test', file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = object.__new__(module.DualArmMoveIt)
    result.stop = threading.Event()
    result.lock = threading.Lock()
    result.active, result.fault, result.enabled = None, False, True
    result.feedback = SimpleNamespace(snapshot=lambda: {})
    result._test_module = module
    return result


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self, status=4, code=1):
        self.cancelled = 0
        self.result = completed(SimpleNamespace(status=status, result=SimpleNamespace(error_code=SimpleNamespace(val=code))))

    def cancel_goal_async(self):
        self.cancelled += 1
        return completed(None)

    def get_result_async(self):
        return self.result


def client(pending):
    return SimpleNamespace(wait_for_server=lambda timeout_sec: True, send_goal_async=lambda goal: pending)


@pytest.mark.parametrize('status,code', [(6, 1), (5, 1), (4, -1), (4, -4)])
def test_action_status_and_moveit_code_must_both_succeed(motion, status, code):
    handle = Handle(status, code)
    with pytest.raises(RuntimeError, match='MoveIt failed'):
        motion._action(client(completed(handle)), object(), True)
    assert handle.cancelled == 1
    assert motion.active is None


def test_late_goal_acceptance_after_timeout_is_cancelled_and_fault_latched(motion):
    pending = Future()
    def timeout(*args, **kwargs):
        raise TimeoutError('injected')
    motion.wait = timeout
    with pytest.raises(TimeoutError):
        motion._action(client(pending), object(), True)
    handle = Handle()
    pending.set_result(handle)
    assert handle.cancelled == 1
    assert motion.fault
    with pytest.raises(RuntimeError, match='uncertain'):
        motion.guard(True)


def test_disabled_execution_sends_no_action(motion):
    motion.enabled = False
    def forbidden(goal):
        pytest.fail('disabled execution dispatched an action')
    endpoint = SimpleNamespace(wait_for_server=lambda timeout_sec: True, send_goal_async=forbidden)
    with pytest.raises(RuntimeError, match='disabled'):
        motion._action(endpoint, object(), True)


def test_success_result_and_explicit_cancel(motion):
    handle = Handle()
    result = motion._action(client(completed(handle)), object(), True)
    assert result.error_code.val == 1
    motion.active = handle
    motion.cancel()
    assert motion.stop.is_set() and handle.cancelled == 1


def test_stale_feedback_prevents_dispatch(motion):
    def stale():
        raise RuntimeError('stale feedback')
    motion.feedback.snapshot = stale
    with pytest.raises(RuntimeError, match='stale feedback'):
        motion._action(client(Future()), object(), True)


def test_stop_during_server_discovery_prevents_dispatch(motion):
    def discovery(timeout_sec):
        motion.stop.set()
        return True
    def forbidden(goal):
        pytest.fail('motion sent after stop during server discovery')
    endpoint = SimpleNamespace(wait_for_server=discovery, send_goal_async=forbidden)
    with pytest.raises(RuntimeError, match='Cancelled'):
        motion._action(endpoint, object(), True)


def test_measured_target_failure_identifies_joint_and_error(motion, monkeypatch):
    motion.guard = lambda execute=False: {'right_j3': 0.0}
    ticks = iter([10.0, 16.0])
    monkeypatch.setattr(motion._test_module.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match=r'right_j3: target=1.15 deg.*error=1.15 deg.*limit=0.86 deg'):
        motion.verify_targets({'right_j3': 0.02})
