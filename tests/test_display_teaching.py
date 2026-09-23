"""Geometry and mode transitions without ROS or a robot connection."""
from pathlib import Path
import sys
import ast
import threading
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/fr3_dual_arm_grasp'))
from fr3_dual_arm_grasp.display_geometry import display_views, matrix, vector, camera_neutral, object_path
from fr3_dual_arm_grasp.teaching import TeachingMode


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('z_sign', [-1, 1])
def test_camera_center_and_all_interpolated_views_with_offset(side, z_sign):
    camera = dict(xyz=[.06, 0, 1.44], rpy=[0, np.pi/4, 0])
    offset = matrix([.035, .012, .08, 0, 0, 0, 1])
    taught = [.4, -.2, 1.0] + Rotation.from_euler('xyz', [.2, .3, -.4]).as_quat().tolist()
    neutral = camera_neutral(camera, taught, offset)
    expected = np.array(camera['xyz']) + Rotation.from_euler('xyz', camera['rpy']).apply([.3, 0, 0])
    np.testing.assert_allclose(neutral[:3, 3], expected, atol=1e-12)
    expected_rotation = (matrix(taught) @ offset)[:3, :3] @ Rotation.from_euler('x', 45, degrees=True).as_matrix()
    np.testing.assert_allclose(neutral[:3, :3], expected_rotation, atol=1e-12)
    views = display_views(side, z_sign)
    assert len(views) == 25
    assert all(x == 0 and y == 0 for x, y, z in views)
    assert [v[2] for v in views] == list(range(0, z_sign * 361, z_sign * 15))
    previous = neutral
    for angles in views[1:]:
        target = neutral.copy()
        target[:3, :3] = neutral[:3, :3] @ Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        poses = object_path(previous, target, offset)
        for pose in poses:
            obj = matrix(pose) @ offset
            np.testing.assert_allclose(obj[:3, 3], expected, atol=1e-12)
            delta = Rotation.from_matrix(previous[:3, :3]).inv() * Rotation.from_matrix(obj[:3, :3])
            assert delta.magnitude() <= np.deg2rad(15) + 1e-12
            previous = obj
        np.testing.assert_allclose(previous, target, atol=1e-12)
    np.testing.assert_allclose(previous, neutral, atol=1e-12)


def scan_method():
    # Execute the production method without importing ROS/node startup code.
    source = Path(__file__).resolve().parents[1] / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/app.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DemoApp')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'scan_display')
    namespace = dict(np=np, Rotation=Rotation, display_views=display_views,
                     object_path=object_path, vector=vector)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['scan_display']


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('fail', [False, True])
def test_actual_scan_uses_adjacent_cartesian_views_and_stops_on_failure(side, fail):
    neutral = matrix([.3, 0, 1, 0, 0, 0, 1])
    offset = matrix([.03, .01, .08, 0, 0, 0, 1])
    calls, messages, entries = [], [], []
    def cartesian(arm, poses, execute):
        calls.append((arm, poses, execute))
        if fail:
            raise RuntimeError('Cartesian path incomplete')
    app = SimpleNamespace(display_neutral=lambda: neutral, tcp_object=lambda arm: offset,
        demo_config={}, move_display_neutral=lambda *args: entries.append(args),
        motion=SimpleNamespace(guard=lambda execute: None, cartesian=cartesian,
            pose=lambda *args: pytest.fail('Inspection must not dispatch free-path poses')),
        publish=messages.append, stop_event=threading.Event(), dwell=0)
    if fail:
        with pytest.raises(RuntimeError, match=r'2/25.*15.*Cartesian path incomplete'):
            scan_method()(app, side)
        assert len(calls) == 1
    else:
        scan_method()(app, side)
        assert len(calls) == 24
        for i, (arm, poses, execute) in enumerate(calls, 1):
            assert arm == side and execute is True
            target = neutral.copy()
            target[:3, :3] = Rotation.from_euler('z', 15*i, degrees=True).as_matrix()
            np.testing.assert_allclose(matrix(poses[-1]) @ offset, target, atol=1e-12)
    assert entries == [(side, True)]


def test_modes_stop_both_before_drag_and_exit_both_before_reactivation():
    events = []
    mode = TeachingMode(lambda s, active: events.append(('switch', s, active)),
                        lambda s, cmd, value: events.append((cmd, s, value)))
    mode.enter()
    assert mode.state == 'teaching'
    assert events[:2] == [('switch', 'left', False), ('switch', 'right', False)]
    events.clear()
    mode.leave()
    assert mode.state == 'motion'
    assert events[-2:] == [('switch', 'left', True), ('switch', 'right', True)]
    assert events.index(('DragTeachSwitch', 'right', 0)) < events.index(('switch', 'left', True))


def test_partial_drag_failure_never_restarts_controllers():
    events = []
    def command(side, cmd, value):
        if side == 'right':
            raise RuntimeError('injected')
    mode = TeachingMode(lambda s, active: events.append(active), command)
    with pytest.raises(RuntimeError):
        mode.enter()
    assert mode.state == 'fault' and not any(events)


def test_failed_reactivation_releases_both_sides():
    events = []
    def switch(side, active):
        events.append((side, active))
        if side == 'right' and active:
            raise RuntimeError('injected')
    mode = TeachingMode(switch, lambda *args: None)
    with pytest.raises(RuntimeError):
        mode.leave()
    assert mode.state == 'fault'
    assert events[-2:] == [('left', False), ('right', False)]
