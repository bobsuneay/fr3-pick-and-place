"""Geometry and mode transitions without ROS or a robot connection."""
from pathlib import Path
import sys
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/fr3_dual_arm_grasp'))
from fr3_dual_arm_grasp.display_geometry import (
    display_views, matrix, vector, camera_neutral, object_path)
from fr3_dual_arm_grasp.teaching import TeachingMode


def test_display_views_is_one_direction_full_turn():
    assert display_views('right') == list(range(15, 180, 15)) + [180]
    assert display_views('left', 30) == [30, 60, 90, 120, 150, 180]


def test_camera_neutral_center_sits_on_optical_axis():
    camera = dict(xyz=[.06, 0, 1.44], rpy=[0, np.pi / 4, 0])
    offset = matrix([.035, .012, .08, 0, 0, 0, 1])
    taught = [.4, -.2, 1.0] + Rotation.from_euler('xyz', [.2, .3, -.4]).as_quat().tolist()
    neutral = camera_neutral(camera, taught, offset)
    expected = np.array(camera['xyz']) + Rotation.from_euler('xyz', camera['rpy']).apply([.3, 0, 0])
    np.testing.assert_allclose(neutral[:3, 3], expected, atol=1e-12)
    expected_rotation = ((matrix(taught) @ offset)[:3, :3]
                         @ Rotation.from_euler('x', 45, degrees=True).as_matrix())
    np.testing.assert_allclose(neutral[:3, :3], expected_rotation, atol=1e-12)


def test_object_path_preserves_centre_and_interpolates_orientation():
    offset = matrix([.035, .012, .08, 0, 0, 0, 1])
    start = matrix([.4, -.2, 1.0, 0, 0, 0, 1])
    end = start.copy()
    end[:3, :3] = start[:3, :3] @ Rotation.from_euler('z', 60, degrees=True).as_matrix()
    poses = object_path(start, end, offset)
    assert poses
    for pose in poses:
        np.testing.assert_allclose((matrix(pose) @ offset)[:3, 3], start[:3, 3], atol=1e-12)
    np.testing.assert_allclose(matrix(poses[-1]) @ offset, end, atol=1e-12)


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
