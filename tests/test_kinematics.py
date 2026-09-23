"""Offline checks for the numeric IK used by the demo, without ROS."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_description'))
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_grasp'))

from fr3_dual_arm_description.model import build_model, read_yaml
from fr3_dual_arm_grasp.kinematics import (
    build_kinematics, matrix_to_pose, pose_to_matrix)
from fr3_dual_arm_grasp.display_geometry import display_views


REFERENCE_TCP = {
    'left': [0.3529353758780258, 0.20780372384616536, 1.054250226176732,
             -0.026542185373719517, 0.9914130359438571, -0.1279559258919703,
             0.004794328451419095],
    'right': [0.3887845771334418, -0.21058782325760428, 1.051178904251485,
              0.9396660622312608, 0.3364024771899355, 0.06104303926792472,
              -0.011610865092919158],
}


@pytest.fixture(scope='module')
def kinematics():
    share = ROOT / 'src/fr3_dual_arm_description'
    arms = read_yaml(share / 'config/arms.yaml')
    root = build_model(share, share / 'config/scene.yaml', arms, mode='mock')
    return build_kinematics(root, arms), arms


def test_forward_kinematics_matches_urdf_convention(kinematics):
    chains, arms = kinematics
    for side, expected in REFERENCE_TCP.items():
        pose = matrix_to_pose(chains[side].fk(arms[side]['initial']))
        np.testing.assert_allclose(pose, expected, atol=1e-12)


def test_display_turn_is_reachable(kinematics):
    chains, arms = kinematics
    share = ROOT / 'src/fr3_dual_arm_description'
    seed = [math.radians(v) for v in (-109, -137, -107, -28, 94, 18)]
    tcp = chains['right'].fk(seed)
    neutral = tcp.copy()
    z_axis = neutral[:3, 2]
    x_axis = neutral[:3, 0]

    joints = list(seed)
    # Z turn endpoint (direction -1, i.e. -180 degrees).
    maneuvers = [(z_axis, -180.0, 'Z -180'), (x_axis, 30.0, 'X +30'), (x_axis, -30.0, 'X -30')]
    for axis, angle, label in maneuvers:
        target = neutral.copy()
        target[:3, :3] = (Rotation.from_rotvec(np.asarray(axis) * math.radians(angle)).as_matrix()
                          @ neutral[:3, :3])
        solution, error = chains['right'].solve_ik(target, joints)
        assert solution is not None, f'display maneuver {label} is not reachable'
        assert error[0] < 0.005 and error[1] < math.radians(1.0)
        joints = solution


def test_handover_y_axis_is_reachable(kinematics):
    chains, arms = kinematics
    center = np.array([0.35, 0.0, 1.0])
    separation = 0.16
    axis = np.array([0.0, 1.0, 0.0])
    z_right, z_left = axis, -axis
    y_right = np.array([0.0, 0.0, 1.0])
    y_left = np.array([1.0, 0.0, 0.0])
    right_r = np.column_stack((np.cross(y_right, z_right), y_right, z_right))
    left_r = np.column_stack((np.cross(y_left, z_left), y_left, z_left))
    right = np.eye(4)
    left = np.eye(4)
    right[:3, :3], left[:3, :3] = right_r, left_r
    right[:3, 3] = center + axis * separation / 2.0
    left[:3, 3] = center - axis * separation / 2.0
    assert chains['right'].solve_ik(right, arms['right']['initial'])[0] is not None
    assert chains['left'].solve_ik(left, arms['left']['initial'])[0] is not None
