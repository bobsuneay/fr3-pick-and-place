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
from fr3_dual_arm_grasp.display_geometry import camera_neutral, display_views


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


def test_display_views_are_sequentially_reachable(kinematics):
    chains, arms = kinematics
    share = ROOT / 'src/fr3_dual_arm_description'
    scene = read_yaml(share / 'config/scene.yaml')
    camera = scene['camera']
    # A representative taught grasp: above the right-arm plane, TCP Z down.
    taught = pose_to_matrix(
        [0.45, -0.1, 0.82] + Rotation.from_euler('xyz', [math.pi, 0, 0]).as_quat().tolist())
    neutral = camera_neutral(camera, matrix_to_pose(taught), np.eye(4), distance=0.30)

    neutral_solution, neutral_error = chains['right'].solve_ik(
        neutral, arms['right']['initial'])
    assert neutral_solution is not None
    assert neutral_error[0] < 0.005 and neutral_error[1] < math.radians(1.0)
    views = display_views('right')
    for angles in views:
        target = neutral.copy()
        target[:3, :3] = neutral[:3, :3] @ Rotation.from_euler(
            'xyz', angles, degrees=True).as_matrix()
        solution, error = chains['right'].solve_ik(target, neutral_solution)
        assert solution is not None, f'view {angles} is not reachable'
        assert error[0] < 0.005 and error[1] < math.radians(1.0)
