"""Regression tests for standalone upright-cylinder perception."""
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_grasp'))
from fr3_dual_arm_grasp.perception import estimate_upright_cylinder


def test_dense_table_is_removed_before_cylinder_clustering():
    center = np.array([0.50, -0.48])
    radius, table_z, height = 0.008, 0.750, 0.035
    angle = np.linspace(0, 2*np.pi, 48, endpoint=False)
    levels = np.linspace(table_z + .001, table_z + height, 18)
    cylinder = np.array([
        [center[0] + radius*np.cos(a), center[1] + radius*np.sin(a), z]
        for z in levels for a in angle
    ])
    x = np.linspace(.38, .62, 100)
    y = np.linspace(-.60, -.36, 100)
    table = np.array([[px, py, table_z] for px in x for py in y])
    cfg = {
        'roi_min': [.38, -.60, .750], 'roi_max': [.62, -.36, .82],
        'table_z_m': table_z, 'table_clearance_m': .001,
        'height_m': height, 'radius_m': radius,
        'min_points': 20, 'cluster_radius_m': .010,
        'voxel_size_m': .0025, 'preferred_xy': center,
    }

    found = estimate_upright_cylinder(np.vstack((table, cylinder)), cfg)

    assert np.allclose(found.pose[:2, 3], center, atol=.002)
    assert table_z < found.pose[2, 3] < table_z + height
