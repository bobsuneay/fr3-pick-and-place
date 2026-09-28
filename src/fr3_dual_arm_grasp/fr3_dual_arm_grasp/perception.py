"""ROS-independent bolt point-cloud perception and grasp geometry.

These helpers are extracted from ``fr3_bolt_inspection_cell.core`` so the
application package does not depend on Gazebo or the old monolithic package.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


@dataclass
class Estimate:
    pose: np.ndarray
    dimensions: np.ndarray
    points: np.ndarray
    head_resolved: bool


def estimate_upright_cylinder(points, cfg):
    """Find isolated upright cylinder centres in the right-hand bin point cloud."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    roi_min, roi_max = np.asarray(cfg['roi_min']), np.asarray(cfg['roi_max'])
    keep = np.all(np.isfinite(p), axis=1) & np.all((p >= roi_min) & (p <= roi_max), axis=1)
    p = p[keep]
    if len(p):
        # D405 clouds can contain hundreds of thousands of pixels. Voxelize
        # before neighborhood clustering to bound CPU and memory.
        voxel=float(cfg.get('voxel_size_m',.0025))
        _, indices=np.unique(np.floor((p-roi_min)/voxel).astype(np.int32),axis=0,return_index=True)
        p=p[np.sort(indices)]
    if len(p) < int(cfg.get('min_points', 20)):
        raise ValueError('右侧 ROI 中圆柱点数不足；检查相机视野、深度距离和 ROI')
    tree = cKDTree(p)
    visited = np.zeros(len(p), dtype=bool)
    candidates = []
    for seed in range(len(p)):
        if visited[seed]:
            continue
        visited[seed] = True
        stack, indices = [seed], []
        while stack:
            i = stack.pop(); indices.append(i)
            for j in tree.query_ball_point(p[i], float(cfg.get('cluster_radius_m', .010))):
                if not visited[j]:
                    visited[j] = True; stack.append(j)
        if len(indices) < int(cfg.get('min_points', 20)):
            continue
        cloud = p[indices]
        low, high = np.quantile(cloud[:, 2], [.03, .97])
        height = high-low
        if not .65*cfg['height_m'] <= height <= 1.35*cfg['height_m']:
            continue
        xy = cloud[:, :2]
        # Fit a circle to the visible cylinder surface; its median is biased
        # toward the camera when only a partial arc is visible.
        design=np.column_stack((2*xy[:,0],2*xy[:,1],np.ones(len(xy))))
        rhs=np.sum(xy*xy,axis=1)
        fit,_,_,_=np.linalg.lstsq(design,rhs,rcond=None)
        center_xy=fit[:2]
        radial=np.linalg.norm(xy-center_xy,axis=1)
        radius=float(np.median(radial))
        if not .45*cfg['radius_m'] <= radius <= 1.8*cfg['radius_m']:
            continue
        center = np.r_[center_xy, (low+high)/2]
        pose = np.eye(4); pose[:3, 3] = center
        candidates.append((center, pose, cloud))
    if not candidates:
        raise ValueError('未在右手盒格内识别到竖直圆柱')
    # Choose an exposed object nearest the configured camera-facing/front edge.
    preferred = np.asarray(cfg['preferred_xy'], dtype=float)
    selected = min(candidates, key=lambda c: np.linalg.norm(c[0][:2]-preferred))
    return Estimate(selected[1], np.array([cfg['height_m'],2*cfg['radius_m'],2*cfg['radius_m']]), selected[2], True)



def grasp_in_object(offset, below=False):
    result = np.eye(4)
    result[:3, :3] = [[0, 1, 0], [-1 if below else 1, 0, 0], [0, 0, 1 if below else -1]]
    result[:3, 3] = [offset, 0, 0]
    return result


def perpendicular_receiver_grasp(offset, donor_grasp):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('x', np.pi / 2).as_matrix() @ donor_grasp[:3, :3]
    result[:3, 3] = [offset, 0, 0]
    return result


def table_pick_tcp(object_pose, axial_offset, depth_offset):
    result = np.asarray(object_pose, dtype=float) @ grasp_in_object(axial_offset)
    result = result.copy()
    result[2, 3] -= depth_offset
    return result


def centered_views(center, neutral_rotation, object_tcp, views):
    for angles in views:
        obj = np.eye(4)
        obj[:3, 3] = center
        obj[:3, :3] = neutral_rotation @ Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        yield obj, obj @ object_tcp


def interpolate_object(a, b, object_tcp, step=0.002, angle_step=0.04):
    rotations = Rotation.from_matrix([a[:3, :3], b[:3, :3]])
    angle = (rotations[0].inv() * rotations[1]).magnitude()
    n = max(1, math.ceil(np.linalg.norm(b[:3, 3] - a[:3, 3]) / step), math.ceil(angle / angle_step))
    slerp = Slerp([0, 1], rotations)
    output = []
    for f in np.linspace(0, 1, n + 1)[1:]:
        obj = np.eye(4)
        obj[:3, 3] = (1 - f) * a[:3, 3] + f * b[:3, 3]
        obj[:3, :3] = slerp(f).as_matrix()
        output.append(obj @ object_tcp)
    return output
