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


def estimate_bolt(points, cfg):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = np.all(np.isfinite(p), axis=1)
    keep &= np.all((p >= cfg['roi_min']) & (p <= cfg['roi_max']), axis=1)
    p = p[keep]
    if len(p) < cfg['min_points']:
        raise ValueError('Not enough bolt points above the calibrated tabletop')
    if len(p) > cfg['max_points']:
        raise ValueError('ROI too large or tabletop not removed; refine ROI/table_z')

    tree = cKDTree(p)
    visited = np.zeros(len(p), dtype=bool)
    clusters = []
    for seed in range(len(p)):
        if visited[seed]:
            continue
        stack, indices = [seed], []
        visited[seed] = True
        while stack:
            i = stack.pop()
            indices.append(i)
            for j in tree.query_ball_point(p[i], cfg['cluster_radius']):
                if not visited[j]:
                    visited[j] = True
                    stack.append(j)
        if len(indices) >= cfg['min_points']:
            clusters.append(p[indices])

    candidates = []
    for cloud in clusters:
        xy = cloud[:, :2]
        eig, vec = np.linalg.eigh(np.cov(xy.T))
        if eig[-1] < 2.5 * max(eig[0], 1e-12):
            continue
        axis = np.r_[vec[:, -1], 0.0]
        if axis[0] < 0:
            axis = -axis
        lateral = np.cross([0, 0, 1], axis)
        along, across = cloud @ axis, cloud @ lateral
        lo, hi = np.quantile(along, [0.01, 0.99])
        width = np.quantile(across, 0.99) - np.quantile(across, 0.01)
        if not (0.8 * cfg['bolt_length'] <= hi - lo <= 1.2 * cfg['bolt_length'] and
                0.75 * cfg['shaft_radius'] <= width <= 2.7 * cfg['head_radius']):
            continue
        end_length = 0.8 * cfg['head_length']
        ends = [across[along < lo + end_length], across[along > hi - end_length]]
        spans = [np.ptp(e) if len(e) >= 4 else 0 for e in ends]
        resolved = min(spans) > 0 and max(spans) / min(spans) > 1.15
        if resolved and spans[0] > spans[1]:
            axis, lateral = -axis, -lateral
            along, across = cloud @ axis, cloud @ lateral
            lo, hi = np.quantile(along, [0.01, 0.99])
        center = axis * (lo + hi) / 2 + lateral * np.mean(np.quantile(across, [0.01, 0.99]))
        center[2] = np.clip(
            np.quantile(cloud[:, 2], 0.98) - cfg['head_radius'],
            cfg['table_z'] + cfg['shaft_radius'],
            cfg['table_z'] + cfg['head_radius'])
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack((axis, lateral, [0, 0, 1]))
        pose[:3, 3] = center
        candidates.append(Estimate(pose, np.array([hi - lo, width, 2 * cfg['head_radius']]), cloud, resolved))

    if len(candidates) != 1:
        raise ValueError(f'Expected one isolated bolt, found {len(candidates)}; narrow ROI')
    if not candidates[0].head_resolved:
        raise ValueError('Bolt head/tail ambiguous; need a clearer point cloud before handover')
    return candidates[0]


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
