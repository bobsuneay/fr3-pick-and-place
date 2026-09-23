"""Offline arm kinematics for robust TCP-to-joint resolution.

This module has no ROS dependency so it can be exercised by the offline test
suite and imported by :mod:`fr3_dual_arm_grasp.motion` as an alternative to
MoveIt's KDL IK service.  All transforms follow the URDF ``rpy`` convention
``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``, which is also the convention used by
:func:`scipy.spatial.transform.Rotation.from_euler` with ``'xyz'``.
"""

from dataclasses import dataclass, field
import math

import numpy as np
from scipy.spatial.transform import Rotation


def fixed_transform(xyz, rpy):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def pose_to_matrix(pose):
    """Convert ``[x, y, z, qx, qy, qz, qw]`` into a 4x4 transform."""
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    transform[:3, 3] = np.asarray(pose[:3], dtype=float)
    return transform


def matrix_to_pose(transform):
    """Convert a 4x4 transform into ``[x, y, z, qx, qy, qz, qw]``."""
    return (transform[:3, 3].tolist() +
            Rotation.from_matrix(transform[:3, :3]).as_quat().tolist())


@dataclass
class ArmChain:
    side: str
    names: list
    lower: np.ndarray
    upper: np.ndarray
    base: np.ndarray = field(default_factory=lambda: np.eye(4))
    # Each entry is (fixed parent->child transform, revolute axis or None).
    chain: list = field(default_factory=list)

    def __post_init__(self):
        self.lower = np.asarray(self.lower, dtype=float)
        self.upper = np.asarray(self.upper, dtype=float)

    def fk(self, q):
        """Return the world -> gripper_tcp transform for six joint values."""
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError('expected six joint values')
        transform = self.base.copy()
        index = 0
        for origin, axis in self.chain:
            transform = transform @ origin
            if axis is not None:
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                transform = transform @ rotation
                index += 1
        return transform

    def solve_ik(self, target, seed, max_iter=250, position_tol=1e-3,
                 orientation_tol=math.radians(0.5)):
        """Damped least-squares IK for a 4x4 world-frame TCP target.

        Returns ``(joints, error)`` where ``error`` is ``(position_m,
        orientation_rad)`` on success, or ``(None, None)`` when the solver does
        not converge.  Joints are clipped to the URDF limits with a small margin.
        """
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (4, 4) or seed.shape != (6,):
            raise ValueError('target must be 4x4 and seed a six-vector')
        margin = 1e-3
        lower = self.lower + margin
        upper = self.upper - margin
        q = np.clip(seed.copy(), lower, upper)
        damping = 0.05
        previous_norm = math.inf
        epsilon = 1e-5

        for _ in range(max_iter):
            transform = self.fk(q)
            rotation = transform[:3, :3]
            position = transform[:3, 3]
            delta_position = target[:3, 3] - position
            delta_rotation = target[:3, :3] @ rotation.T
            angle = _rotation_angle(delta_rotation)
            axis = _rotation_axis(delta_rotation, angle)
            # Both the position error and the Jacobian are expressed in the
            # world frame, so the DLS step is consistent.
            delta_orientation = rotation @ axis * angle
            error = np.concatenate([delta_position, delta_orientation])

            position_error = float(np.linalg.norm(delta_position))
            orientation_error = float(np.linalg.norm(delta_orientation))
            if position_error <= position_tol and orientation_error <= orientation_tol:
                return q, (position_error, orientation_error)
            norm = float(np.linalg.norm(error))

            jacobian = np.zeros((6, 6))
            for column in range(6):
                plus = q.copy()
                minus = q.copy()
                plus[column] += epsilon
                minus[column] -= epsilon
                plus_transform = self.fk(plus)
                minus_transform = self.fk(minus)
                jacobian[:3, column] = (
                    plus_transform[:3, 3] - minus_transform[:3, 3]) / (2 * epsilon)
                delta_rot = plus_transform[:3, :3] @ minus_transform[:3, :3].T
                column_angle = _rotation_angle(delta_rot)
                column_axis = _rotation_axis(delta_rot, column_angle)
                jacobian[3:, column] = (
                    plus_transform[:3, :3] @ column_axis * column_angle) / (2 * epsilon)

            gram = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
            try:
                step = jacobian.T @ np.linalg.solve(gram, error)
            except np.linalg.LinAlgError:
                break
            q = np.clip(q + step, lower, upper)
            damping = damping * 0.7 if norm < previous_norm else damping * 1.5
            previous_norm = norm

        return None, None


def _rotation_angle(matrix):
    cosine = (np.trace(matrix) - 1.0) / 2.0
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def _rotation_axis(matrix, angle):
    if angle < 1e-9:
        return np.zeros(3)
    scale = 2.0 * math.sin(angle)
    if abs(scale) < 1e-12:
        # Angle is close to pi; recover the axis from the symmetric part.
        return _axis_from_symmetric(matrix)
    return np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ]) / scale


def _axis_from_symmetric(matrix):
    diagonal = np.array([matrix[0, 0], matrix[1, 1], matrix[2, 2]])
    axis = np.sqrt(np.maximum(0.0, (diagonal + 1.0) / 2.0))
    if axis[0] >= axis[1] and axis[0] >= axis[2]:
        axis = np.array([axis[0], (matrix[0, 1] + matrix[1, 0]) / (4 * axis[0]),
                         (matrix[0, 2] + matrix[2, 0]) / (4 * axis[0])])
    elif axis[1] >= axis[2]:
        axis = np.array([(matrix[0, 1] + matrix[1, 0]) / (4 * axis[1]), axis[1],
                         (matrix[1, 2] + matrix[2, 1]) / (4 * axis[1])])
    else:
        axis = np.array([(matrix[0, 2] + matrix[2, 0]) / (4 * axis[2]),
                         (matrix[1, 2] + matrix[2, 1]) / (4 * axis[2]), axis[2]])
    norm = np.linalg.norm(axis)
    return axis / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0])


def build_arm_chain(root, side, arms):
    """Build an :class:`ArmChain` from the assembled robot model XML."""
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        limit = joint.find('limit')
        joints[joint.get('name')] = {
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'xyz': [float(v) for v in (origin.get('xyz') or '0 0 0').split()],
            'rpy': [float(v) for v in (origin.get('rpy') or '0 0 0').split()],
            'axis': [float(v) for v in (axis.get('xyz') if axis is not None else '1 0 0').split()],
            'lower': float(limit.get('lower')) if limit is not None else 0.0,
            'upper': float(limit.get('upper')) if limit is not None else 0.0,
            'type': joint.get('type'),
        }

    ordered = []
    current = side + '_gripper_tcp'
    while current != side + '_base_link':
        found = False
        for name, joint in joints.items():
            if joint['child'] == current:
                ordered.append((name, joint))
                current = joint['parent']
                found = True
                break
        if not found:
            raise ValueError('arm chain is missing a joint for link ' + current)
    ordered.reverse()

    names = []
    lower = []
    upper = []
    chain = []
    for _name, joint in ordered:
        origin = fixed_transform(joint['xyz'], joint['rpy'])
        if joint['type'] == 'revolute':
            names.append(_name)
            lower.append(joint['lower'])
            upper.append(joint['upper'])
            chain.append((origin, np.asarray(joint['axis'], dtype=float)))
        else:
            chain.append((origin, None))
    if len(names) != 6:
        raise ValueError('expected six revolute joints in the arm chain')

    return ArmChain(
        side=side,
        names=names,
        lower=lower,
        upper=upper,
        base=fixed_transform(arms[side]['xyz'], arms[side]['rpy']),
        chain=chain,
    )


def build_kinematics(root, arms):
    return {side: build_arm_chain(root, side, arms) for side in ('left', 'right')}
