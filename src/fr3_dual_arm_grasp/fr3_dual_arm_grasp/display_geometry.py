"""Object-centred inspection, matching fr3-sim5 core.centered_views/interpolate_object."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# Inspection angles are generated per arm because the two arms have mirrored
# joint limits.  Z rotates in one direction only; X reverses for the left arm.
_DISPLAY_ANGLES = (15, 60, 120, 180)
VIEWS = [[0, 0, 0]] + [[angle, 0, 0] for angle in _DISPLAY_ANGLES] + [[0, 0, angle] for angle in _DISPLAY_ANGLES]


def display_views(side, z_sign=1):
    """Return IK-friendly one-way X/Z inspection poses for one arm."""
    x_sign = 1 if side == 'right' else -1
    z_sign = 1 if z_sign >= 0 else -1
    return ([[0, 0, 0], [-90, 0, 0]] +
            [[0, 0, z_sign * angle] for angle in range(15, 361, 15)])


def matrix(pose):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def vector(transform):
    return transform[:3, 3].tolist() + Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()


def camera_neutral(camera, taught_tcp, tcp_object, distance=0.30):
    # Model's optical joint is RPY(-pi/2, 0, -pi/2): optical +Z is camera_link +X.
    optical = (Rotation.from_euler('xyz', camera['rpy']) *
               Rotation.from_euler('xyz', [-math.pi/2, 0, -math.pi/2]))
    obj = matrix(taught_tcp) @ tcp_object
    # Display pose: rotate the gripper about its local X axis by +45 degrees.
    obj[:3, :3] = obj[:3, :3] @ Rotation.from_euler('x', 45, degrees=True).as_matrix()
    obj[:3, 3] = np.asarray(camera['xyz']) + optical.apply([0, 0, distance])
    return obj


def object_path(start, end, tcp_object):
    """SLERP the object, not TCP, keeping its centre fixed during inspection."""
    rotations = Rotation.from_matrix([start[:3, :3], end[:3, :3]])
    angle = (rotations[0].inv() * rotations[1]).magnitude()
    # Keep adjacent orientation changes small so MoveIt's IK can stay on the
    # same solution branch during the inspection rotation.  The previous
    # 0.04-rad angular step could jump across a difficult wrist configuration.
    # Adjacent orientation waypoints are at most 15 degrees apart.
    count = max(1, math.ceil(np.linalg.norm(end[:3, 3]-start[:3, 3])/.002), math.ceil(angle/math.radians(15)))
    slerp = Slerp([0, 1], rotations)
    inverse = np.linalg.inv(tcp_object)
    poses = []
    for fraction in np.linspace(0, 1, count+1)[1:]:
        obj = np.eye(4)
        obj[:3, 3] = (1-fraction)*start[:3, 3] + fraction*end[:3, 3]
        obj[:3, :3] = slerp(fraction).as_matrix()
        poses.append(vector(obj @ inverse))
    return poses

