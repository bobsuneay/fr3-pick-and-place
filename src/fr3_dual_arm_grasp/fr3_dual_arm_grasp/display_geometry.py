"""Object-centred inspection, matching fr3-sim5 core.centered_views/interpolate_object."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

VIEWS = [[0, 0, 0], [0, 0, -30], [0, 15, 0], [0, -15, 0],
         [0, 35, 0], [0, -35, 0], [15, 0, 0], [-15, 0, 0],
         [60, 0, 0], [-60, 0, 0], [120, 0, 0], [-120, 0, 0],
         [180, 0, 0], [0, 0, 60], [0, 0, -60], [0, 0, 120],
         [0, 0, -120], [0, 0, 180]]


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
    obj[:3, 3] = np.asarray(camera['xyz']) + optical.apply([0, 0, distance])
    return obj


def object_path(start, end, tcp_object):
    """SLERP the object, not TCP, keeping its centre fixed during inspection."""
    rotations = Rotation.from_matrix([start[:3, :3], end[:3, :3]])
    angle = (rotations[0].inv() * rotations[1]).magnitude()
    count = max(1, math.ceil(np.linalg.norm(end[:3, 3]-start[:3, 3])/.002), math.ceil(angle/.04))
    slerp = Slerp([0, 1], rotations)
    inverse = np.linalg.inv(tcp_object)
    poses = []
    for fraction in np.linspace(0, 1, count+1)[1:]:
        obj = np.eye(4)
        obj[:3, 3] = (1-fraction)*start[:3, 3] + fraction*end[:3, 3]
        obj[:3, :3] = slerp(fraction).as_matrix()
        poses.append(vector(obj @ inverse))
    return poses
