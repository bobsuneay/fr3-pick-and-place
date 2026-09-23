"""Acknowledged static scene and tracked workpiece ownership."""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import (CollisionObject, AttachedCollisionObject,
                             PlanningSceneComponents, AllowedCollisionEntry)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from fr3_dual_arm_gazebo.world_builder import table_boxes
from .motion import pose_msg


def matrix(pose):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def vector(transform):
    return transform[:3, 3].tolist() + Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()


def finger_links(side):
    return [side + '_left_finger', side + '_right_finger']


def box_object(name, dimensions, pose, frame='world', shape='box'):
    obj = CollisionObject()
    obj.id, obj.header.frame_id, obj.operation = name, frame, CollisionObject.ADD
    primitive = SolidPrimitive()
    if shape == 'cylinder':
        primitive.type = SolidPrimitive.CYLINDER
        # shape_msgs CYLINDER expects [height, radius]; ``dimensions`` is
        # [height, diameter, diameter] from the demo config.
        primitive.dimensions = [float(dimensions[0]), float(dimensions[1]) / 2.0]
    else:
        primitive.type, primitive.dimensions = SolidPrimitive.BOX, list(map(float, dimensions))
    obj.primitives, obj.primitive_poses = [primitive], [pose_msg(pose)]
    return obj


class DemoScene:
    OBJECT = 'fr3_demo_workpiece'

    def __init__(self, node, motion, scene, dimensions, tcp_offset,
                 show_workpiece=False, shape='box'):
        self.motion, self.scene = motion, scene
        self.dimensions, self.offset = dimensions, tcp_offset
        self.shape = shape
        self.show_workpiece = bool(show_workpiece)
        self.owner, self.local_pose, self.world_pose = None, None, None
        self.touch_sides = []
        self.apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.ready = False

    def diff(self):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = req.scene.robot_state.is_diff = True
        return req

    def commit(self, req):
        if not self.motion.service(self.apply, req).success:
            raise RuntimeError('MoveIt rejected planning scene update')

    def initialize(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        existing = self.motion.service(self.get, req).scene.robot_state.attached_collision_objects
        if any(x.object.id == self.OBJECT for x in existing):
            raise RuntimeError('Previous demo still has an attached workpiece; recover before initializing')
        update = self.diff()
        for name, size, xyz in table_boxes(self.scene):
            update.scene.world.collision_objects.append(box_object(name, size, list(xyz) + [0, 0, 0, 1]))
        self.commit(update)
        self.ready = True

    def allow(self, sides, table=False):
        # Read-modify-write preserves every unrelated collision pair. The only
        # allowances this application owns involve its own workpiece.
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = deepcopy(self.motion.service(self.get, req).scene.allowed_collision_matrix)
        names = [self.OBJECT, 'table_top'] + finger_links('left') + finger_links('right')
        for name in names:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                row = AllowedCollisionEntry()
                row.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(row)
        object_index = acm.entry_names.index(self.OBJECT)
        allowed = {link for side in sides for link in finger_links(side)}
        if table:
            allowed.add('table_top')
        for index, name in enumerate(acm.entry_names):
            value = name in allowed
            acm.entry_values[object_index].enabled[index] = value
            acm.entry_values[index].enabled[object_index] = value
        update = self.diff()
        update.scene.allowed_collision_matrix = acm
        self.commit(update)

    def place_initial(self, grasp_pose):
        if not self.ready:
            self.initialize()
        self.world_pose = vector(matrix(grasp_pose) @ matrix(self.offset))
        if self.show_workpiece:
            self.allow(['right'], table=True)
            req = self.diff()
            req.scene.world.collision_objects = [box_object(self.OBJECT, self.dimensions, self.world_pose, shape=self.shape)]
            self.commit(req)

    def current_world(self):
        if self.owner:
            tcp = self.motion.tcp_poses()[self.owner]
            return vector(matrix(tcp) @ matrix(self.local_pose))
        return self.world_pose

    def attached(self, side, local, touch_sides):
        attached = AttachedCollisionObject()
        attached.link_name = side + '_gripper_tcp'
        attached.object = box_object(
            self.OBJECT, self.dimensions, local, attached.link_name, self.shape)
        attached.touch_links = [link for s in touch_sides for link in finger_links(s)]
        return attached

    def attach(self, side, transfer=False):
        world = self.current_world()
        tcp = self.motion.tcp_poses()[side]
        local = vector(np.linalg.inv(matrix(tcp)) @ matrix(world))
        touch = ['left', 'right'] if transfer else [side]
        if self.show_workpiece:
            self.allow(touch)
            req = self.diff()
            if self.owner:
                old = AttachedCollisionObject()
                old.link_name, old.object.id, old.object.operation = self.owner + '_gripper_tcp', self.OBJECT, CollisionObject.REMOVE
                req.scene.robot_state.attached_collision_objects.append(old)
            else:
                old = CollisionObject()
                old.id, old.operation = self.OBJECT, CollisionObject.REMOVE
                req.scene.world.collision_objects.append(old)
            req.scene.robot_state.attached_collision_objects.append(self.attached(side, local, touch))
            self.commit(req)
        self.owner, self.local_pose, self.touch_sides = side, local, touch

    def set_touch(self, sides):
        self.allow(sides)
        if self.owner:
            req = self.diff()
            req.scene.robot_state.attached_collision_objects = [self.attached(self.owner, self.local_pose, sides)]
            self.commit(req)
        self.touch_sides = list(sides)

    def detach(self):
        world = self.current_world()
        if self.show_workpiece:
            req = self.diff()
            old = AttachedCollisionObject()
            old.link_name, old.object.id, old.object.operation = self.owner + '_gripper_tcp', self.OBJECT, CollisionObject.REMOVE
            req.scene.robot_state.attached_collision_objects = [old]
            req.scene.world.collision_objects = [box_object(self.OBJECT, self.dimensions, world, shape=self.shape)]
            self.commit(req)
        self.owner, self.local_pose, self.world_pose = None, None, world

    def clear_after_manual_recovery(self):
        # Called only by the dedicated operator recovery command after physical
        # recovery. It never opens a gripper or moves the robot.
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        bodies = self.motion.service(self.get, query).scene.robot_state.attached_collision_objects
        req = self.diff()
        for body in bodies:
            if body.object.id == self.OBJECT:
                body.object.operation = CollisionObject.REMOVE
                req.scene.robot_state.attached_collision_objects.append(body)
        old = CollisionObject()
        old.id, old.operation = self.OBJECT, CollisionObject.REMOVE
        req.scene.world.collision_objects.append(old)
        self.commit(req)
        self.allow([])
        self.owner = self.local_pose = self.world_pose = None
