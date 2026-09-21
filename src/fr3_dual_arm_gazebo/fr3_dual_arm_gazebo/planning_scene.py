"""Publish the same table geometry to MoveIt for collision-aware planning."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from fr3_dual_arm_gazebo.world_builder import load_scene, table_boxes


def main(args=None):
    rclpy.init(args=args)
    node = Node('fr3_dual_arm_static_scene')
    node.declare_parameter('scene_file', '')
    scene_file = node.get_parameter('scene_file').value
    if not scene_file:
        node.get_logger().error('scene_file parameter is required')
        return 1
    scene = load_scene(Path(scene_file))
    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=60.0):
        node.get_logger().error('MoveIt /apply_planning_scene unavailable after 60 s')
        return 1
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    for name, size, position in table_boxes(scene):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = name
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(x) for x in size]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(x) for x in position]
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        request.scene.world.collision_objects.append(obj)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done() or not future.result().success:
        node.get_logger().error('MoveIt did not acknowledge the static scene')
        return 1
    node.get_logger().info('Published tabletop and legs to MoveIt planning scene')
    return 0
