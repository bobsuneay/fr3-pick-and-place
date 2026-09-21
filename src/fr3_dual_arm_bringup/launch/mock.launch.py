from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from fr3_dual_arm_description.model import (
    SIDES, build_model, controllers, manager_model, moveit_config, read_yaml)


def start(context):
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    enable_execution = arg('enable_execution') == 'true'
    scene = Path(arg('scene')).expanduser().resolve()
    arms = read_yaml(Path(arg('arms')).expanduser())

    temp = tempfile.TemporaryDirectory(prefix='fr3_dual_arm_mock_')
    run = Path(temp.name)
    root = build_model(description_share, scene, arms, mode='mock')
    robot_xml = ET.tostring(root, encoding='unicode')

    description = {
        'robot_description': ParameterValue(robot_xml, value_type=str),
        'use_sim_time': False,
    }
    moveit_params = {
        **description,
        **moveit_config(root, arms, mode='mock'),
        'allow_trajectory_execution': enable_execution,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    moveit_params['robot_description_semantic'] = ParameterValue(
        moveit_params['robot_description_semantic'], value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[description],
        output='screen',
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_params],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[moveit_params],
        arguments=['-d', str(description_share / 'config' / 'dual_arm.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    managers = []
    spawners = []
    for side in SIDES:
        manager_name = side + '_controller_manager'
        side_file = run / (side + '_controllers.yaml')
        side_file.write_text(yaml.safe_dump(controllers('mock', side)), encoding='utf-8')
        manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            arguments=['--ros-args', '-r', f'controller_manager:__node:={manager_name}'],
            parameters=[{
                'robot_description': ParameterValue(manager_model(root, side), value_type=str),
                'use_sim_time': False,
            }, str(side_file)],
            output='screen',
        )
        managers.append(manager)
        for suffix in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
            args = [
                f'{side}_{suffix}',
                '-c', f'/{manager_name}',
                '--controller-manager-timeout', '120',
            ]
            if suffix != 'joint_state_broadcaster' and not enable_execution:
                args.append('--inactive')
            spawners.append(Node(
                package='controller_manager',
                executable='spawner',
                prefix='/usr/bin/python3',
                arguments=args,
                output='screen',
            ))

    def success(actions, stage):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{stage} failed'))]
            return actions
        return callback

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: temp.cleanup())),
    ]
    for current, following in zip(spawners, spawners[1:]):
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=current,
            on_exit=success([following], 'controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=spawners[-1],
        on_exit=success([move_group], 'last controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=move_group,
        on_exit=[EmitEvent(event=Shutdown(reason='move_group exited'))])))
    for manager in managers:
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='required manager exited'))])))

    return handlers + [rsp] + managers + [spawners[0], rviz]


def generate_launch_description():
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    return LaunchDescription([
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'scene',
            default_value=str(description_share / 'config/scene.yaml')),
        DeclareLaunchArgument(
            'arms',
            default_value=str(description_share / 'config/arms.yaml')),
        OpaqueFunction(function=start),
    ])
