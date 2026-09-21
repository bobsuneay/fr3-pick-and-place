"""Launch the dual-arm FR3 cell in Gazebo with MoveIt and RViz."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from fr3_dual_arm_description.model import (
    SIDES, build_model, controllers, moveit_config, read_yaml)
from fr3_dual_arm_gazebo.world_builder import load_scene, world_xml


def start(context):
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    arg = lambda name: LaunchConfiguration(name).perform(context)

    enable_execution = arg('enable_execution') == 'true'
    scene_path = Path(arg('scene')).expanduser().resolve()
    arms_path = Path(arg('arms')).expanduser().resolve()
    scene = load_scene(scene_path)
    arms = read_yaml(arms_path)

    temp = tempfile.TemporaryDirectory(prefix='fr3_dual_arm_gazebo_')
    run = Path(temp.name)
    controller_file = run / 'gazebo_controllers.yaml'
    controller_file.write_text(yaml.safe_dump(controllers()), encoding='utf-8')

    root = build_model(description_share, scene_path, arms, mode='gazebo',
                       controller_file=controller_file)
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename')
        if not uri.startswith('package://fr3_dual_arm_description/'):
            raise ValueError(f'Unexpected mesh URI: {uri}')
        relative = uri.removeprefix('package://fr3_dual_arm_description/')
        if not (description_share / relative).is_file():
            raise FileNotFoundError(uri)

    robot_xml = ET.tostring(root, encoding='unicode')
    (run / 'robot.urdf').write_text(robot_xml, encoding='utf-8')

    description = {
        'robot_description': ParameterValue(robot_xml, value_type=str),
        'use_sim_time': True,
    }
    moveit_params = {
        **description,
        **moveit_config(root, arms),
        'allow_trajectory_execution': enable_execution,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    moveit_params['robot_description_semantic'] = ParameterValue(
        moveit_params['robot_description_semantic'], value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[description],
        output='screen',
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_params],
        output='screen',
    )
    scene_node = Node(
        package='fr3_dual_arm_gazebo',
        executable='publish_scene',
        prefix='/usr/bin/python3',
        parameters=[{'scene_file': str(scene_path), 'use_sim_time': True}],
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

    spawners = []
    for side in SIDES:
        for suffix in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
            args = [
                f'{side}_{suffix}',
                '-c', '/controller_manager',
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
        on_exit=success([move_group, scene_node], 'last controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=scene_node,
        on_exit=success([rviz], 'static planning scene'))))
    for process in (rsp, move_group):
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Required node exited'))])))

    world = run / 'cell.world'
    world.write_text(world_xml(scene), encoding='utf-8')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': str(world),
            'gui': arg('gui'),
            'pause': 'false',
        }.items())
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix='/usr/bin/python3',
        arguments=[
            '-entity', 'fr3_dual_arm',
            '-topic', 'robot_description',
            '-timeout', '120',
        ],
        output='screen',
    )
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=success([spawners[0]], 'Gazebo entity spawn'))))

    return handlers + [
        LogInfo(msg='Starting FR3 dual-arm Gazebo simulation'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        rsp,
        gazebo,
        spawn,
    ]


def generate_launch_description():
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    return LaunchDescription([
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'scene',
            default_value=str(description_share / 'config/scene.yaml')),
        DeclareLaunchArgument(
            'arms',
            default_value=str(description_share / 'config/arms.yaml')),
        OpaqueFunction(function=start),
    ])
