from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from fr3_dual_arm_description.model import (
    SIDES, build_model, controllers, manager_model, moveit_config,
    read_yaml, validate_hardware)


def start(context):
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    if arg('enable_execution') not in ('true', 'false'):
        raise ValueError('enable_execution must be true or false')
    enable_execution = arg('enable_execution') == 'true'
    hardware = validate_hardware(read_yaml(Path(arg('hardware')).expanduser()))
    required = {'fairino_hardware/FairinoHardwareInterface',
                'fairino_hardware/FairinoGripperHardwareInterface'}
    exporters = {}
    for package in get_packages_with_prefixes():
        if package.startswith('fairino_hardware'):
            manifest = Path(get_package_share_directory(package)) / 'fairino_hardware.xml'
            if manifest.exists():
                names = {e.get('name') for e in ET.parse(manifest).iter('class')}
                if required & names:
                    exporters[package] = names
    if set(exporters) != {hardware['driver_package']} or not required <= exporters.get(hardware['driver_package'], set()):
        raise RuntimeError('Source exactly one patched fairino_hardware_v3_9_7 exporting '
                           f'both arm and gripper plugins; found {exporters}. '
                           'Run scripts/apply_fairino_patches.sh, rebuild and source install/setup.bash.')
    scene = Path(arg('scene')).expanduser().resolve()
    arms = read_yaml(Path(arg('arms')).expanduser())

    temp = tempfile.TemporaryDirectory(prefix='fr3_dual_arm_real_')
    run = Path(temp.name)
    root = build_model(description_share, scene, arms, mode='real', hardware=hardware)
    robot_xml = ET.tostring(root, encoding='unicode')

    description = {
        'robot_description': ParameterValue(robot_xml, value_type=str),
        'use_sim_time': False,
    }
    moveit_params = {
        **description,
        **moveit_config(root, arms, mode='real'),
        'allow_trajectory_execution': enable_execution,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # SDK gripper speed is configured separately from MoveIt trajectory timing.
        'trajectory_execution.allowed_goal_duration_margin': 32.0,
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
        side_file.write_text(yaml.safe_dump(controllers('real', side)), encoding='utf-8')
        manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            arguments=['--ros-args', '-r', f'controller_manager:__node:={manager_name}'],
            parameters=[{
                'robot_description': ParameterValue(manager_model(root, side), value_type=str),
                'use_sim_time': False,
            }, str(side_file)],
            remappings=[('/gripper_registers', f'/{side}/gripper_registers')],
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
    checker = Path(get_package_share_directory('fr3_dual_arm_bringup')) / 'launch/check_real_ready.py'
    ready = ExecuteProcess(cmd=['/usr/bin/python3', str(checker)] +
                           (['--require-actions'] if enable_execution else []), output='screen')
    for current, following in zip(spawners, spawners[1:]):
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=current,
            on_exit=success([following], 'controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=spawners[-1],
        on_exit=success([ready], 'last controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=ready, on_exit=success([move_group, rviz], 'real feedback/action check'))))
    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=move_group,
        on_exit=[EmitEvent(event=Shutdown(reason='move_group exited'))])))
    for manager in managers:
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='required manager exited'))])))

    messages = [LogInfo(msg='FR3 SDK gripper integration v2: expecting 14 measured joints; '
                            'gripper actions end in /gripper_cmd. Model: ' + str(description_share))]
    if hardware['gripper']['block'] == 0:
        messages.append(LogInfo(msg='Legacy gripper.block=0 overridden to SDK non-blocking block=1.'))
    return handlers + messages + [rsp] + managers + [spawners[0]]


def generate_launch_description():
    description_share = Path(get_package_share_directory('fr3_dual_arm_description'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value=''),
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
