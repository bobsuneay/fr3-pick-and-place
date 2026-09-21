"""One entry point for measured-feedback teaching in mock or real mode."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('mock', 'gazebo', 'real'):
        raise ValueError('mode must be mock, gazebo or real')
    bringup = Path(get_package_share_directory('fr3_dual_arm_bringup'))
    camera_topic = arg('camera_topic') or (
        '/head_camera/image_raw' if mode in ('mock', 'gazebo') else '/camera/color/image_raw')
    arguments = {key: arg(key) for key in ('scene', 'arms', 'enable_execution', 'rviz')}
    if mode == 'gazebo':
        arguments['camera_topic'] = camera_topic
    if mode == 'real':
        if not arg('hardware'):
            raise ValueError('hardware:=/path/to/fr3_dual_arm.hardware.yaml required for real mode')
        arguments['hardware'] = arg('hardware')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            bringup / 'launch' / ('sim.launch.py' if mode == 'gazebo' else f'{mode}.launch.py'))),
                                 launch_arguments=arguments.items()),
        Node(package='fr3_dual_arm_grasp', executable='teach_ui' if arg('gui') == 'true' else 'grasp_node',
             output='screen', parameters=[{
                 'mode': mode, 'hardware': arg('hardware'),
                 'arms_file': arg('arms'), 'scene_file': arg('scene'),
                 'demo_config': arg('demo_config'), 'points_file': arg('points'),
                 'enable_execution': ParameterValue(LaunchConfiguration('enable_execution'), value_type=bool),
                 'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
             }]),
    ]


def generate_launch_description():
    description = Path(get_package_share_directory('fr3_dual_arm_description'))
    grasp = Path(get_package_share_directory('fr3_dual_arm_grasp'))
    defaults = {
        'mode': 'mock', 'enable_execution': 'false', 'hardware': '', 'rviz': 'true',
        'camera_topic': '',
        'gui': 'true', 'speed': '0.1', 'points': '~/.ros/fr3_demo/teach_points.json',
        'scene': str(description / 'config/scene.yaml'),
        'arms': str(description / 'config/arms.yaml'),
        'demo_config': str(grasp / 'config/demo.yaml'),
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=start)])
