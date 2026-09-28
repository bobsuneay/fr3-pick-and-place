"""Start wrist D405 USB drivers; Gazebo sensors live in the robot model."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fr3_dual_arm_bringup.cameras import camera_remappings, selected_cameras


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode in ('mock', 'gazebo'):
        return [LogInfo(msg='Wrist cameras: Gazebo sensors are loaded by sim.launch.py; '
                            'mock mode provides geometry only. No USB drivers started.')]
    if mode != 'real':
        raise ValueError('mode must be mock, gazebo or real')
    with open(arg('camera_config'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    cameras = selected_cameras(config, {side: arg(side + '_serial') for side in ('left', 'right')})
    # Check model presence before starting drivers so every device has an attached TF root.
    with open(arg('scene'), encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)
    for name, _ in cameras:
        if name not in scene.get('cameras', {}):
            raise ValueError(f'{name} is enabled but missing from scene camera mounts')
    return [Node(package='realsense2_camera', executable='realsense2_camera_node',
                 name=name, namespace='/', output='screen', parameters=[params],
                 remappings=camera_remappings(name)) for name, params in cameras]


def generate_launch_description():
    bringup = Path(get_package_share_directory('fr3_dual_arm_bringup'))
    description = Path(get_package_share_directory('fr3_dual_arm_description'))
    defaults = {
        'mode': 'real', 'left_serial': '', 'right_serial': '',
        'camera_config': str(bringup / 'config/cameras.yaml'),
        'scene': str(description / 'config/scene.yaml'),
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=start)])
