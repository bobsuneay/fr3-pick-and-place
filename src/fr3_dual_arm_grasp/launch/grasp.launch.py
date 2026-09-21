"""Start the standalone teaching UI alongside an existing MoveIt bringup."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('fr3_dual_arm_description'))
    grasp = Path(get_package_share_directory('fr3_dual_arm_grasp'))
    defaults = {
        'arms_file': str(description / 'config/arms.yaml'),
        'scene_file': str(description / 'config/scene.yaml'),
        'demo_config': str(grasp / 'config/demo.yaml'),
        'points_file': '~/.ros/fr3_demo/teach_points.json',
        'mode': 'mock', 'hardware': '',
        'enable_execution': 'false', 'speed': '0.1',
    }
    parameters = {key: LaunchConfiguration(key) for key in defaults}
    parameters['enable_execution'] = ParameterValue(LaunchConfiguration('enable_execution'), value_type=bool)
    parameters['speed'] = ParameterValue(LaunchConfiguration('speed'), value_type=float)
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
        [Node(package='fr3_dual_arm_grasp', executable='teach_ui', parameters=[parameters], output='screen')])
