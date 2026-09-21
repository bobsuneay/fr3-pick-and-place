from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('fr3_dual_arm_gazebo')) / 'launch/sim.launch.py')
            ),
            launch_arguments={
                'enable_execution': LaunchConfiguration('enable_execution'),
                'rviz': LaunchConfiguration('rviz'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
    ])
