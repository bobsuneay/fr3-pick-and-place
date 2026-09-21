from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='head_camera'),
        DeclareLaunchArgument('arm', default_value='right'),
        LogInfo(msg='TODO: integrate easy_handeye or vendor hand-eye calibration'),
    ])
