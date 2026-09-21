from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value=''),
        DeclareLaunchArgument('left_manager', default_value='left_controller_manager'),
        DeclareLaunchArgument('right_manager', default_value='right_controller_manager'),
        # TODO: Launch one ros2_control_node per arm after vendor SDK/driver
        # packages are installed and the hardware interface is implemented.
    ])
