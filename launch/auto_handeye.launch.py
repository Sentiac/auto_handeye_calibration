from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Absolute path to calibration YAML'),
        Node(
            package='auto_handeye_calibration',
            executable='auto_handeye_node',
            name='auto_handeye_calibration',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
