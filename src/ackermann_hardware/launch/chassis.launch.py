"""Start the Jetson SocketCAN and PWM chassis driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create a safely-disabled real chassis driver."""
    share = get_package_share_directory('ackermann_hardware')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(share, 'config', 'chassis.yaml'),
            ),
            DeclareLaunchArgument('can_interface', default_value='can0'),
            DeclareLaunchArgument(
                'enable_on_start',
                default_value='false',
                choices=['true', 'false'],
                description='Keep false until wheels are lifted and steering is unloaded.',
            ),
            Node(
                package='ackermann_hardware',
                executable='chassis_node',
                name='chassis_driver',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'can_interface': LaunchConfiguration('can_interface'),
                        'enable_on_start': LaunchConfiguration('enable_on_start'),
                    },
                ],
            ),
        ]
    )
