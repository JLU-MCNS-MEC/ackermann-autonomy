"""Start the real command boundary and Jetson chassis driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Connect the Nav2-safe Twist boundary to the real chassis."""
    bringup_share = get_package_share_directory('ackermann_bringup')
    hardware_share = get_package_share_directory('ackermann_hardware')
    control_boundary = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'control_boundary.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'input_topic': '/cmd_vel_safe',
            'output_topic': '/drive',
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_steering': LaunchConfiguration('max_steering'),
        }.items(),
    )
    chassis = Node(
        package='ackermann_hardware',
        executable='chassis_node',
        name='chassis_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'max_steering': LaunchConfiguration('max_steering'),
                'enable_on_start': LaunchConfiguration('enable_on_start'),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(hardware_share, 'config', 'chassis.yaml'),
            ),
            DeclareLaunchArgument('can_interface', default_value='can0'),
            DeclareLaunchArgument('wheelbase', default_value='0.56'),
            DeclareLaunchArgument('max_speed', default_value='0.30'),
            DeclareLaunchArgument('max_steering', default_value='0.55'),
            DeclareLaunchArgument(
                'enable_on_start', default_value='false', choices=['true', 'false']
            ),
            control_boundary,
            chassis,
        ]
    )
