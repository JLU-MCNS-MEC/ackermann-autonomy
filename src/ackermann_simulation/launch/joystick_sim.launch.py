"""Drive the Gazebo Ackermann vehicle with a safety-gated joystick."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo, Xbox input, the manual adapter and optional RViz."""
    simulation_share = get_package_share_directory('ackermann_simulation')
    default_world = os.path.join(
        simulation_share, 'worlds', 'sensor_lab.sdf'
    )
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            # sensor_lab.sdf intentionally retains the historical line_track
            # world name used by its bridge and validation tooling.
            'world_name': 'line_track',
            'gz_args': LaunchConfiguration('gz_args'),
            'start_x': '0.0',
            'start_y': '0.0',
            'enable_line_follower': 'false',
            'enable_waypoint_nav': 'false',
            'enable_ackermann_adapter': 'true',
            'enable_rgbd': LaunchConfiguration('enable_rgbd'),
            'enable_render_sensors': LaunchConfiguration(
                'enable_render_sensors'
            ),
            'max_speed': LaunchConfiguration('max_forward_speed'),
            'max_steering': LaunchConfiguration('max_steering'),
            'command_timeout': '0.25',
        }.items(),
    )
    joystick = Node(
        package='joy',
        executable='joy_node',
        name='xbox_joy',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )
    manual_control = Node(
        package='ackermann_autonomy',
        executable='joy_to_ackermann',
        name='joy_to_ackermann',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'speed_axis': LaunchConfiguration('speed_axis'),
            'steering_axis': LaunchConfiguration('steering_axis'),
            'deadman_button': LaunchConfiguration('deadman_button'),
            'speed_sign': LaunchConfiguration('speed_sign'),
            'steering_sign': LaunchConfiguration('steering_sign'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed'),
            'max_reverse_speed': LaunchConfiguration('max_reverse_speed'),
            'max_steering': LaunchConfiguration('max_steering'),
        }],
    )
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_joystick',
        output='screen',
        arguments=[
            '-d',
            os.path.join(simulation_share, 'rviz', 'dynamics.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('speed_axis', default_value='1'),
        DeclareLaunchArgument('steering_axis', default_value='0'),
        DeclareLaunchArgument('deadman_button', default_value='5'),
        DeclareLaunchArgument('speed_sign', default_value='1.0'),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.6'),
        DeclareLaunchArgument('max_reverse_speed', default_value='0.3'),
        DeclareLaunchArgument('max_steering', default_value='0.55'),
        DeclareLaunchArgument('enable_rgbd', default_value='false'),
        DeclareLaunchArgument('enable_render_sensors', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=f'-r {default_world}',
            description='Gazebo arguments; add -s --headless-rendering for server-only mode.',
        ),
        simulator,
        joystick,
        manual_control,
        rviz,
    ])
