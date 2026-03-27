"""Launch file — starts all WallBot Painter nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare('wallbot_painter'), 'config', 'robot_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='wallbot_painter',
            executable='navigation_node',
            name='navigation_node',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        Node(
            package='wallbot_painter',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        Node(
            package='wallbot_painter',
            executable='imu_slip_controller',
            name='imu_slip_controller',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        Node(
            package='wallbot_painter',
            executable='painting_controller',
            name='painting_controller',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        Node(
            package='wallbot_painter',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
