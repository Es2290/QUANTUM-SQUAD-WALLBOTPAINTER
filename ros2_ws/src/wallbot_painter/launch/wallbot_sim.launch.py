"""Launch file — full WallBot Painter stack + hardware simulator + RViz.

Use this instead of wallbot.launch.py when no physical Raspberry Pi / ESP32
/ motors are connected. It starts the same 5 software nodes, adds a
hardware_simulator node that stands in for the ESP32 (simulated encoder
ticks + IMU), a robot_state_publisher for the URDF, and RViz2 pre-configured
to show the robot moving through its painting trajectory.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('wallbot_painter')

    config = PathJoinSubstitution(
        [FindPackageShare('wallbot_painter'), 'config', 'robot_params.yaml']
    )
    rviz_config = os.path.join(pkg_share, 'config', 'wallbot.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'spider_v2.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Reuse the existing launch file for the 5 core software nodes
    core_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wallbot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'slip_factor',
            default_value='1.0',
            description='1.0 = no simulated wheel slip, <1.0 injects slip for testing'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to launch RViz2 automatically'
        ),

        core_nodes,

        Node(
            package='wallbot_painter',
            executable='hardware_simulator',
            name='hardware_simulator',
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slip_factor': LaunchConfiguration('slip_factor'),
            }],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
