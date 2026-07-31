"""Session 1 launch file — Gazebo physics sanity check.

Starts Gazebo Harmonic with a flat floor + a static vertical wall
(for future reference), publishes the robot_description from the
Gazebo-ready URDF, and spawns the robot ~30 cm above the floor.

No ROS2 <-> Gazebo bridge yet (that's Session 2) and no adhesion plugin
yet (Session 3) — this session only validates that the robot's mass,
inertia and collisions behave sensibly under normal gravity.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('wallbot_painter')
    world_path = os.path.join(pkg_share, 'worlds', 'wallbot_world.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'spider_v2_gazebo.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_wallbot',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wallbot',
            '-x', '0', '-y', '0', '-z', '0.3',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
    ])
