"""Launch file — starts all WallBot Painter nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'enable_mqtt',
            default_value='false',
            description='Start the MQTT dashboard gateway'
        ),
        DeclareLaunchArgument('mqtt_host', default_value='localhost'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('robot_id', default_value='wallbot'),
        DeclareLaunchArgument(
            'enable_uart', default_value='false',
            description='Start the ESP32 UART bridge for physical hardware'
        ),
        DeclareLaunchArgument('uart_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('uart_baudrate', default_value='921600'),

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

        Node(
            package='wallbot_painter',
            executable='uart_bridge',
            name='uart_bridge',
            parameters=[{
                'port': LaunchConfiguration('uart_port'),
                'baudrate': LaunchConfiguration('uart_baudrate'),
            }],
            condition=IfCondition(LaunchConfiguration('enable_uart')),
            output='screen',
        ),

        Node(
            package='wallbot_painter',
            executable='mqtt_dashboard',
            name='mqtt_dashboard',
            parameters=[config, {
                'mqtt_host': LaunchConfiguration('mqtt_host'),
                'mqtt_port': LaunchConfiguration('mqtt_port'),
                'robot_id': LaunchConfiguration('robot_id'),
            }],
            condition=IfCondition(LaunchConfiguration('enable_mqtt')),
            output='screen',
        ),
    ])
