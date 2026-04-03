"""Launch file for motor control stack with UART bridge to ESP32."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for motor control."""
    
    # Find the package share directory
    pkg_share = FindPackageShare('wallbot_painter').find('wallbot_painter')
    
    # Path to YAML config file
    config_file = PathJoinSubstitution(
        [pkg_share, 'config', 'uart_params.yaml']
    )

    return LaunchDescription([
        # UART Bridge node (ESP32 ↔ ROS2 communication)
        Node(
            package='wallbot_painter',
            executable='uart_bridge',
            name='uart_bridge',
            parameters=[config_file],
            output='screen',
        ),

        # Motor Controller node (velocity → RPM commands, odometry)
        Node(
            package='wallbot_painter',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config_file],
            output='screen',
            remappings=[
                # Subscribe to velocity commands
                ('cmd_vel', '/cmd_vel'),
                # Receive encoder ticks from uart_bridge
                ('encoder_ticks', '/encoder_ticks'),
                # Publish motor RPM commands (received by uart_bridge)
                ('motor_rpm_cmd', '/motor_rpm_cmd'),
                # Publish odometry
                ('odom', '/odom'),
            ],
        ),
    ])
