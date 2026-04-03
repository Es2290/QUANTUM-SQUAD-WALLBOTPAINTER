from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('wallbot_painter')

    # Inclure le lancement du contrôle moteur (UART bridge + motor controller)
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'motor_control.launch.py'])
        )
    )

    # Node de contrôle peinture (pour répondre aux messages de teleop)
    painting_controller = Node(
        package='wallbot_painter',
        executable='painting_controller',
        name='painting_controller',
        output='screen'
    )

    # Node Teleop (Interface temps réel)
    # On utilise 'prefix' pour xterm ou gnome-terminal si on veut une fenêtre séparée, 
    # mais ici on va le laisser dans le terminal courant
    teleop_node = Node(
        package='wallbot_painter',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        emulate_tty=True  # Important pour la capture du clavier
    )

    return LaunchDescription([
        motor_control_launch,
        painting_controller,
        teleop_node
    ])
