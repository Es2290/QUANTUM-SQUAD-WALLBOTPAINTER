#!/bin/bash
# launch_wallbot.sh - Démarrage rapide du WallBot Painter

set -e

echo "🚀 WallBot Painter - Motor Control Stack"
echo "=========================================="
echo ""

# Vérifier que l'environnement existe
if [ ! -d "$HOME/ros2_env" ]; then
    echo "❌ Environnement ROS2 non trouvé: $HOME/ros2_env"
    echo "   Crée-le avec: python3 -m venv ~/ros2_env"
    exit 1
fi

# Activer l'environnement
echo "✅ Activation de l'environnement ROS2..."
source $HOME/ros2_env/bin/activate

# Vérifier pyserial
echo "✅ Vérification de pyserial..."
python3 -c "import serial; print('   Version:', serial.__version__)" || {
    echo "❌ pyserial non disponible"
    exit 1
}

# Vérifier ROS2 (sur le système hôte)
echo "✅ Vérification de ROS2..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 Humble not found at /opt/ros/humble"
    echo "   Install it or source from your host Ubuntu terminal"
    exit 1
fi

# Se placer dans le workspace
cd "$(dirname "$0")/ros2_ws" || exit 1

# Afficher l'état
echo ""
echo "📦 Workspace: $(pwd)"
echo "🐍 Python: $(python3 --version)"
echo "📡 UART Bridge: uart_bridge"
echo "🎮 Motor Controller: motor_controller"
echo ""
echo "Lancement du stack motor_control..."
echo ""

# Lancer le stack
ros2 launch wallbot_painter motor_control.launch.py
