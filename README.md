# QUANTUM-SQUAD-WALLBOTPAINTER

## Description

Ce projet consiste en la conception d'un **robot grimpeur autonome de 5 kg** dédié à la **peinture de structures métalliques verticales**.

## Architecture

### Adhérence — Roues magnétiques passives
L'adhérence est assurée par des roues magnétiques passives garantissant un maintien sécurisé sur surfaces métalliques sans consommation d'énergie.

### Propulsion — 4 moteurs brushless FOC
La propulsion repose sur quatre moteurs brushless pilotés par des contrôleurs ESC FOC (Field-Oriented Control), permettant un mouvement fluide et précis.

### Contrôle en boucle fermée
Un système de contrôle en boucle fermée intègre :
- Des **encodeurs haute résolution** pour mesurer la vitesse et la position réelles
- Une **centrale inertielle (IMU)** pour détecter et compenser les glissements en temps réel

### Intelligence distribuée sous ROS 2
- **Raspberry Pi 5** : navigation, planification de trajectoire, contrôle de haut niveau
- **ESP32** : tâches temps réel (lecture encodeurs, commandes moteurs, lecture IMU)

### Système de peinture
Un diffuseur automatique dont le **débit est asservi à la vitesse réelle** du robot garantit une application homogène de la peinture.

### Sécurité
- Retour automatique à la base (**RTH**) en cas de batterie faible
- Arrêt d'urgence sur défaut de glissement détecté
- Surveillance continue de l'état du robot

## Structure du projet

```
QUANTUM-SQUAD-WALLBOTPAINTER/
├── ros2_ws/                        # Espace de travail ROS 2
│   └── src/
│       └── wallbot_painter/        # Package ROS 2 principal
│           ├── wallbot_painter/    # Nœuds Python
│           │   ├── navigation_node.py       # Navigation (Raspberry Pi 5)
│           │   ├── motor_controller.py      # Contrôle moteurs brushless FOC
│           │   ├── imu_slip_controller.py   # Compensation glissement IMU/encodeurs
│           │   ├── painting_controller.py   # Asservissement débit peinture
│           │   └── safety_monitor.py        # Sécurité & RTH batterie faible
│           ├── config/
│           │   └── robot_params.yaml        # Paramètres du robot
│           ├── launch/
│           │   └── wallbot.launch.py        # Fichier de lancement ROS 2
│           └── test/
│               └── test_wallbot.py          # Tests unitaires
└── esp32_firmware/                 # Firmware ESP32 (tâches temps réel)
    └── main/
        ├── main.c                  # Point d'entrée firmware
        ├── motor_driver.c/.h       # Pilote moteurs ESC FOC
        └── encoder.c/.h            # Lecture encodeurs haute résolution
```

## Installation

### Prérequis
- ROS 2 Humble (ou supérieur)
- Python 3.10+
- ESP-IDF v5.x (pour le firmware ESP32)

### ROS 2

```bash
cd ros2_ws
colcon build --packages-select wallbot_painter
source install/setup.bash
```

### Lancement

```bash
ros2 launch wallbot_painter wallbot.launch.py
```

### Firmware ESP32

```bash
cd esp32_firmware
idf.py build flash monitor
```

## Tests

```bash
cd ros2_ws
python -m pytest src/wallbot_painter/test/ -v
```
