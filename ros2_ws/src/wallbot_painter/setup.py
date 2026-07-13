from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wallbot_painter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Quantum Squad',
    maintainer_email='team@quantum-squad.local',
    description='Autonomous magnetic-wheel climbing robot for painting vertical metallic structures.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = wallbot_painter.navigation_node:main',
            'motor_controller = wallbot_painter.motor_controller:main',
            'uart_bridge = wallbot_painter.uart_bridge:main',
            'imu_slip_controller = wallbot_painter.imu_slip_controller:main',
            'painting_controller = wallbot_painter.painting_controller:main',
            'safety_monitor = wallbot_painter.safety_monitor:main',
            'teleop_node = wallbot_painter.teleop_node:main',
            'hardware_simulator = wallbot_painter.hardware_simulator:main',
        ],
    },
)
