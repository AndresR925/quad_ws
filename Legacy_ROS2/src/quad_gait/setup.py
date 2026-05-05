from setuptools import setup
import os
from glob import glob

package_name = 'quad_gait'

setup(
    name=package_name,
    version='0.0.1',
    packages=[  # módulos Python principales
        'body_motion_planner',
        'gait_generator',
        'nodes',
    ],
    data_files=[
        # Registro del paquete en ROS2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrés Ruano',
    maintainer_email='andres34f@gmail.com',
    description='Paquete para robot cuadrúpedo: gait planner y body motion planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Nodo principal del robot
            'gait_node = nodes.gait_node:main',
            # Script de prueba opcional
            'gait_main = nodes.main:main',
        ],
    },
)
