from setuptools import setup

package_name = 'quad_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrés Ruano',
    maintainer_email='andres34f@gmail.com',
    description='Paquete de arranque (bringup) para el robot cuadrúpedo. Inicializa teleoperación, gait, IK y micro-ROS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Por ahora dejamos vacío, los nodos se lanzan desde launch
            # 'nombre_nodo = quad_bringup.nombre_archivo:main',
        ],
    },
)
