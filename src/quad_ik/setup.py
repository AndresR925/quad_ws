from setuptools import setup

package_name = 'quad_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=['quad_ik'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrés Ruano',
    maintainer_email='andres34f@gmail.com',
    description='Nodo de cinemática inversa para las patas del cuadrúpedo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leg_ik_node = quad_ik.leg_ik_node:main'
        ],
    },
)