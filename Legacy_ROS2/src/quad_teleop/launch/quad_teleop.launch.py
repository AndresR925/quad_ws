import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    node_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'], #Lanza nodo Joy incluido en ROS2
        output='screen'
    )

    node_quad_teleop_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'quad_teleop', 'quad_teleop_joy_node'], #Lanza el nodo de teleoperación
        output='screen'
    )

    return LaunchDescription([
        node_joy,
        node_quad_teleop_joy,
    ])