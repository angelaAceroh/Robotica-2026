"""Lanza el SCARA en Gazebo (gz sim).

Este paquete solo contiene la descripcion del robot; los nodos viven en
scara_kinematics, asi que aqui unicamente se reenvia.

    ros2 launch scara_urdf gazebo.launch.py
    ros2 launch scara_urdf gazebo.launch.py rviz:=true
    ros2 launch scara_urdf gazebo.launch.py brazo:=false   (simulacion sola)

Es lo mismo que "ros2 launch scara_kinematics gazebo.launch.py": se mantiene
porque es el nombre por el que se lanzaba desde ROS 1.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    interno = os.path.join(get_package_share_directory('scara_kinematics'),
                           'launch', 'gazebo.launch.py')
    reenviados = ('world', 'gui', 'rviz', 'brazo', 'port', 'hw', 'ventana')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('brazo', default_value='true'),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('hw', default_value='true'),
        DeclareLaunchArgument('ventana', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(interno),
            launch_arguments={a: LaunchConfiguration(a) for a in reenviados}.items()),
    ])
