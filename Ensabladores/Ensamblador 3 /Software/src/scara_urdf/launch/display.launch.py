"""Equivalente en ROS 2 de display.launch (ROS 1).

    ros2 launch scara_urdf display.launch.py
    ros2 launch scara_urdf display.launch.py gui:=false
    ros2 launch scara_urdf display.launch.py model:=/ruta/a/otro.urdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'scara_urdf'


def _crear_nodos(context, *args, **kwargs):
    modelo = LaunchConfiguration('model').perform(context)
    rviz_cfg = LaunchConfiguration('rvizconfig').perform(context)
    con_gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')

    # En ROS 2 no existe el parametro global /robot_description: hay que
    # pasarle el XML completo al robot_state_publisher.
    if modelo.endswith(('.xacro', '.xml')):
        import xacro
        robot_description = xacro.process_file(modelo).toxml()
    else:
        with open(modelo, 'r') as f:
            robot_description = f.read()

    jsp = 'joint_state_publisher_gui' if con_gui else 'joint_state_publisher'

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package=jsp,
            executable=jsp,
            name=jsp,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory(PKG)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(share, 'urdf', 'scara_urdf.urdf'),
            description='Ruta absoluta al archivo URDF/xacro del robot.'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=os.path.join(share, 'rviz', 'urdf.rviz'),
            description='Ruta absoluta a la configuracion de RViz.'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='true = joint_state_publisher_gui (sliders), false = sin GUI.'),
        OpaqueFunction(function=_crear_nodos),
    ])
