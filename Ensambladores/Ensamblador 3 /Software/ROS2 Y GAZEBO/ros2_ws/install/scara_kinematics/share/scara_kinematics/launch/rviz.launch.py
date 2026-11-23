"""El SCARA real dibujado en RViz2.

    ros2 launch scara_kinematics rviz.launch.py
    ros2 launch scara_kinematics rviz.launch.py ventana:=false
    ros2 launch scara_kinematics rviz.launch.py port:=/dev/ttyUSB0

Es brazo.launch.py -los seis nodos- mas RViz. Lo que se ve es el brazo de verdad:
cc arma /joint_states con lo que miden los encoders, robot_state_publisher lo
convierte en TF y RViz lo dibuja. Sin fisica y sin simulacion, asi que si el
dibujo no cuadra con el brazo, el problema esta en los encoders o en el cero, no
en un PID.

Ademas de las mallas, en RViz salen scara/ee_pose (la punta), scara/ee_path (su
rastro) y scara/ee_marker, los tres publicados por cc.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('scara_kinematics')
    brazo = os.path.join(share, 'launch', 'brazo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('hw', default_value='true'),
        DeclareLaunchArgument('ventana', default_value='true'),
        DeclareLaunchArgument('registro', default_value=''),
        DeclareLaunchArgument(
            'rvizconfig', default_value=os.path.join(share, 'rviz', 'scara.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(brazo),
            launch_arguments={'port': LaunchConfiguration('port'),
                              'hw': LaunchConfiguration('hw'),
                              'ventana': LaunchConfiguration('ventana'),
                              'registro': LaunchConfiguration('registro'),
                              'rsp': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')]),
    ])
