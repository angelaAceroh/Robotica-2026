"""
Visualizacion del robot PLANTA BRAZO en RViz2.

    ros2 launch planta_brazo display.launch.py             # sliders
    ros2 launch planta_brazo display.launch.py demo:=true  # trayectoria automatica
    ros2 launch planta_brazo display.launch.py rviz:=false # sin RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'planta_brazo'
    share = get_package_share_directory(pkg)

    demo = LaunchConfiguration('demo')
    usar_rviz = LaunchConfiguration('rviz')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(pkg), 'urdf', 'planta_brazo.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': False,
    }
    parametros = os.path.join(share, 'config', 'parametros.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='false',
                              description='true: trayectoria automatica en vez de sliders'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='lanzar RViz2'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        Node(package=pkg, executable='gui_articulaciones.py', output='screen',
             parameters=[parametros], condition=UnlessCondition(demo)),

        Node(package=pkg, executable='trayectoria_demo.py', output='screen',
             condition=IfCondition(demo)),

        Node(package=pkg, executable='cinematica_directa.py', output='screen',
             parameters=[parametros]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(usar_rviz),
             arguments=['-d', os.path.join(share, 'rviz', 'planta_brazo.rviz')]),
    ])
