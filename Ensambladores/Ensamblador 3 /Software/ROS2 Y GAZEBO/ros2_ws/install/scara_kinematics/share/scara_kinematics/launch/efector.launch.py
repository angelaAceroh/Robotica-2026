"""La ventana `efector` y el nodo `vector`, juntos: escribes (x, y, z) y el brazo va.

    ros2 launch scara_kinematics efector.launch.py                         # con la placa (main.py)
    ros2 launch scara_kinematics efector.launch.py simular:=true           # sin placa
    ros2 launch scara_kinematics efector.launch.py simular:=true gz:=true  # sin placa, en Gazebo
    ros2 launch scara_kinematics efector.launch.py gz:=true                # placa, gz de espejo

Los dos leen config/scara_urdf3.yaml (el robot) y config/micropython.yaml (los
limites de los motores y como se pasa de junta a motor). Cerrar la ventana
cierra todo.

Con gz:=true se abre Gazebo (sin la cadena de los seis nodos: brazo:=false) y
`vector` le manda a sus mandos la posicion de los motores pasada a junta. Sin el
paso del husillo, la altura que ensena gz es la que pide la trayectoria.

No se lanza con brazo.launch.py: `esp` y `vector` querrian el mismo puerto.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('scara_kinematics')
    robot = os.path.join(share, 'config', 'scara_urdf3.yaml')
    motores = os.path.join(share, 'config', 'micropython.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(LaunchConfiguration('gz')),
        launch_arguments={'brazo': 'false', 'rviz': 'false',
                          'gui': LaunchConfiguration('gui')}.items())
    vector = Node(
        package='scara_kinematics', executable='vector', name='vector', output='screen',
        parameters=[robot, motores, {
            'simular': ParameterValue(LaunchConfiguration('simular'), value_type=bool),
            'gz': ParameterValue(LaunchConfiguration('gz'), value_type=bool),
            'puerto': ParameterValue(LaunchConfiguration('puerto'), value_type=str)}])
    ventana = Node(
        package='scara_kinematics', executable='efector', name='efector', output='screen',
        parameters=[robot, motores])

    return LaunchDescription([
        DeclareLaunchArgument('simular', default_value='false',
                              description='true = sin placa: vector hace como que mueve'),
        DeclareLaunchArgument('gz', default_value='false',
                              description='true = abre Gazebo y le copia los motores'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='con gz:=true, false = Gazebo sin ventana'),
        DeclareLaunchArgument('puerto', default_value='',
                              description='vacio = buscar la ESP32 sola'),
        gazebo,
        vector,
        ventana,
        RegisterEventHandler(OnProcessExit(target_action=ventana,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
