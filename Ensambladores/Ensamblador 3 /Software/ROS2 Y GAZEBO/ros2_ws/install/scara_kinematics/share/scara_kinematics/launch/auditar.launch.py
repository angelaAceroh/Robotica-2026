"""La cadena de siempre, con el auditor mirando cada salto.

    ros2 launch scara_kinematics auditar.launch.py
    ros2 launch scara_kinematics auditar.launch.py hw:=false     # sin placa: para la sonda
    ros2 launch scara_kinematics auditar.launch.py informe:=~/auditoria.csv

Es `brazo.launch.py` tal cual -los seis nodos y la ventana- mas un septimo nodo
que no toca nada: solo escucha los cuatro topics por los que viajan los datos y
lleva la cuenta de lo que llega, lo que se pierde y lo que tarda.

    [esp] --aduana--> [alvin|simon|teodoro] --junta--> [cc] --joint_states/ee_pose-->
                                  |                      |
                                  +------ [auditor] -----+
                                     mira, no toca

Con `hw:=false` no hay placa y la cadena esta parada: entonces el servicio
`scara/auditor/comprobar` mete por ella una rampa conocida (la sonda) y
certifica que sale identica por el otro extremo. Con placa, ese mismo servicio
hace la prueba de ida y vuelta contra el firmware, que no mueve ningun motor.

    ros2 service call /scara/auditor/comprobar std_srvs/srv/Trigger
    ros2 topic echo /scara/auditor/informe
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# El mismo techo real que fija brazo.launch.py. El auditor lo necesita para dos
# cosas: saber cuanto es "un salto imposible" en la telemetria, y comparar el
# vmax que devuelve la placa en el volcado de "?" con el que se le mando.
MAX_VEL_HW = [1.5, 1.5, 0.008]


def cadena(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    params = os.path.join(share, 'config', 'scara_urdf3.yaml')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch',
                                                       'brazo.launch.py')),
            launch_arguments={'port': LaunchConfiguration('port'),
                              'hw': LaunchConfiguration('hw'),
                              'ventana': LaunchConfiguration('ventana'),
                              'rsp': LaunchConfiguration('rsp'),
                              'registro': LaunchConfiguration('registro')}.items()),
        Node(package='scara_kinematics', executable='auditor', name='auditor',
             output='screen',
             parameters=[params, {'max_vel_hw': MAX_VEL_HW,
                                  'registro': LaunchConfiguration('informe')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='puerto de la ESP32; vacio = buscarlo solo'),
        DeclareLaunchArgument('hw', default_value='true',
                              description='false = sin aduana; la sonda va asi'),
        DeclareLaunchArgument('ventana', default_value='true',
                              description='la interfaz del brazo (brazo_hw)'),
        DeclareLaunchArgument('rsp', default_value='true',
                              description='robot_state_publisher, para TF y RViz'),
        DeclareLaunchArgument('registro', default_value='',
                              description='CSV donde esp_rec anota la punta'),
        DeclareLaunchArgument('informe', default_value='',
                              description='CSV donde el auditor anota los saltos'),
        OpaqueFunction(function=cadena),
    ])
