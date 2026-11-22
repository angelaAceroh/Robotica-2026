"""El SCARA real: los seis nodos y la ventana del brazo.

    ros2 launch scara_kinematics brazo.launch.py
    ros2 launch scara_kinematics brazo.launch.py port:=/dev/ttyUSB0
    ros2 launch scara_kinematics brazo.launch.py ventana:=false
    ros2 launch scara_kinematics brazo.launch.py registro:=~/scara.csv

La cadena entera, de la placa a la punta:

    ESP32 --USB--> [esp] la aduana --+--> scara/alvin/aduana   --> [alvin]
                                     +--> scara/simon/aduana   --> [simon]
                                     +--> scara/teodoro/aduana --> [teodoro]
                                                                      |
                                                    scara/<motor>/junta
                                                                      v
                                                                    [cc]
                                                     joint_states ----+----> robot_state_publisher
                                                     scara/ee_pose ---+----> [esp_rec]

La aduana es la unica que abre /dev/ttyUSB0. La ventana brazo_hw no calcula nada:
mira los topics y pide cero y potencia por servicio.

RViz se anade con rviz.launch.py, y Gazebo con gazebo.launch.py; los dos incluyen
este mismo fichero, asi que la cadena es siempre esta.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Techo de velocidad del brazo real: el husillo no pasa de 8 mm/s. El max_vel del
# YAML es el de la simulacion y ahi el husillo vuela; si se le deja ese, la
# consigna se adelanta al brazo.
MAX_VEL_HW = [1.5, 1.5, 0.008]


def cadena(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    params = os.path.join(share, 'config', 'scara_urdf3.yaml')
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    comunes = [params, {'use_sim_time': sim}]

    nodos = [
        # 5. La aduana: el unico nodo con el puerto serie abierto.
        Node(package='scara_kinematics', executable='esp', name='esp', output='screen',
             parameters=comunes + [{'port': LaunchConfiguration('port'),
                                    'max_vel_hw': MAX_VEL_HW}],
             condition=IfCondition(LaunchConfiguration('hw'))),
    ]
    # 2, 3 y 4. Un nodo por motor, cada uno vigilando su junta.
    nodos += [Node(package='scara_kinematics', executable=m, name=m, output='screen',
                   parameters=comunes)
              for m in ('alvin', 'simon', 'teodoro')]
    nodos += [
        # 1. Cinematica directa: junta las tres y resuelve la punta.
        Node(package='scara_kinematics', executable='cc', name='cc', output='screen',
             parameters=comunes + [{'publicar_gz': LaunchConfiguration('gz')}]),
        # 6. El receptor del final de la cadena.
        Node(package='scara_kinematics', executable='esp_rec', name='esp_rec',
             output='screen',
             parameters=comunes + [{'registro': LaunchConfiguration('registro'),
                                    'reenviar': LaunchConfiguration('reenviar')}]),
        # La interfaz del brazo.
        Node(package='scara_kinematics', executable='brazo_hw', name='brazo_hw',
             output='screen', parameters=comunes,
             condition=IfCondition(LaunchConfiguration('ventana'))),
    ]

    if LaunchConfiguration('rsp').perform(context).lower() in ('true', '1', 'yes'):
        urdf = os.path.join(get_package_share_directory('scara_urdf'), 'urdf',
                            'scara_urdf.urdf')
        with open(urdf) as f:
            descripcion = f.read()
        nodos.append(
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='screen',
                 parameters=[{'robot_description': descripcion, 'use_sim_time': sim}]))
    return nodos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='puerto de la ESP32; vacio = buscarlo solo'),
        DeclareLaunchArgument('hw', default_value='true',
                              description='false = sin aduana (sin placa)'),
        DeclareLaunchArgument('ventana', default_value='true',
                              description='la interfaz del brazo (brazo_hw)'),
        DeclareLaunchArgument('rsp', default_value='true',
                              description='robot_state_publisher, para TF y RViz'),
        DeclareLaunchArgument('gz', default_value='false',
                              description='true = cc refleja las juntas en Gazebo'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='use_sim_time; lo pone gazebo.launch.py'),
        DeclareLaunchArgument('registro', default_value='',
                              description='CSV donde esp_rec anota la punta'),
        DeclareLaunchArgument('reenviar', default_value='false',
                              description='esp_rec devuelve la T a la placa'),
        OpaqueFunction(function=cadena),
    ])
