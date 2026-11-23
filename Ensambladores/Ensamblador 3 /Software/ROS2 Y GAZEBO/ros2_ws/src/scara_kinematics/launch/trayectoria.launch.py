"""Planificacion de trayectorias: en Gazebo, o en el brazo de la mesa.

    ros2 launch scara_kinematics trayectoria.launch.py demo:=true
    ros2 launch scara_kinematics trayectoria.launch.py demo:=true rviz:=true
    ros2 launch scara_kinematics trayectoria.launch.py perfil:=trapecio
    ros2 launch scara_kinematics trayectoria.launch.py registro:=~/tray.csv
    ros2 launch scara_kinematics trayectoria.launch.py destino:=placa   <-- el brazo real

El argumento `destino` elige las dos unicas topologias que no se pelean por un
topic. No es un gusto: `cc` con publicar_gz escribe en los mismos
/scara_urdf/cmd/<junta> que escribe `tray`, y dos nodos mandando sobre el mismo
controlador es una pelea que gana el ultimo que publique.

destino:=gz  (por defecto)  LA SIMULACION SOLA
    gazebo.launch.py con brazo:=false: ni cadena de seis nodos ni puerto serie.
    `tray` manda Float64 a los controladores de gz y lee el /joint_states que
    publica gz, asi que puede comparar lo que pidio con lo que la fisica hizo.

        [tray] --Float64--> /scara_urdf/cmd/<junta> --> [gz]
           ^                                              |
           +---------------- /joint_states <-- puente ----+

destino:=placa              EL BRAZO DE LA MESA, CON GAZEBO DE ESPEJO
    gazebo.launch.py con brazo:=true: la cadena entera de seis nodos. `tray` NO
    toca los mandos de gz; publica la consigna en scara/hw/cmd y la aduana la
    convierte ella sola en la linea 'T q1 q2 q3' del protocolo. El espejo en gz
    lo sigue haciendo `cc`, que es su trabajo desde siempre.

        [tray] --JointState--> scara/hw/cmd --> [esp] --USB--> ESP32
           ^                                                     |
           +-- /joint_states <-- [cc] <-- alvin/simon/teodoro <---+
                                   |
                                   +--Float64--> [gz]  (el espejo, de cc)

    Aqui `tray` planifica con `max_vel_hw` (husillo 0.008 m/s) en vez del
    max_vel del YAML (0.10 m/s, que es de la simulacion), y no manda nada hasta
    que scara/hw/ok sea true: placa viva, habilitada y con el cero hecho.

Ordenes, en los dos casos (la lista entera esta en tray.py):

    ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'demo'"
    ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'recta 0.20 0.20 0.11'"
    ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'circulo 0.30 0 0.08 0.15 codo=down'"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def montar(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    params = os.path.join(share, 'config', 'scara_urdf3.yaml')
    destino = LaunchConfiguration('destino').perform(context).lower()
    if destino not in ('gz', 'placa'):
        raise RuntimeError(f"destino tiene que ser 'gz' o 'placa', no {destino!r}")
    al_brazo = destino == 'placa'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            # Con destino=placa levanta la cadena de los seis nodos y el espejo
            # de cc; con destino=gz, la simulacion sola.
            'brazo': 'true' if al_brazo else 'false',
            'rviz': LaunchConfiguration('rviz'),
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'port': LaunchConfiguration('port'),
            'hw': LaunchConfiguration('hw'),
            'ventana': LaunchConfiguration('ventana'),
            'spawn_delay': LaunchConfiguration('spawn_delay')}.items())

    # El value_type no es adorno: un argumento de launch siempre llega como texto
    # y launch_ros le adivina el tipo. Con espera:=99 adivina int, el nodo lo
    # declaro double y el arranque se cae con InvalidParameterType. Diciendo el
    # tipo aqui, espera:=99 y espera:=99.0 valen igual.
    tray = Node(
        package='scara_kinematics', executable='tray', name='tray', output='screen',
        parameters=[params, {
            'use_sim_time': not al_brazo,
            'destino': destino,
            'demo': ParameterValue(LaunchConfiguration('demo'), value_type=bool),
            'lazo': ParameterValue(LaunchConfiguration('lazo'), value_type=bool),
            'perfil': ParameterValue(LaunchConfiguration('perfil'), value_type=str),
            'codo': ParameterValue(LaunchConfiguration('codo'), value_type=str),
            'duracion': ParameterValue(LaunchConfiguration('duracion'), value_type=float),
            'factor_vel': ParameterValue(LaunchConfiguration('factor_vel'), value_type=float),
            'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
            'espera': ParameterValue(LaunchConfiguration('espera'), value_type=float),
            'registro': ParameterValue(LaunchConfiguration('registro'), value_type=str)}])

    return [gazebo, tray]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('destino', default_value='gz',
                              description='gz = solo la simulacion | placa = el brazo real'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true = recorre solo los 25 metodos al arrancar'),
        DeclareLaunchArgument('lazo', default_value='false',
                              description='true = el demo se repite sin parar'),
        DeclareLaunchArgument('perfil', default_value='quintico',
                              description='cubico | quintico | septico | trapecio | scurve'),
        DeclareLaunchArgument('codo', default_value='auto',
                              description='auto | nearest | up | down'),
        DeclareLaunchArgument('duracion', default_value='0.0',
                              description='segundos por movimiento; 0 = lo calcula el max_vel'),
        DeclareLaunchArgument('factor_vel', default_value='0.35',
                              description='fraccion del max_vel del YAML que gz sigue de verdad'),
        DeclareLaunchArgument('dt', default_value='0.02',
                              description='periodo de la consigna (0.02 = 50 Hz)'),
        DeclareLaunchArgument('espera', default_value='6.0',
                              description='margen para que gz cargue el robot antes del demo'),
        DeclareLaunchArgument('registro', default_value='',
                              description='CSV con lo pedido y lo que se midio'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = gz sin ventana'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0'),
        DeclareLaunchArgument('port', default_value='',
                              description='solo con destino:=placa; vacio = buscarlo solo'),
        DeclareLaunchArgument('hw', default_value='true',
                              description='solo con destino:=placa: false = sin aduana, '
                                          'para ensayar la cadena sin abrir el puerto serie'),
        DeclareLaunchArgument('ventana', default_value='true',
                              description='solo con destino:=placa: la interfaz brazo_hw'),
        OpaqueFunction(function=montar),
    ])
