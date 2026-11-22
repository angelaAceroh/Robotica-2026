"""Las rutas de ruta.py (perforado, contorno) en Gazebo, o en el brazo de la mesa.

    ros2 launch scara_kinematics ruta.launch.py                          # perforado
    ros2 launch scara_kinematics ruta.launch.py ruta:=contorno
    ros2 launch scara_kinematics ruta.launch.py perfil:=lineal           # la ley de la guia
    ros2 launch scara_kinematics ruta.launch.py registro:=~/ruta.csv     # lo medido, a CSV
    ros2 launch scara_kinematics ruta.launch.py destino:=placa           # el brazo real

Mismas dos topologias que trayectoria.launch.py, y por la misma razon: con
destino:=gz la cadena de los seis nodos va apagada (brazo:=false), porque `cc`
escribe en los mismos /scara_urdf/cmd/<junta> que este nodo. Con destino:=placa
se levanta la cadena entera y el nodo manda por scara/hw/cmd; ahi no arranca
solo: hay que habilitar, hacer el cero y mandar la ruta a mano

    ros2 topic pub --once /scara/ruta/orden std_msgs/String "data: 'perforado'"

No lanza `tray`: los dos a la vez se pelearian por los mismos mandos.
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
            'brazo': 'true' if al_brazo else 'false',
            'rviz': LaunchConfiguration('rviz'),
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'port': LaunchConfiguration('port'),
            'hw': LaunchConfiguration('hw'),
            'ventana': LaunchConfiguration('ventana'),
            'spawn_delay': LaunchConfiguration('spawn_delay')}.items())

    ruta = Node(
        package='scara_kinematics', executable='ruta_nodo', name='ruta', output='screen',
        parameters=[params, {
            'use_sim_time': not al_brazo,
            'destino': destino,
            'ruta': ParameterValue(LaunchConfiguration('ruta'), value_type=str),
            'perfil': ParameterValue(LaunchConfiguration('perfil'), value_type=str),
            'codo': ParameterValue(LaunchConfiguration('codo'), value_type=str),
            'factor_vel': ParameterValue(LaunchConfiguration('factor_vel'), value_type=float),
            'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
            'espera': ParameterValue(LaunchConfiguration('espera'), value_type=float),
            'lazo': ParameterValue(LaunchConfiguration('lazo'), value_type=bool),
            'registro': ParameterValue(LaunchConfiguration('registro'), value_type=str)}])

    return [gazebo, ruta]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('destino', default_value='gz',
                              description='gz = solo la simulacion | placa = el brazo real'),
        DeclareLaunchArgument('ruta', default_value='perforado',
                              description='perforado | contorno (con destino:=placa no '
                                          'arranca sola)'),
        DeclareLaunchArgument('perfil', default_value='trapecio',
                              description='lineal (la guia) | trapecio | cubico | quintico '
                                          '| septico | scurve'),
        DeclareLaunchArgument('codo', default_value='down', description='down | up'),
        DeclareLaunchArgument('factor_vel', default_value='0.35',
                              description='fraccion del max_vel del YAML que gz sigue'),
        DeclareLaunchArgument('dt', default_value='0.02',
                              description='periodo de la consigna (0.02 = 50 Hz)'),
        DeclareLaunchArgument('espera', default_value='6.0',
                              description='margen para que gz cargue el robot'),
        DeclareLaunchArgument('lazo', default_value='false',
                              description='true = repite la ruta sin parar'),
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
                              description='solo con destino:=placa: false = sin aduana'),
        DeclareLaunchArgument('ventana', default_value='true',
                              description='solo con destino:=placa: la interfaz brazo_hw'),
        OpaqueFunction(function=montar),
    ])
