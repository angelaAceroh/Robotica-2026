"""
Trayectoria RECTILINEA entre dos puntos, resuelta por CINEMATICA INVERSA,
sobre la misma simulacion dinamica de Gazebo (gz sim Harmonic).

Es una copia de gazebo.launch.py: monta exactamente el mismo mundo, el mismo
robot y el mismo puente, pero en vez del panel de deslizadores arranca
trayectoria_lineal.py, que subdivide el segmento y manda las consignas que
calcula la CI.  gazebo.launch.py queda intacto.

    ros2 launch planta_brazo trayectoria_lineal.launch.py
    ros2 launch planta_brazo trayectoria_lineal.launch.py rviz:=true
    ros2 launch planta_brazo trayectoria_lineal.launch.py n_puntos:=15
    ros2 launch planta_brazo trayectoria_lineal.launch.py p_ini:="300,-150,250" p_fin:="300,150,250"
    ros2 launch planta_brazo trayectoria_lineal.launch.py interp:=articular
    ros2 launch planta_brazo trayectoria_lineal.launch.py bucle:=true
    ros2 launch planta_brazo trayectoria_lineal.launch.py gui:=false

Los puntos van en MILIMETROS y en el marco 'world' (el suelo, bajo el eje de
la base).  Al terminar deja la tabla en consola y en ~/trayectoria_lineal.md
y ~/trayectoria_lineal.csv.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MUNDO = 'lab_brazo'          # debe coincidir con <world name=...> de lab_brazo.sdf
MODELO = 'planta_brazo'


def generate_launch_description():
    pkg = 'planta_brazo'
    share = get_package_share_directory(pkg)
    parametros = os.path.join(share, 'config', 'parametros.yaml')
    mundo_sdf = os.path.join(share, 'worlds', f'{MUNDO}.sdf')

    usar_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    ruta_recursos = os.path.dirname(share)

    xacro_file = os.path.join(share, 'urdf', 'planta_brazo.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]),
                                            value_type=str),
        'use_sim_time': True,
    }

    gz_args = PythonExpression([
        "'", mundo_sdf, " -r -v 2' if '", gui, "'.lower() in ('true','1') "
        "else '", mundo_sdf, " -r -s -v 2'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    puente = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='puente_gz',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{MUNDO}/model/{MODELO}/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{MODELO}/J1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{MODELO}/J2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{MODELO}/J3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            (f'/world/{MUNDO}/model/{MODELO}/joint_state', '/joint_states_gz'),
        ])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-topic', 'robot_description',
                   '-name', MODELO,
                   '-world', MUNDO,
                   '-z', '0.0',
                   '-allow_renaming', 'true'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='abrir tambien RViz2 con la traza'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo solo servidor'),
        DeclareLaunchArgument('p_ini', default_value='300,-150,250',
                              description='punto A del segmento, en mm "x,y,z"'),
        DeclareLaunchArgument('p_fin', default_value='300,150,250',
                              description='punto B del segmento, en mm "x,y,z"'),
        DeclareLaunchArgument('n_puntos', default_value='12',
                              description='subdivisiones del segmento (min. 10)'),
        DeclareLaunchArgument('interp', default_value='cartesiano',
                              description='cartesiano (recta real) | articular'),
        DeclareLaunchArgument('preferencia', default_value='codo_arriba',
                              description='codo_arriba | codo_abajo | cercana'),
        DeclareLaunchArgument('t_tramo', default_value='1.6',
                              description='segundos de cada tramo'),
        DeclareLaunchArgument('salida', default_value='~/trayectoria_lineal',
                              description='ruta base de los resultados'),
        DeclareLaunchArgument('bucle', default_value='false',
                              description='true: tras la tabla, barre A<->B sin '
                                          'parar (para grabar)'),
        DeclareLaunchArgument('t_barrido', default_value='4.0',
                              description='segundos de cada barrido en bucle'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ruta_recursos),

        gazebo,
        puente,

        Node(package=pkg, executable='remuestreo_juntas.py', output='screen',
             parameters=[{'frecuencia': 60.0, 'use_sim_time': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        spawn,

        # el que hace el trabajo: subdivide la recta y resuelve la CI
        Node(package=pkg, executable='trayectoria_lineal.py', output='screen',
             parameters=[parametros, {
                 'modo': 'gazebo',
                 'use_sim_time': True,
                 'p_ini': LaunchConfiguration('p_ini'),
                 'p_fin': LaunchConfiguration('p_fin'),
                 'n_puntos': ParameterValue(LaunchConfiguration('n_puntos'),
                                            value_type=int),
                 'interp': LaunchConfiguration('interp'),
                 'preferencia': LaunchConfiguration('preferencia'),
                 't_tramo': ParameterValue(LaunchConfiguration('t_tramo'),
                                           value_type=float),
                 'salida': LaunchConfiguration('salida'),
                 'bucle': ParameterValue(LaunchConfiguration('bucle'),
                                         value_type=bool),
                 't_barrido': ParameterValue(LaunchConfiguration('t_barrido'),
                                             value_type=float),
             }]),

        # cinematica directa + traza del TCP (usa el estado REAL de Gazebo)
        Node(package=pkg, executable='cinematica_directa.py', output='screen',
             parameters=[parametros, {'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(usar_rviz),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'rviz', 'planta_brazo.rviz')]),
    ])
