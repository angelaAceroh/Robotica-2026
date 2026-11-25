"""
ESPEJO: Gazebo repite lo que hace el ROBOT FISICO, en vivo.

Levanta la simulacion pero SIN nodo de trayectoria: el modelo se queda
quieto esperando consignas.  Quien se las manda es el programa del robot
fisico, que publica la posicion MEDIDA en los encoders:

    terminal 1:  ros2 launch planta_brazo espejo.launch.py
    terminal 2:  cd ~/robot_fisico && python3 mover_robot_vel.py --ros ...

El puerto serie solo lo puede abrir un programa a la vez, por eso el que
publica es el propio mover_robot_vel.py (clase EspejoROS) y no un nodo
aparte: reutiliza la telemetria que ya esta leyendo.

Desde las correcciones del 2026-10-23 el modelo de Gazebo tiene las MISMAS
medidas que el robot construido, asi que lo que veas en pantalla es del
mismo tamano que lo que tienes delante.
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

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ruta_recursos),

        gazebo,
        puente,

        Node(package=pkg, executable='remuestreo_juntas.py', output='screen',
             parameters=[{'frecuencia': 60.0, 'use_sim_time': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        spawn,

        # NO hay nodo de trayectoria: el modelo espera las consignas que le
        # manda mover_robot_vel.py --ros desde el robot fisico.

        # cinematica directa + traza del TCP (usa el estado REAL de Gazebo)
        Node(package=pkg, executable='cinematica_directa.py', output='screen',
             parameters=[parametros, {'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(usar_rviz),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'rviz', 'planta_brazo.rviz')]),
    ])
