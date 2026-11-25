# Launch: Control cinematico por jacobiano inverso en Gazebo.
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

MUNDO = 'lab_brazo'
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
        DeclareLaunchArgument('figura', default_value='tarea',
                              description='tarea (puntos de la guia) | recta | circulo'),
        DeclareLaunchArgument('perfil', default_value='trapezoidal',
                              description='trapezoidal | cubico | quintico'),
        DeclareLaunchArgument('rampa', default_value='0.25',
                              description='fraccion de rampa del trapezoidal'),
        DeclareLaunchArgument('centro', default_value='300,0,250',
                              description='centro del circulo, en mm "x,y,z"'),
        DeclareLaunchArgument('radio', default_value='80.0',
                              description='radio del circulo, en mm'),
        DeclareLaunchArgument('duracion', default_value='4.0',
                              description='segundos de cada TRAMO'),
        DeclareLaunchArgument('frecuencia', default_value='100.0',
                              description='Hz del lazo (dt = 1/f)'),
        DeclareLaunchArgument('t_home', default_value='3.0',
                              description='segundos para ir al home al arrancar'),
        DeclareLaunchArgument('realimentacion', default_value='medida',
                              description='medida (lazo cerrado sobre Gazebo) | '
                                          'plan (lazo sobre el estado integrado)'),
        DeclareLaunchArgument('pasadas', default_value='1',
                              description='cuantas veces se recorre la figura'),
        DeclareLaunchArgument('ganancia', default_value='5.0',
                              description='K del lazo cartesiano [1/s]'),
        DeclareLaunchArgument('amortiguacion', default_value='0.0',
                              description='lambda de la pseudoinversa amortiguada'),
        DeclareLaunchArgument('graficas', default_value='true',
                              description='ventana de graficas en vivo'),
        DeclareLaunchArgument('guardar', default_value='false',
                              description='true: ademas escribe el CSV'),
        DeclareLaunchArgument('salida', default_value='~/control_cinematico',
                              description='ruta base del CSV si guardar:=true'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ruta_recursos),

        gazebo,
        puente,

        Node(package=pkg, executable='remuestreo_juntas.py', output='screen',
             parameters=[{'frecuencia': 60.0, 'use_sim_time': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        spawn,

        Node(package=pkg, executable='control_cinematico.py', output='screen',
             parameters=[parametros, {
                 'modo': 'gazebo',
                 'use_sim_time': True,
                 'figura': LaunchConfiguration('figura'),
                 'perfil': LaunchConfiguration('perfil'),
                 'rampa': ParameterValue(LaunchConfiguration('rampa'),
                                         value_type=float),
                 'p_ini': LaunchConfiguration('p_ini'),
                 'p_fin': LaunchConfiguration('p_fin'),
                 'centro': LaunchConfiguration('centro'),
                 'radio': ParameterValue(LaunchConfiguration('radio'),
                                         value_type=float),
                 'duracion': ParameterValue(LaunchConfiguration('duracion'),
                                            value_type=float),
                 'ganancia': ParameterValue(LaunchConfiguration('ganancia'),
                                            value_type=float),
                 'amortiguacion': ParameterValue(
                     LaunchConfiguration('amortiguacion'), value_type=float),
                 'pasadas': ParameterValue(LaunchConfiguration('pasadas'),
                                           value_type=int),
                 'frecuencia': ParameterValue(
                     LaunchConfiguration('frecuencia'), value_type=float),
                 't_home': ParameterValue(LaunchConfiguration('t_home'),
                                          value_type=float),
                 'realimentacion': LaunchConfiguration('realimentacion'),
                 'graficas': ParameterValue(LaunchConfiguration('graficas'),
                                            value_type=bool),
                 'guardar': ParameterValue(LaunchConfiguration('guardar'),
                                           value_type=bool),
                 'salida': LaunchConfiguration('salida'),
             }]),

        Node(package=pkg, executable='cinematica_directa.py', output='screen',
             parameters=[parametros, {'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(usar_rviz),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'rviz', 'planta_brazo.rviz')]),
    ])
