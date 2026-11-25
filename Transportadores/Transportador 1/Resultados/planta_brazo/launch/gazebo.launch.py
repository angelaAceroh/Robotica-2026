# Launch: Gazebo + puente ROS + panel o demo + RViz.
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

    demo = LaunchConfiguration('demo')
    usar_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    panel = LaunchConfiguration('panel')
    comparar = LaunchConfiguration('comparar')

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
        DeclareLaunchArgument('demo', default_value='false',
                              description='true: recorre la trayectoria sin panel'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='abrir tambien RViz2 con la traza del TCP'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo solo servidor, sin ventana'),
        DeclareLaunchArgument('panel', default_value='true',
                              description='false: sin panel de deslizadores'),
        DeclareLaunchArgument('comparar', default_value='false',
                              description='true: recorre los puntos de prueba y '
                                          'saca la tabla directa vs inversa'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ruta_recursos),

        gazebo,
        puente,

        Node(package=pkg, executable='remuestreo_juntas.py', output='screen',
             parameters=[{'frecuencia': 60.0, 'use_sim_time': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        spawn,

        Node(package=pkg, executable='gui_articulaciones.py', output='screen',
             parameters=[parametros, {'modo': 'gazebo', 'use_sim_time': True}],
             condition=IfCondition(PythonExpression([
                 "'", panel, "'.lower() in ('true','1') and "
                 "'", demo, "'.lower() not in ('true','1') and "
                 "'", comparar, "'.lower() not in ('true','1')"]))),

        Node(package=pkg, executable='trayectoria_demo.py', output='screen',
             parameters=[{'modo': 'gazebo', 'periodo': 3.0,
                          'use_sim_time': True}],
             condition=IfCondition(demo)),

        Node(package=pkg, executable='comparacion_cinematicas.py',
             output='screen', condition=IfCondition(comparar),
             parameters=[parametros, {'modo': 'gazebo', 'use_sim_time': True}]),

        Node(package=pkg, executable='cinematica_directa.py', output='screen',
             parameters=[parametros, {'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(usar_rviz),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'rviz', 'planta_brazo.rviz')]),
    ])
