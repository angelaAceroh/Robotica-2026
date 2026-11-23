"""El SCARA en Gazebo (gz sim), con la simulacion siguiendo al brazo real.

    ros2 launch scara_kinematics gazebo.launch.py
    ros2 launch scara_kinematics gazebo.launch.py rviz:=true
    ros2 launch scara_kinematics gazebo.launch.py gui:=false      (gz sin ventana)
    ros2 launch scara_kinematics gazebo.launch.py brazo:=false    (solo simulacion)
    ros2 launch scara_kinematics gazebo.launch.py spawn_delay:=10 (maquina lenta)

Con brazo:=true (lo normal) arranca la cadena de los seis nodos y cc, ademas de
resolver la punta, refleja las tres juntas en los controladores de gz: el robot
simulado hace lo mismo que el de la mesa, y la diferencia entre los dos es lo
que se ve de un vistazo. Para que no se peleen por el mismo topic, lo que mide
gz sale por gz/joint_states y /joint_states lo publica cc, que es el brazo real.

Con brazo:=false no hay placa ni nodos: queda la simulacion sola, y las juntas se
mueven a mano publicando Float64 en /scara_urdf/cmd/<junta>. Ahi /joint_states lo
publica gz, que es el unico que mide algo.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODELO = 'scara_urdf'
URDF = 'scara_urdf_gz.urdf'


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    params_file = os.path.join(share, 'config', 'scara_urdf3.yaml')
    with open(params_file) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    juntas = list(params['joint_names'])
    cmd_prefix = params['cmd_prefix']

    urdf = os.path.join(get_package_share_directory(MODELO), 'urdf', URDF)
    with open(urdf) as f:
        descripcion = f.read()

    mundo = LaunchConfiguration('world').perform(context)
    nombre_mundo = os.path.splitext(os.path.basename(mundo))[0]
    js_gz = f'/world/{nombre_mundo}/model/{MODELO}/joint_state'

    con_brazo = LaunchConfiguration('brazo').perform(context).lower() in ('true', '1', 'yes')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    gz_args = f'-r -v 3 {mundo}' if gui else f'-r -s -v 3 {mundo}'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    # El puente se configura por YAML y no por "topic@tipo" porque los topics del
    # JointPositionController llevan un segmento numerico (".../0/cmd_pos") que
    # ROS 2 no acepta como nombre: hay que dar nombres distintos a cada lado.
    #
    # Con la cadena levantada, /joint_states es del brazo real (lo publica cc) y
    # lo de gz se aparta a gz/joint_states, que es lo que mira la ventana.
    entradas = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
         'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/gz/joint_states' if con_brazo else '/joint_states',
         'gz_topic_name': js_gz,
         'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model',
         'direction': 'GZ_TO_ROS'},
    ]
    for j in juntas:
        entradas.append({
            'ros_topic_name': f'{cmd_prefix}{j}',
            'gz_topic_name': f'/model/{MODELO}/joint/{j}/0/cmd_pos',
            'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
            'direction': 'ROS_TO_GZ'})

    cfg = os.path.join(tempfile.gettempdir(), f'{MODELO}_puente.yaml')
    with open(cfg, 'w') as f:
        yaml.safe_dump(entradas, f)

    sim = {'use_sim_time': True}
    espera = float(LaunchConfiguration('spawn_delay').perform(context))
    nodos = [
        gz_sim,
        # El robot_state_publisher lo lleva este launch y no brazo.launch.py porque
        # aqui el URDF que vale es el de gz, con sus plugins.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': descripcion}, sim]),
        # El spawn va con un pequeno retraso: "create" solo espera a que gz
        # conteste la lista de mundos, y el servicio que de verdad hace falta
        # (/world/<w>/create, del sistema UserCommands) se anuncia cerca de un
        # segundo despues. El margen es barato y evita depender de ese hueco.
        TimerAction(period=espera, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', MODELO,
                            '-x', '0', '-y', '0', '-z', '0'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': cfg}, sim]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             parameters=[sim],
             arguments=['-d', os.path.join(share, 'rviz', 'scara.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]

    if con_brazo:
        nodos.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'brazo.launch.py')),
            launch_arguments={'port': LaunchConfiguration('port'),
                              'hw': LaunchConfiguration('hw'),
                              'ventana': LaunchConfiguration('ventana'),
                              'registro': LaunchConfiguration('registro'),
                              'gz': 'true', 'sim': 'true', 'rsp': 'false'}.items()))
    return nodos


def _existe(pkg: str) -> bool:
    try:
        get_package_share_directory(pkg)
    except PackageNotFoundError:
        return False
    return True


def generate_launch_description():
    # Para que gz resuelva package://<pkg>/meshes/*.STL
    recursos = os.pathsep.join(
        os.path.dirname(get_package_share_directory(p))
        for p in ('scara_urdf', 'scara_kinematics') if _existe(p))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = gz sin ventana (solo servidor)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('brazo', default_value='true',
                              description='false = simulacion sola, sin los seis nodos'),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('hw', default_value='true'),
        DeclareLaunchArgument('ventana', default_value='true'),
        DeclareLaunchArgument('registro', default_value=''),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='segundos de margen para que gz cargue '
                                          'sus sistemas antes de meter el robot'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            recursos + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=launch_setup),
    ])
