"""El control cinematico del SCARA en Gazebo, con las juntas movidas por VELOCIDAD.

    ros2 launch scara_kinematics control.launch.py                     # un circulo
    ros2 launch scara_kinematics control.launch.py tarea:=perforado
    ros2 launch scara_kinematics control.launch.py tarea:="salto 320 30 150"
    ros2 launch scara_kinematics control.launch.py k:=10 rqt:=true rviz:=true
    ros2 launch scara_kinematics control.launch.py gui:=false registro:=~/control.csv

Es un Gazebo aparte del de gazebo.launch.py, y la diferencia es una sola: los
controladores de las juntas. El de siempre mueve cada junta con un PID de
POSICION; el control cinematico manda VELOCIDADES de junta (qd = J^-1 ...), y
encima de un PID de posicion serian dos lazos peleandose. Aqui hombro, codo y
husillo llevan gz-sim-joint-controller-system en modo velocidad: la junta hace
la velocidad que se le pide, que es justo lo que supone la teoria.

Para eso se lee scara_urdf_gz.urdf y se escribe una copia en /tmp cambiando esos
tres plugins (la muneca sigue con el suyo, quieta en su cero). Ni el URDF ni
gazebo.launch.py se tocan.

    control_nodo --Float64--> /scara_urdf/control/cmd_vel/<junta>
                              --> /model/scara_urdf/joint/<junta>/cmd_vel  (gz)
    gz --> /world/<w>/model/scara_urdf/joint_state --> /joint_states --> control_nodo

No hay placa ni cadena de los seis nodos (solo Gazebo). No se lanza a la vez
que gazebo.launch.py, ruta.launch.py ni trayectoria.launch.py: serian dos
robots con el mismo nombre en dos gz distintos y los topics se cruzarian.
"""
import math
import os
import re
import shutil
import tempfile

import yaml
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODELO = 'scara_urdf'
URDF = 'scara_urdf_gz.urdf'

_PLUGIN_POS = re.compile(
    r'(?:<!--(?:(?!-->).)*-->\s*)?'           # el comentario de encima, si lo hay
    r'<plugin filename="gz-sim-joint-position-controller-system"\s+'
    r'name="gz::sim::systems::JointPositionController">(.*?)</plugin>', re.S)


def urdf_por_velocidad(texto: str, juntas) -> str:
    """El URDF de gz con esas juntas movidas por velocidad en vez de por posicion.

    use_force_commands=false: gz impone la velocidad directamente (hasta el
    esfuerzo maximo del <limit> de la junta), sin PID de por medio.
    """
    cambiadas = []

    def cambiar(m):
        j = re.search(r'<joint_name>\s*(.*?)\s*</joint_name>', m.group(1)).group(1)
        if j not in juntas:
            return m.group(0)
        cambiadas.append(j)
        return ('<plugin filename="gz-sim-joint-controller-system"\n'
                '            name="gz::sim::systems::JointController">\n'
                f'      <joint_name>{j}</joint_name>\n'
                '      <use_force_commands>false</use_force_commands>\n'
                '    </plugin>')

    nuevo = _PLUGIN_POS.sub(cambiar, texto)
    faltan = set(juntas) - set(cambiadas)
    if faltan:
        raise RuntimeError(f'{URDF} no tiene controlador de posicion para {sorted(faltan)}: '
                           f'no se puede pasar a velocidad')
    return nuevo


def _camara(desde, hacia):
    """Pose de la camara de gz en `desde` mirando a `hacia` (gz mira por su +X)."""
    dx, dy, dz = (b - a for a, b in zip(desde, hacia))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    x, y, z = desde
    return (f'pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: '
            f'{{x: {-sy * sp:.5f} y: {cy * sp:.5f} z: {sy * cp:.5f} w: {cy * cp:.5f}}}}}')


def montar(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    params_file = os.path.join(share, 'config', 'scara_urdf3.yaml')
    with open(params_file) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    juntas = list(params['joint_names'])
    prefijo = '/scara_urdf/control/cmd_vel/'

    with open(os.path.join(get_package_share_directory(MODELO), 'urdf', URDF)) as f:
        descripcion = urdf_por_velocidad(f.read(), juntas)
    copia = os.path.join(tempfile.gettempdir(), f'{MODELO}_control_vel.urdf')
    with open(copia, 'w') as f:
        f.write(descripcion)

    mundo = LaunchConfiguration('world').perform(context)
    nombre_mundo = os.path.splitext(os.path.basename(mundo))[0]
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('no encuentro el comando gz: falta Gazebo (gz sim)')
    # Lo mismo que hace ros_gz_sim/gz_sim.launch.py, pero SIN shell. Aquel lanza
    # `sh -c "ruby gz sim ..."`: al cerrar, el SIGINT le llega al sh y no al
    # servidor, a los 5 s el launch mata el sh y gz-sim-main se queda huerfano,
    # comiendo CPU. El siguiente arranque choca con el ("Another world of the
    # same name is running") y `create` se queda pidiendo la lista de mundos.
    # Sin shell, ruby hace exec de gz-sim-main y el SIGINT le llega a el: se
    # cierra en una decima de segundo.
    gz_sim = ExecuteProcess(
        cmd=['ruby', gz, 'sim', '-r', '-v', '3'] + ([] if gui else ['-s'])
        + [mundo, '--force-version', '10'],
        name='gazebo', output='screen',
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', '')])})

    # Por YAML, como en gazebo.launch.py: nombres distintos a cada lado.
    entradas = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
         'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/joint_states',
         'gz_topic_name': f'/world/{nombre_mundo}/model/{MODELO}/joint_state',
         'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model',
         'direction': 'GZ_TO_ROS'},
    ]
    for j in juntas:
        entradas.append({
            'ros_topic_name': f'{prefijo}{j}',
            'gz_topic_name': f'/model/{MODELO}/joint/{j}/cmd_vel',
            'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
            'direction': 'ROS_TO_GZ'})
    cfg = os.path.join(tempfile.gettempdir(), f'{MODELO}_control_puente.yaml')
    with open(cfg, 'w') as f:
        yaml.safe_dump(entradas, f)

    sim = {'use_sim_time': True}
    espera = float(LaunchConfiguration('spawn_delay').perform(context))
    acciones = [
        gz_sim,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': descripcion}, sim]),
        # El retraso, por lo mismo que en gazebo.launch.py: el servicio de crear
        # aparece algo despues de que gz conteste.
        TimerAction(period=espera, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-file', copia, '-name', MODELO, '-x', '0', '-y', '0', '-z', '0'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': cfg}, sim]),
        Node(package='scara_kinematics', executable='control_nodo', name='control',
             output='screen',
             parameters=[params_file, sim, {
                 'tarea': ParameterValue(LaunchConfiguration('tarea'), value_type=str),
                 'k': ParameterValue(LaunchConfiguration('k'), value_type=float),
                 'perfil': ParameterValue(LaunchConfiguration('perfil'), value_type=str),
                 'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
                 'espera': ParameterValue(LaunchConfiguration('espera'), value_type=float),
                 'registro': ParameterValue(LaunchConfiguration('registro'), value_type=str),
                 'pintar_gz': gui}]),
    ]
    if gui:
        # Encuadre: cerca, en diagonal y algo desde arriba, para ver a la vez el
        # plano y z. La ventana tarda en ofrecer el servicio de la camara: se
        # espera a que aparezca (30 s como mucho). Luego se gira con el raton.
        pedir = ('gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera '
                 '--reptype gz.msgs.Boolean --timeout 3000 --req "'
                 + _camara((0.62, -0.42, 0.52), (0.30, 0.0, 0.12)) + '"')
        acciones.append(ExecuteProcess(
            cmd=['bash', '-c', 'for i in $(seq 60); do gz service -l 2>/dev/null | '
                 'grep -q "^/gui/move_to/pose$" && break; sleep 0.5; done; ' + pedir],
            output='log'))
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        acciones.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            parameters=[sim], arguments=['-d', os.path.join(share, 'rviz', 'control.rviz')]))
    if LaunchConfiguration('rqt').perform(context).lower() in ('true', '1'):
        # Cuatro rqt_plot: x e y pedidas y medidas, z, el error y qd.
        acciones.append(Node(
            package='rqt_gui', executable='rqt_gui', name='rqt_control', output='log',
            arguments=['--perspective-file', os.path.join(share, 'rqt', 'control.perspective')]))
    return acciones


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
        DeclareLaunchArgument('tarea', default_value='circulo',
                              description='la primera orden: circulo | perforado | contorno '
                                          '| home | "ir X Y Z" | "salto X Y Z" | nada'),
        DeclareLaunchArgument('k', default_value='5.0',
                              description='ganancia K [1/s]; tau = 1/K'),
        DeclareLaunchArgument('perfil', default_value='trapecio',
                              description='lineal | trapecio | cubico | quintico | septico '
                                          '| scurve'),
        DeclareLaunchArgument('dt', default_value='0.01',
                              description='periodo del lazo [s] (0.01 = 100 Hz)'),
        DeclareLaunchArgument('espera', default_value='6.0',
                              description='margen para que gz cargue el robot'),
        DeclareLaunchArgument('registro', default_value='',
                              description='CSV con lo pedido y lo medido, tic a tic'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = gz sin ventana'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rqt', default_value='false',
                              description='true = rqt con las graficas del control'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            recursos + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=montar),
    ])
