"""El control cinematico con el brazo de verdad: control_nodo + puente_placa + la ventana.

    ros2 launch scara_kinematics control_placa.launch.py                     # el brazo
    ros2 launch scara_kinematics control_placa.launch.py simular:=true       # sin el brazo
    ros2 launch scara_kinematics control_placa.launch.py k:=2 vel_max:=0.1   # mas suave
    ros2 launch scara_kinematics control_placa.launch.py activar:=false      # motores sueltos
    ros2 launch scara_kinematics control_placa.launch.py ventana:=false tarea:="salto 420 -150 150"
    ros2 launch scara_kinematics control_placa.launch.py rviz:=true rqt:=true

En la placa tiene que estar main_control.py (CONTROL_PLACA.md §2), no el main.py
de `vector`.

    [control_placa_gui] --orden--> [control_nodo] --qd--> [puente_placa] --USB--> main_control.py
                                         ^                      |
                                         +--- /joint_states <---+

control_nodo es el MISMO de Gazebo (no se ha tocado), con otros parametros:

  - manda a /scara_urdf/placa/cmd_vel/<junta>, que recoge puente_placa;
  - sus topes son los recorridos de los motores (limites_motor, en
    config/micropython.yaml, los de los programas de Sergio) pasados a junta,
    con `margen` (grados o mm de motor) de margen: rechaza antes de mover lo que
    se salga, y en marcha no deja que una junta siga contra su tope;
  - techo de velocidad mas bajo (vel_max, rad/s en hombro y codo; vel_z, mm/s
    en Teodoro, que no pasa de unos 2 mm/s), K = 3, 50 Hz,
    sin tarea al arrancar y sin pintar en Gazebo;
  - el circulo por defecto, dentro de la franja que alcanza el brazo.

Todo con el reloj del PC: aqui no hay Gazebo. No se lanza a la vez que
brazo.launch.py ni efector.launch.py (el mismo puerto), ni que control.launch.py
(dos control_nodo con el mismo nombre).
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from scara_kinematics import alcance as A
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

PREFIJO = '/scara_urdf/placa/cmd_vel/'


def _si(context, nombre) -> bool:
    return LaunchConfiguration(nombre).perform(context).lower() in ('true', '1', 'yes')


def montar(context, *args, **kwargs):
    share = get_package_share_directory('scara_kinematics')
    robot_yaml = os.path.join(share, 'config', 'scara_urdf3.yaml')
    motores_yaml = os.path.join(share, 'config', 'micropython.yaml')
    with open(robot_yaml) as f:
        rp = yaml.safe_load(f)['/**']['ros__parameters']
    with open(motores_yaml) as f:
        mp = yaml.safe_load(f)['/**']['ros__parameters']

    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    cal = A.Calibracion.desde_parametros(lambda nombre, defecto: mp.get(nombre, defecto))
    husillo = LaunchConfiguration('husillo').perform(context).lower()
    if husillo == 'virtual':
        cal.paso_husillo_mm = 0.0          # la d3 de mentira: los topes del CAD
    margen = float(LaunchConfiguration('margen').perform(context))
    lim = PV.limites_junta(cal, robot, margen)
    zona = PV.zona_trabajo(PV.robot_con_limites(robot, lim))
    cx, cy, _ = zona['centro']
    radio = round(max(5.0, min(15.0, 0.5 * zona['holgura'])))
    topes = {'q1_min': lim[0][0], 'q1_max': lim[0][1],
             'q2_min': lim[1][0], 'q2_max': lim[1][1],
             'd3_min': lim[2][0] * 1e-3, 'd3_max': lim[2][1] * 1e-3}
    vel_max = float(LaunchConfiguration('vel_max').perform(context))
    vel_z = float(LaunchConfiguration('vel_z').perform(context)) * 1e-3     # m/s
    k = float(LaunchConfiguration('k').perform(context))
    perfil = LaunchConfiguration('perfil').perform(context)

    acciones = []
    if _si(context, 'simular'):
        # main_control.py de verdad, con motores simulados detras de un pty
        puerto = os.path.join(tempfile.gettempdir(), 'scara_placa_simulada')
        if os.path.islink(puerto):
            os.unlink(puerto)               # uno viejo podria apuntar a otro terminal
        fallo = LaunchConfiguration('fallo').perform(context)
        exe = os.path.join(get_package_prefix('scara_kinematics'), 'lib', 'scara_kinematics',
                           'placa_simulada')
        acciones.append(ExecuteProcess(
            cmd=[exe, '--enlace', puerto] + (['--fallo', fallo] if fallo else []),
            name='placa_simulada', output='screen'))
    else:
        puerto = LaunchConfiguration('puerto').perform(context)

    acciones.append(Node(
        package='scara_kinematics', executable='puente_placa', name='puente_placa',
        output='screen',
        parameters=[robot_yaml, motores_yaml, {
            'puerto': puerto, 'cmd_vel_prefix': PREFIJO, 'husillo': husillo,
            'activar': _si(context, 'activar')}]))
    acciones.append(Node(
        package='scara_kinematics', executable='control_nodo', name='control',
        output='screen',
        parameters=[robot_yaml, topes, {
            'cmd_vel_prefix': PREFIJO,
            'max_vel_hw': [vel_max, vel_max, vel_z, 2.0],
            'k': k, 'perfil': perfil,
            'tarea': LaunchConfiguration('tarea').perform(context),
            'dt': float(LaunchConfiguration('dt').perform(context)),
            'registro': LaunchConfiguration('registro').perform(context),
            'espera': 1.0, 'pintar_gz': False,
            'circulo': [round(cx), round(cy), float(radio)], 'z_circulo': R.Z_SEGURO,
            'v_ir': 20.0, 'v_circulo': 10.0}]))
    if _si(context, 'ventana'):
        acciones.append(Node(
            package='scara_kinematics', executable='control_placa_gui',
            name='control_placa_gui', output='screen',
            parameters=[robot_yaml, topes, {'k': k, 'perfil': perfil}],
            on_exit=Shutdown(reason='se ha cerrado la ventana del control')))
    if _si(context, 'rviz'):
        urdf = os.path.join(get_package_share_directory('scara_urdf'), 'urdf', 'scara_urdf.urdf')
        with open(urdf) as f:
            descripcion = f.read()
        acciones += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='log', parameters=[{'robot_description': descripcion}]),
            Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                 arguments=['-d', os.path.join(share, 'rviz', 'control.rviz')])]
    if _si(context, 'rqt'):
        acciones.append(Node(
            package='rqt_gui', executable='rqt_gui', name='rqt_control', output='log',
            arguments=['--perspective-file', os.path.join(share, 'rqt', 'control.perspective')]))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('puerto', default_value='',
                              description='vacio = buscar la ESP32 sola'),
        DeclareLaunchArgument('simular', default_value='false',
                              description='true = sin el brazo: main_control.py con motores '
                                          'simulados (placa_simulada)'),
        DeclareLaunchArgument('fallo', default_value='',
                              description='con simular: alvin_inv | simon_sin_encoder'),
        DeclareLaunchArgument('k', default_value='3.0', description='ganancia K [1/s]'),
        DeclareLaunchArgument('vel_max', default_value='0.2',
                              description='techo de hombro y codo [rad/s]'),
        # 21/10: con 8 mm/s Teodoro se quedaba atras (hace unos 2 mm/s) y el
        # vigilante de la placa soltaba los motores. Sus programas, hasta 1 mm/s
        DeclareLaunchArgument('vel_z', default_value='1.0',
                              description='techo de Teodoro [mm/s]'),
        DeclareLaunchArgument('tarea', default_value='nada',
                              description='la primera orden de control_nodo (nada = ninguna)'),
        DeclareLaunchArgument('perfil', default_value='trapecio'),
        DeclareLaunchArgument('dt', default_value='0.02',
                              description='periodo de control_nodo [s]; la placa mide a 50 Hz'),
        DeclareLaunchArgument('registro', default_value='',
                              description='CSV de control_nodo, tic a tic'),
        DeclareLaunchArgument('activar', default_value='true',
                              description='false = los motores se quedan sueltos hasta '
                                          '«Activar motores»'),
        DeclareLaunchArgument('husillo', default_value='auto',
                              description='auto | virtual | real. virtual = Teodoro '
                                          'quieto y la z de mentira (para probar el plano)'),
        DeclareLaunchArgument('margen', default_value='0.5',
                              description='grados (Alvin, Simon) o mm (Teodoro) que se '
                                          'dejan antes de cada tope'),
        DeclareLaunchArgument('ventana', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rqt', default_value='false'),
        OpaqueFunction(function=montar),
    ])
