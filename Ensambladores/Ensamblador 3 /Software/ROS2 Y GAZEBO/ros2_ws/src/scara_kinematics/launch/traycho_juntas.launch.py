"""El brazo de verdad POR JUNTAS, con el boton «traycho» (TRAYCHO_JUNTAS.md).

    ros2 launch scara_kinematics traycho_juntas.launch.py                    # el brazo
    ros2 launch scara_kinematics traycho_juntas.launch.py activar:=false     # motores sueltos
    ros2 launch scara_kinematics traycho_juntas.launch.py simular:=true      # sin el brazo

    [juntas_gui] --orden--> [juntas_nodo] --v--> [puente_placa] --USB--> main_control.py
                                  ^                     |
                                  +-- /joint_states <---+

Es control_placa.launch.py (que no se ha tocado) con juntas_nodo en el sitio de
control_nodo y la ventana de juntas_gui. Cada junta va a su velocidad fija (las
`velocidades` de config/micropython.yaml) y se corrige al llegar.

Al puente se le sube vel_max_motor a lo que deja la placa (VEL_MAX de
main_control.py): con el de control_nodo, 15 grados/s, recortaria los 84 grados/s
de Alvin.

Simon va LENTO (24/10, «limita a Simon, mejor lento pero seguro»: su transmision
resbala): a vel_simon grados/s (30) en vez de los 180 de sus 30 RPM, y el puente
no le deja pasar de un 20 % mas. Solo aqui: config/micropython.yaml no cambia
(sus velocidades son las de `vector`). Para otra: vel_simon:=60.

Alvin tambien va LENTO (24/10, Sergio: «si vas muy rapido la circunferencia de
soporte se cae»): a vel_alvin grados/s (30) en vez de los 84 de sus 14 RPM, y
el puente no le deja pasar de un 20 % mas. Para otra: vel_alvin:=45.

HOME mas lejos de la base (24/10, tarde): en Simon 0 el antebrazo toca la base.
P1 (el HOME) es ahora juntas.P1_MM, con Simon en -27.5, y Simon no pasa de
simon_max (-5) hacia la base.

Sin Teodoro (24/10, tiene una falla mecanica): husillo:=virtual activa solo
Alvin y Simon, y la z la integra el puente con lo que se pide (de mentira).
z_inicial:=220 es la z de la que parte (arriba, como P1).
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from scara_kinematics import alcance as A
from scara_kinematics import juntas as J
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

PREFIJO = '/scara_urdf/placa/cmd_vel/'
VEL_MAX_MOTOR = [90.0, 190.0, 10.0]      # = VEL_MAX de la tabla MOTORES de main_control.py
SIMON_LENTO = 30.0                       # grados/s de Simon (con 30 RPM serian 180)
ALVIN_LENTO = 30.0                       # grados/s de Alvin (con 14 RPM serian 84)


def simon_lento(velocidades, vel_simon):
    """Las RPM para juntas_nodo y el vel_max_motor para el puente, con Simon a
    vel_simon grados/s como mucho (y el puente, un 20 % mas)."""
    rpm = [float(v) for v in velocidades]
    rpm[1] = min(rpm[1], vel_simon / A.POR_SEGUNDO[1])
    vel_max = list(VEL_MAX_MOTOR)
    vel_max[1] = min(vel_max[1], 1.2 * rpm[1] * A.POR_SEGUNDO[1])
    return rpm, vel_max


def alvin_lento(rpm, vel_max, vel_alvin):
    """Lo mismo para Alvin, sobre lo que da simon_lento: a vel_alvin grados/s
    como mucho (y el puente, un 20 % mas)."""
    rpm = [float(v) for v in rpm]
    vel_max = [float(v) for v in vel_max]
    rpm[0] = min(rpm[0], vel_alvin / A.POR_SEGUNDO[0])
    vel_max[0] = min(vel_max[0], 1.2 * rpm[0] * A.POR_SEGUNDO[0])
    return rpm, vel_max


def simon_lejos(cal, q2, simon_max):
    """Los topes de Simon en junta (rad) con su lado de la base (el MAX de la
    placa: 0 es el antebrazo contra la base) en simon_max como mucho."""
    q2 = [float(v) for v in q2]
    base = cal.a_junta([0.0, cal.limites[1][1], 0.0])[1]
    tope = cal.a_junta([0.0, simon_max, 0.0])[1]
    if abs(base - q2[0]) < abs(base - q2[1]):
        q2[0] = max(q2[0], tope)
    else:
        q2[1] = min(q2[1], tope)
    return q2


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
    lim = PV.limites_junta(cal, robot, float(LaunchConfiguration('margen').perform(context)))
    # Teodoro, ademas, no por debajo de teodoro_min (el soporte cilindrico)
    d3_min = max(lim[2][0], cal.a_junta(
        [0.0, 0.0, float(LaunchConfiguration('teodoro_min').perform(context))])[2])
    q2 = simon_lejos(cal, lim[1], float(LaunchConfiguration('simon_max').perform(context)))
    topes = {'q1_min': lim[0][0], 'q1_max': lim[0][1],
             'q2_min': q2[0], 'q2_max': q2[1],
             'd3_min': d3_min * 1e-3, 'd3_max': lim[2][1] * 1e-3}

    velocidades, vel_max = simon_lento(
        mp['velocidades'], float(LaunchConfiguration('vel_simon').perform(context)))
    velocidades, vel_max = alvin_lento(
        velocidades, vel_max, float(LaunchConfiguration('vel_alvin').perform(context)))

    acciones = [LogInfo(msg=f'Simon lento: {velocidades[1] * A.POR_SEGUNDO[1]:g} grados/s '
                            f'(el puente le deja hasta {vel_max[1]:g}); vel_simon:=... '
                            f'para otra'),
                LogInfo(msg=f'Alvin lento: {velocidades[0] * A.POR_SEGUNDO[0]:g} grados/s '
                            f'(el puente le deja hasta {vel_max[0]:g}); vel_alvin:=... '
                            f'para otra')]
    if _si(context, 'simular'):
        # main_control.py de verdad, con motores simulados detras de un pty
        puerto = os.path.join(tempfile.gettempdir(), 'scara_placa_simulada')
        if os.path.islink(puerto):
            os.unlink(puerto)
        exe = os.path.join(get_package_prefix('scara_kinematics'), 'lib', 'scara_kinematics',
                           'placa_simulada')
        acciones.append(ExecuteProcess(cmd=[exe, '--enlace', puerto],
                                       name='placa_simulada', output='screen'))
    else:
        puerto = LaunchConfiguration('puerto').perform(context)

    puente = {'puerto': puerto, 'cmd_vel_prefix': PREFIJO,
              'husillo': LaunchConfiguration('husillo').perform(context),
              'activar': _si(context, 'activar'), 'vel_max_motor': vel_max}
    z0 = LaunchConfiguration('z_inicial').perform(context)
    if z0:
        puente['z_inicial'] = float(z0)
    acciones.append(Node(
        package='scara_kinematics', executable='puente_placa', name='puente_placa',
        output='screen',
        parameters=[robot_yaml, motores_yaml, puente]))
    acciones.append(Node(
        package='scara_kinematics', executable='juntas_nodo', name='control',
        output='screen',
        parameters=[robot_yaml, motores_yaml, topes, {
            'cmd_vel_prefix': PREFIJO, 'velocidades': velocidades,
            'dt': float(LaunchConfiguration('dt').perform(context))}]))
    acciones.append(Node(
        package='scara_kinematics', executable='juntas_gui', name='juntas_gui',
        output='screen',
        parameters=[robot_yaml, topes],
        on_exit=Shutdown(reason='se ha cerrado la ventana')))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('puerto', default_value='',
                              description='vacio = buscar la ESP32 sola'),
        DeclareLaunchArgument('simular', default_value='false',
                              description='true = sin el brazo: main_control.py con motores '
                                          'simulados (placa_simulada)'),
        DeclareLaunchArgument('dt', default_value='0.02',
                              description='periodo de juntas_nodo [s]; la placa mide a 50 Hz'),
        DeclareLaunchArgument('activar', default_value='true',
                              description='false = los motores se quedan sueltos hasta '
                                          '«Activar motores»'),
        DeclareLaunchArgument('teodoro_min', default_value=str(J.TEODORO_MIN_MM),
                              description='mm: Teodoro no baja de aqui (0 = arriba del '
                                          'todo); lleva un soporte cilindrico debajo'),
        DeclareLaunchArgument('simon_max', default_value=str(J.SIMON_MAX),
                              description='grados de la placa: Simon no se acerca mas a '
                                          'la base (0 = el antebrazo contra ella)'),
        DeclareLaunchArgument('margen', default_value='0.5',
                              description='grados (Alvin, Simon) o mm (Teodoro) que se '
                                          'dejan antes de cada tope'),
        DeclareLaunchArgument('husillo', default_value='auto',
                              description='virtual = sin Teodoro: solo Alvin y Simon, '
                                          'y la z de mentira'),
        DeclareLaunchArgument('z_inicial', default_value='',
                              description='mm: con husillo:=virtual, la z de la que parte '
                                          '(vacio = la del puente, 150)'),
        DeclareLaunchArgument('vel_simon', default_value=str(SIMON_LENTO),
                              description='grados/s de Simon (con sus 30 RPM del yaml, '
                                          '180): lento pero seguro, su transmision '
                                          'resbala'),
        DeclareLaunchArgument('vel_alvin', default_value=str(ALVIN_LENTO),
                              description='grados/s de Alvin (con sus 14 RPM del yaml, '
                                          '84): deprisa, se cae la circunferencia de '
                                          'soporte'),
        OpaqueFunction(function=montar),
    ])
