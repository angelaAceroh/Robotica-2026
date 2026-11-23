"""El control con el brazo de verdad y el boton «traycho» (TRAYCHO.md).

    ros2 launch scara_kinematics traycho.launch.py k:=2 vel_max:=0.1      # el brazo
    ros2 launch scara_kinematics traycho.launch.py simular:=true          # sin el brazo

Es control_placa.launch.py entero, con los mismos argumentos (puerto, simular,
k, vel_max, vel_z, activar, margen...), pero con su ventana cambiada por la de
traycho_gui: la misma, mas el boton. control_placa.launch.py no se ha tocado.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from scara_kinematics import alcance as A
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R


def ventana(context, *args, **kwargs):
    """La ventana, con los mismos topes que control_placa.launch.py le pone a
    control_nodo (los recorridos de los motores, con su margen)."""
    share = get_package_share_directory('scara_kinematics')
    robot_yaml = os.path.join(share, 'config', 'scara_urdf3.yaml')
    with open(robot_yaml) as f:
        rp = yaml.safe_load(f)['/**']['ros__parameters']
    with open(os.path.join(share, 'config', 'micropython.yaml')) as f:
        mp = yaml.safe_load(f)['/**']['ros__parameters']
    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    cal = A.Calibracion.desde_parametros(lambda nombre, defecto: mp.get(nombre, defecto))
    if LaunchConfiguration('husillo').perform(context).lower() == 'virtual':
        cal.paso_husillo_mm = 0.0
    lim = PV.limites_junta(cal, robot, float(LaunchConfiguration('margen').perform(context)))
    topes = {'q1_min': lim[0][0], 'q1_max': lim[0][1],
             'q2_min': lim[1][0], 'q2_max': lim[1][1],
             'd3_min': lim[2][0] * 1e-3, 'd3_max': lim[2][1] * 1e-3}
    return [Node(
        package='scara_kinematics', executable='traycho_gui', name='traycho_gui',
        output='screen',
        parameters=[robot_yaml, topes, {
            'k': float(LaunchConfiguration('k').perform(context)),
            'perfil': LaunchConfiguration('perfil').perform(context)}],
        on_exit=Shutdown(reason='se ha cerrado la ventana del control'))]


def generate_launch_description():
    placa = os.path.join(get_package_share_directory('scara_kinematics'), 'launch',
                         'control_placa.launch.py')
    return LaunchDescription([
        # Los que usa la ventana; con los mismos valores por defecto que alli
        DeclareLaunchArgument('k', default_value='3.0'),
        DeclareLaunchArgument('perfil', default_value='trapecio'),
        DeclareLaunchArgument('margen', default_value='0.5'),
        DeclareLaunchArgument('husillo', default_value='auto'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(placa),
                                 launch_arguments={'ventana': 'false'}.items()),
        OpaqueFunction(function=ventana),
    ])
