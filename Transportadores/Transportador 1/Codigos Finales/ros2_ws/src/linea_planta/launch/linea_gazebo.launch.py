"""
LINEA DE ENSAMBLE en Gazebo: los cinco robots en su sitio, y el nodo maestro.

    ros2 launch linea_planta linea_gazebo.launch.py                  # solo la planta
    ros2 launch linea_planta linea_gazebo.launch.py maestro:=true    # + nodo maestro
    ros2 launch linea_planta linea_gazebo.launch.py maestro:=true \
        simulados:=transportador2,ensamblador1,ensamblador2,ensamblador3
                                     # el transportador 1 es el brazo real
    ros2 launch linea_planta linea_gazebo.launch.py escala:=1.0      # plano literal
    ros2 launch linea_planta linea_gazebo.launch.py gui:=false       # sin ventana

DE DONDE SALEN LAS POSICIONES
    Del plano del repositorio comun, "PLANO PLANTA ROBOTICA - LAB 1"
    (Laboratorio-1-Robotica-UMNG/media/Plano.png).  Esta en centimetros, con el
    origen en la esquina inferior izquierda de la mesa (86.22 x 85.52), X a la
    derecha e Y hacia arriba.  Aqui se usa ese mismo sistema: el origen de
    Gazebo es esa esquina.

    Los centros de los robots estan acotados en el plano.  Las estaciones
    (entrada, salida, bases con la caja, almacenes) no tienen todas las cotas
    y se midieron sobre la imagen (6.98 px/cm, comprobado con las cotas).

ESCALA
    El plano dibuja el alcance de los transportadores como R25 y el de los
    ensambladores como R26.  Los robots reales alcanzan 0.33-0.58 m segun sus
    URDF, asi que el plano en cm tal cual (escala 1.0) deja los brazos metidos
    unos en otros.  La especificacion del transportador 1 es un circulo de
    D700 mm (R350): 350 / 250 = 1.4, que es la escala por defecto.  Si el
    plano real es otro, se cambia aqui sin tocar nada mas.

LOS ROBOTS EN GAZEBO
    Cada robot escucha sus consignas en  /<robot>/<articulacion>/cmd_pos
    (gz.msgs.Double, radianes o metros).  El nuestro conserva sus topicos de
    siempre, /planta_brazo/J1/cmd_pos ...
    El nodo maestro y los robot_simulado NO mueven los robots de Gazebo:
    coordinan por /<robot>/orden y /<robot>/estado.

    EXCEPCION: el transportador 1.  nodo_transportador1.py --gazebo publica la
    postura MEDIDA del brazo fisico en /planta_brazo/J*/cmd_pos (ROS), y aqui
    hay un puente que la pasa a Gazebo: el brazo de la simulacion copia al
    real.
"""
import math
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MUNDO = 'linea'          # debe coincidir con <world name=...> de worlds/linea.sdf

# ------------------------------------------------------------------ ROBOTS
#  nombre, urdf (None = el nuestro, desde su xacro), centro en el plano (cm),
#  orientacion (grados), desfase del centro real respecto al origen de SU
#  modelo (m, en su propio marco).
#
#  El transportador 2 lleva desfase: su exportador de SolidWorks dejo el
#  robot entero 1.11 m desplazado respecto al origen (la malla de su base va
#  de y=-1.22 a y=-1.00, y su primera articulacion esta en y=-1.1147).  Se
#  compensa aqui para que su EJE quede en su sitio, sin tocar su modelo.
ROBOTS = [
    ('transportador1', None,                   (54.53, 64.02), 0.0, (0.0, 0.0)),
    ('ensamblador1',   'ensamblador1.urdf',    (19.62, 71.18), 0.0, (0.0, 0.0)),
    ('ensamblador2',   'ensamblador2.urdf',    (62.94, 42.12), 0.0, (0.0, 0.0)),
    ('transportador2', 'transportador2.urdf',  (54.53, 21.02), 0.0, (0.002426, -1.1147)),
    ('ensamblador3',   'ensamblador3.urdf',    (19.62, 12.43), 0.0, (0.0, 0.0)),
]

# --------------------------------------------------------------- ESTACIONES
#  Solo visuales, planas, para orientarse.  (centro x, centro y, ancho, alto)
#  en cm del plano, y color.
MESA = (86.22, 85.52)
ESTACIONES = [
    ('entrada', (59.8, 85.6, 25.0, 15.0), (0.10, 0.45, 0.20)),   # P.I.
    ('salida',  (59.8, -0.4, 25.0, 15.0), (0.10, 0.45, 0.20)),   # P.F.
    ('base1',   (38.5, 71.3, 15.0, 25.0), (0.35, 0.85, 0.35)),   # caja en E1
    ('base2',   (45.0, 42.2, 15.0, 25.0), (0.35, 0.85, 0.35)),
    ('base3',   (38.5, 12.5, 15.0, 25.0), (0.35, 0.85, 0.35)),
]
ALMACENES = [(19.62, 49.42), (19.62, 29.48), (69.95, 29.48)]   # magenta
R_ALMACEN = 2.0


def _sdf_planta(e):
    """Mesa + estaciones + almacenes en un solo modelo estatico."""
    m = lambda cm: cm * e / 100.0
    vis = []

    def caja(nombre, cx, cy, sx, sy, sz, z, rgb):
        vis.append(
            '<visual name="%s"><pose>%f %f %f 0 0 0</pose>'
            '<geometry><box><size>%f %f %f</size></box></geometry>'
            '<material><ambient>%g %g %g 1</ambient><diffuse>%g %g %g 1</diffuse>'
            '</material></visual>'
            % (nombre, cx, cy, z, sx, sy, sz, *rgb, *rgb))

    caja('mesa', m(MESA[0] / 2), m(MESA[1] / 2), m(MESA[0]), m(MESA[1]), 0.02, -0.01,
         (0.80, 0.78, 0.72))
    for nombre, (cx, cy, sx, sy), rgb in ESTACIONES:
        caja(nombre, m(cx), m(cy), m(sx), m(sy), 0.004, 0.002, rgb)
    for k, (cx, cy) in enumerate(ALMACENES):
        vis.append(
            '<visual name="almacen%d"><pose>%f %f 0.004 0 0 0</pose>'
            '<geometry><cylinder><radius>%f</radius><length>0.008</length></cylinder>'
            '</geometry><material><ambient>0.85 0.10 0.85 1</ambient>'
            '<diffuse>0.85 0.10 0.85 1</diffuse></material></visual>'
            % (k + 1, m(cx), m(cy), m(R_ALMACEN)))
    return ('<?xml version="1.0"?><sdf version="1.10"><model name="planta">'
            '<static>true</static><link name="link">%s</link></model></sdf>'
            % ''.join(vis))


def _lanzar(context):
    e = float(LaunchConfiguration('escala').perform(context))
    share = get_package_share_directory('linea_planta')
    share_brazo = get_package_share_directory('planta_brazo')

    acciones = []
    acciones.append(Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', MUNDO, '-name', 'planta', '-string', _sdf_planta(e)]))

    for nombre, urdf, (cx, cy), yaw_g, (dx, dy) in ROBOTS:
        yaw = math.radians(yaw_g)
        # el origen del modelo va donde haga falta para que su CENTRO real
        # (su eje J1) caiga en el punto del plano
        x = cx * e / 100.0 - (math.cos(yaw) * dx - math.sin(yaw) * dy)
        y = cy * e / 100.0 - (math.sin(yaw) * dx + math.cos(yaw) * dy)
        if urdf is None:
            xacro = os.path.join(share_brazo, 'urdf', 'planta_brazo.urdf.xacro')
            fuente = ['-string', subprocess.check_output(['xacro', xacro], text=True)]
        else:
            fuente = ['-file', os.path.join(share, 'urdf', urdf)]
        acciones.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            name='crear_' + nombre,
            arguments=['-world', MUNDO, '-name', nombre, *fuente,
                       '-x', '%.4f' % x, '-y', '%.4f' % y, '-z', '0.0',
                       '-Y', '%.4f' % yaw]))
        print('[linea] %-15s en (%.3f, %.3f) m, %+.0f grados'
              % (nombre, cx * e / 100.0, cy * e / 100.0, yaw_g))

    # puente ROS -> Gazebo para el espejo del brazo fisico (transportador 1)
    acciones.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='puente_t1',
        output='screen',
        arguments=['/planta_brazo/%s/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double' % j
                   for j in ('J1', 'J2', 'J3')]))

    if LaunchConfiguration('maestro').perform(context).lower() in ('true', '1'):
        acciones.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('linea_maestro'),
                             'launch', 'prueba_maestro.launch.py')),
            launch_arguments={'simulados': LaunchConfiguration('simulados')}.items()))
    return acciones


def generate_launch_description():
    share = get_package_share_directory('linea_planta')
    share_brazo = get_package_share_directory('planta_brazo')

    # Gazebo resuelve  package://<paquete>/meshes/...  buscando una carpeta
    # <paquete> en GZ_SIM_RESOURCE_PATH: las de los otros equipos estan en
    # share/linea_planta/modelos/, la nuestra en share/planta_brazo.
    rutas = [os.path.join(share, 'modelos'), os.path.dirname(share_brazo)]
    previa = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if previa:
        rutas.append(previa)

    return LaunchDescription([
        DeclareLaunchArgument('escala', default_value='1.4',
                              description='multiplica las cotas del plano (cm)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo sin ventana'),
        DeclareLaunchArgument('maestro', default_value='false',
                              description='true: lanza tambien el nodo maestro '
                                          'y los robots simulados'),
        DeclareLaunchArgument('simulados', default_value='todos',
                              description='con maestro:=true, que robots se '
                                          'simulan (comas, "todos" o "ninguno")'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(rutas)),
        OpaqueFunction(function=_lanzar_gazebo),
        OpaqueFunction(function=_lanzar),
    ])


def _lanzar_gazebo(context):
    share = get_package_share_directory('linea_planta')
    mundo = os.path.join(share, 'worlds', MUNDO + '.sdf')
    sin_ventana = LaunchConfiguration('gui').perform(context).lower() in ('false', '0')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': mundo + ' -r -v 2' + (' -s' if sin_ventana else ''),
                          'on_exit_shutdown': 'true'}.items())]
