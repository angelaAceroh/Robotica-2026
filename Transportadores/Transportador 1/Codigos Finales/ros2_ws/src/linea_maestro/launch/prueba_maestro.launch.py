"""
Nodo maestro, mas un robot_simulado por cada robot que NO este conectado.

    ros2 launch linea_maestro prueba_maestro.launch.py
        -> el maestro y los 5 robots simulados (la prueba de siempre)

    ros2 launch linea_maestro prueba_maestro.launch.py \
        simulados:=transportador2,ensamblador1,ensamblador2,ensamblador3
        -> el transportador 1 lo pone nodo_transportador1.py (el brazo real)

    ros2 launch linea_maestro prueba_maestro.launch.py simulados:=ninguno
        -> solo el maestro: los cinco robots son reales

Un robot que este conectado de verdad NO debe ir en 'simulados': habria dos
nodos contestando en /<robot>/estado y el maestro no sabria a cual creer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PAQUETE = 'linea_maestro'

# (nombre del robot, segundos que tarda cada tarea)
ROBOTS = [
    ('transportador1', 3.0),
    ('transportador2', 3.0),
    ('ensamblador1',   2.0),
    ('ensamblador2',   2.0),
    ('ensamblador3',   2.0),
]


def _nodos(context):
    texto = LaunchConfiguration('simulados').perform(context).strip()
    if texto in ('todos', ''):
        simulados = [n for n, _ in ROBOTS]
    elif texto == 'ninguno':
        simulados = []
    else:
        simulados = [s.strip() for s in texto.split(',') if s.strip()]
    desconocidos = [s for s in simulados if s not in dict(ROBOTS)]
    if desconocidos:
        raise RuntimeError('simulados: no conozco %s (validos: %s)'
                           % (desconocidos, ', '.join(n for n, _ in ROBOTS)))
    reales = [n for n, _ in ROBOTS if n not in simulados]
    print('[maestro] simulados: %s' % (', '.join(simulados) or 'ninguno'))
    print('[maestro] reales (los pone cada equipo): %s' % (', '.join(reales) or 'ninguno'))

    nodos = [
        Node(package=PAQUETE, executable='robot_simulado',
             name=f'sim_{nombre}', output='screen',
             parameters=[{'nombre': nombre, 'duracion': duracion}])
        for nombre, duracion in ROBOTS if nombre in simulados
    ]
    repetir = LaunchConfiguration('repetir').perform(context).lower() in ('true', '1')
    nodos.append(
        Node(package=PAQUETE, executable='maestro', name='maestro', output='screen',
             parameters=[{'repetir': repetir}]))
    return nodos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulados', default_value='todos',
                              description='robots a simular, separados por comas; '
                                          '"todos" o "ninguno"'),
        DeclareLaunchArgument('repetir', default_value='false',
                              description='true: repetir el ciclo sin parar'),
        OpaqueFunction(function=_nodos),
    ])
