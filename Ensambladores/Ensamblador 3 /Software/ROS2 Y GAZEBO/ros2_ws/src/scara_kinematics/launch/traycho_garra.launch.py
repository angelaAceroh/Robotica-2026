"""El traycho por juntas CON LA GARRA (GARRA.md).

    ros2 launch scara_kinematics traycho_garra.launch.py                    # el brazo
    ros2 launch scara_kinematics traycho_garra.launch.py activar:=false     # motores sueltos
    ros2 launch scara_kinematics traycho_garra.launch.py simular:=true      # sin el brazo

    [garra_gui] --orden--> [juntas_nodo] --v--> [puente_garra] --USB--> main_control.py
         |                                        ^                       + garra.py
         +-------- 1 / 0 (scara/placa/orden) -----+

Es traycho_juntas.launch.py (que no se ha tocado) TAL CUAL, con los mismos
argumentos (vel_simon, vel_alvin, teodoro_min, husillo...), cambiando dos nodos:

    puente_placa -> puente_garra   el mismo, que ademas pasa el 1 y el 0 a la placa
    juntas_gui   -> garra_gui      la misma ventana, con la garra

teodoro_min, por defecto, es otro: GT.TEODORO_SUELO_MM (-25), porque con la garra la
mano llega al suelo con Teodoro a -27.8 (19/11), y la ventana no baja de ahi.

Con simular:=true, la placa simulada carga tambien garra.py (con un servo de
mentira), asi que la garra contesta igual que en el brazo.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

from scara_kinematics import garra_traycho as GT

CAMBIO = {'puente_placa': 'puente_garra', 'juntas_gui': 'garra_gui'}


def _traycho_juntas():
    """traycho_juntas.launch.py como modulo (uno nuevo cada vez: lo que se le
    cambia aqui no le llega a nadie mas)."""
    ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'traycho_juntas.launch.py')
    spec = importlib.util.spec_from_file_location('traycho_juntas_launch', ruta)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    tj = _traycho_juntas()
    firmware = os.path.join(get_package_share_directory('scara_kinematics'),
                            'firmware', 'micropython')

    def nodo(**kw):
        kw['executable'] = CAMBIO.get(kw.get('executable'), kw.get('executable'))
        return Node(**kw)

    def proceso(**kw):
        # La placa simulada importa garra.py de la carpeta del firmware instalado
        camino = os.environ.get('PYTHONPATH', '')
        kw['additional_env'] = {'PYTHONPATH': camino + os.pathsep + firmware if camino
                                else firmware}
        return ExecuteProcess(**kw)

    # montar() de traycho_juntas crea los nodos con los Node y ExecuteProcess de
    # su modulo: se le cambian por estos dos, y lo demas queda igual
    tj.Node, tj.ExecuteProcess = nodo, proceso
    acciones = [a for a in tj.generate_launch_description().entities
                if not isinstance(a, OpaqueFunction)]
    # teodoro_min: el suelo de la mano con la garra, no el de traycho_juntas
    acciones = [DeclareLaunchArgument(
        'teodoro_min', default_value=str(GT.TEODORO_SUELO_MM),
        description='Teodoro no por debajo (mm de placa): con la garra, la mano llega '
                    'al suelo a -27.8 y aplasta los cables (GARRA.md)')
                if isinstance(a, DeclareLaunchArgument) and a.name == 'teodoro_min' else a
                for a in acciones]
    return LaunchDescription(acciones + [OpaqueFunction(function=tj.montar)])
