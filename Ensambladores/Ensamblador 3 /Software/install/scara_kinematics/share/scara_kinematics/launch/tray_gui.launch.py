"""Las trayectorias con su ventana: gz sim + tray + tray_gui.

    ros2 launch scara_kinematics tray_gui.launch.py
    ros2 launch scara_kinematics tray_gui.launch.py rviz:=true
    ros2 launch scara_kinematics tray_gui.launch.py perfil:=trapecio codo:=down

Es trayectoria.launch.py tal cual -se INCLUYE, no se copia- mas la ventana. Todos
sus argumentos valen aqui igual: los de la linea de ordenes son globales y
llegan solos al launch incluido.

        [tray_gui] --scara/tray/orden--> [tray] --Float64--> [gz]
            ^                              |                  |
            +-- estado, plan, /rosout -----+                  |
            +-- /joint_states, /clock <------- puente <-------+

Cerrar la ventana cierra todo lo demas: Gazebo, tray y el puente. Si solo
quieres la ventana contra una simulacion que ya esta corriendo:

    ros2 run scara_kinematics tray_gui --ros-args --params-file \\
        install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# La ventana planifica el borrador con los mismos numeros que tray, asi que le
# llegan los mismos argumentos. Los declara trayectoria.launch.py; el default de
# aqui es el suyo, por si este archivo se lanza de otra forma.
def arg(nombre, defecto, tipo):
    return ParameterValue(LaunchConfiguration(nombre, default=defecto), value_type=tipo)


def generate_launch_description():
    share = get_package_share_directory('scara_kinematics')
    params = os.path.join(share, 'config', 'scara_urdf3.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'trayectoria.launch.py'))),
        Node(
            package='scara_kinematics', executable='tray_gui', name='tray_gui',
            output='screen',
            parameters=[params, {
                'destino': arg('destino', 'gz', str),
                'perfil': arg('perfil', 'quintico', str),
                'codo': arg('codo', 'auto', str),
                'duracion': arg('duracion', '0.0', float),
                'factor_vel': arg('factor_vel', '0.35', float)}],
            on_exit=Shutdown(reason='se ha cerrado la ventana de trayectorias')),
    ])
