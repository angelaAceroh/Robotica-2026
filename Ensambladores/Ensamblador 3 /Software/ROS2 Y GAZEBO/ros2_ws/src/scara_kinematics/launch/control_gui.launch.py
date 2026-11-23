"""El control cinematico con su ventana: gz sim + control_nodo + control_gui.

    ros2 launch scara_kinematics control_gui.launch.py
    ros2 launch scara_kinematics control_gui.launch.py tarea:=perforado k:=10
    ros2 launch scara_kinematics control_gui.launch.py tarea:=nada rqt:=true

Es control.launch.py tal cual -se INCLUYE, no se copia- mas la ventana. Todos
sus argumentos valen aqui igual: los de la linea de ordenes son globales y
llegan solos al launch incluido.

        [control_gui] --scara/control/orden--> [control_nodo] --qd--> [gz]
              ^                                     |                  |
              +-- estado, deseado, datos/*, /rosout +                  |
                                  /joint_states, /clock <-- puente <---+

Cerrar la ventana cierra todo lo demas: Gazebo, control_nodo y el puente. Si
solo quieres la ventana contra una simulacion que ya esta corriendo:

    ros2 run scara_kinematics control_gui --ros-args --params-file \\
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


# Los declara control.launch.py; el default de aqui es el suyo, por si este
# archivo se lanza de otra forma.
def arg(nombre, defecto, tipo):
    return ParameterValue(LaunchConfiguration(nombre, default=defecto), value_type=tipo)


def generate_launch_description():
    share = get_package_share_directory('scara_kinematics')
    params = os.path.join(share, 'config', 'scara_urdf3.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'control.launch.py'))),
        Node(
            package='scara_kinematics', executable='control_gui', name='control_gui',
            output='screen',
            parameters=[params, {
                'k': arg('k', '5.0', float),
                'perfil': arg('perfil', 'trapecio', str)}],
            on_exit=Shutdown(reason='se ha cerrado la ventana del control')),
    ])
