#!/usr/bin/env python3
"""
display.launch.py

Lanza RViz2 con el robot EnsamblajeURDFPRIMO cargado, junto con:
  - robot_state_publisher  -> publica los TF a partir del URDF
  - joint_state_publisher_gui -> ventana con sliders para mover cada joint
  - rviz2                  -> visualización 3D

USO:
  1. Coloca este archivo dentro de tu paquete ROS 2, en la carpeta launch/
     (ej: ~/ros2_ws/src/EnsamblajeURDFPRIMO/launch/display.launch.py)
  2. Asegúrate de que el .urdf y la carpeta meshes/ estén en tu paquete,
     en la ruta que espera el URDF: package://EnsamblajeURDFPRIMO/meshes/...
  3. Compila el workspace:
       cd ~/ros2_ws
       colcon build --packages-select EnsamblajeURDFPRIMO
       source install/setup.bash
  4. Ejecuta:
       ros2 launch EnsamblajeURDFPRIMO display.launch.py

Si prefieres correrlo suelto sin compilar un paquete completo, puedes
editar la variable URDF_PATH más abajo para apuntar directamente a la
ruta absoluta de tu archivo .urdf y ejecutar este script con:
       python3 display.launch.py
(en ese caso ros2 launch lo ejecutará igual como script python normal
 gracias a generate_launch_description()).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


PACKAGE_NAME = "EnsamblajeURDFPRIMO"
URDF_FILENAME = "EnsamblajeURDFPRIMO.urdf"


def generate_launch_description():

    # --- Ruta al URDF -----------------------------------------------------
    # Intenta resolver la ruta usando el paquete ROS 2 instalado.
    # Si el paquete no está instalado/compilado, cae a una ruta relativa
    # a este mismo archivo (útil para pruebas rápidas sin colcon build).
    try:
        pkg_share = get_package_share_directory(PACKAGE_NAME)
        default_urdf_path = os.path.join(pkg_share, "urdf", URDF_FILENAME)
        if not os.path.exists(default_urdf_path):
            default_urdf_path = os.path.join(pkg_share, URDF_FILENAME)
    except Exception:
        default_urdf_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", URDF_FILENAME
        )

    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=default_urdf_path,
        description="Ruta absoluta al archivo .urdf del robot",
    )

    use_gui_arg = DeclareLaunchArgument(
        name="use_joint_state_gui",
        default_value="true",
        description="Si es true, abre joint_state_publisher_gui con sliders",
    )

    urdf_path = LaunchConfiguration("urdf_path")
    use_gui = LaunchConfiguration("use_joint_state_gui")

    # xacro funciona también con .urdf plano, así que sirve para ambos casos
    robot_description_content = Command(["xacro ", urdf_path])
    robot_description = {"robot_description": robot_description_content}

    # --- Nodo: robot_state_publisher --------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # --- Nodo: joint_state_publisher_gui (con sliders) ----------------------
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_gui),
    )

    # --- Nodo: joint_state_publisher (sin GUI, alternativa) -----------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_gui),
    )

    # --- Nodo: RViz2 ---------------------------------------------------------
    rviz_config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "urdf_config.rviz"
    )
    rviz_args = ["-d", rviz_config_path] if os.path.exists(rviz_config_path) else []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
    )

    return LaunchDescription([
        urdf_path_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
