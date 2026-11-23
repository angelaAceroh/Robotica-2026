import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Puerto serie de la ESP32'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Velocidad de comunicación serie'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Iniciar interfaz gráfica visualizadora de la Planta'
    )

    serial_node = Node(
        package='scara_control',
        executable='scara_serial_node',
        name='scara_serial_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'sign_m1': -1.0,
            'sign_m2': -1.0,
            'sign_m3': -1.0,
            'ticks_per_meter_j3': 7878.0,
            'z_home_offset': 0.080
        }]
    )

    kinematics_planta_node = Node(
        package='scara_control',
        executable='scara_kinematics_planta_node',
        name='scara_kinematics_planta_node',
        output='screen',
        parameters=[{
            'dir_m1': -1.0,
            'dir_m2': -1.0,
            'dir_m3': -1.0,
            'elbow_sign': -1.0
        }]
    )

    gui_planta_node = Node(
        package='scara_control',
        executable='scara_gui_planta_node',
        name='scara_gui_planta_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        gui_arg,
        serial_node,
        kinematics_planta_node,
        gui_planta_node
    ])
