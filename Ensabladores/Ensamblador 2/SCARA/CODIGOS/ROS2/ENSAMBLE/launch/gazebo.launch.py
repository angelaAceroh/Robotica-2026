import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'Ensamble'
    pkg_share = get_package_share_directory(pkg_name)
    
    workspace_share = os.path.dirname(pkg_share) 

    set_env_gz = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=workspace_share)
    set_env_ign = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=workspace_share)

    urdf_path = os.path.join(pkg_share, 'urdf', 'Ensamble.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    robot_desc = robot_desc.replace('$(find Ensamble)', pkg_share)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # CORRECCIÓN 1: Se agrega robot_description y use_sim_time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # CORRECCIÓN 2: Puente para sincronizar el tiempo de simulación (/clock) de Gazebo con ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'Ensamble', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        set_env_gz,
        set_env_ign,
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_entity
    ])