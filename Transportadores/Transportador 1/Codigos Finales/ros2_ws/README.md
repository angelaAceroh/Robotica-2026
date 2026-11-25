# Espacio de trabajo ROS 2 (Jazzy)

```bash
cd codigo/ros2_ws
colcon build
source install/setup.bash
```

| Paquete | Tipo | Qué es |
|---|---|---|
| [`planta_brazo`](src/planta_brazo) | ament_cmake | modelo URDF/xacro con mallas, launch de RViz y Gazebo, cinemática, Jacobiano, control cinemático y panel de trayectorias |
| [`linea_maestro`](src/linea_maestro) | ament_python | **nodo maestro** de la línea y robot simulado para probarlo |
| [`linea_planta`](src/linea_planta) | ament_python | la línea de 5 robots en Gazebo, con los URDF de los otros equipos adaptados |

Gazebo Harmonic se usa con plugins nativos (`JointPositionController`, `JointStatePublisher`)
y `ros_gz_bridge`, sin `ros2_control`. Cada paquete tiene su propio README.
