# Lanzadores del Modelo 3D (`scara_urdf/launch/`)

Archivos de arranque rápidos para probar el modelo 3D en pantalla sin encender los nodos de control ni la placa ESP32.

## Archivos importantes
* **`display.launch.py`**: Abre RViz2 junto con una ventana de deslizadores para mover cada articulación manualmente.
* **`gazebo.launch.py`**: Llama al lanzador de simulación física en Gazebo del paquete `scara_kinematics`.

## Instructivo básico
* Ejecutar: `ros2 launch scara_urdf display.launch.py`