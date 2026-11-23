# Configuraciones Visuales de RViz2 (`scara_kinematics/rviz/`)

Archivos que guardan la posición de la cámara, colores y elementos visibles en el visualizador 3D **RViz2**.

## Archivos importantes
* **`scara.rviz`**: Vista estándar del modelo 3D del robot con el rastro verde de la trayectoria de la punta.
* **`control.rviz`**: Vista preparada para comparar el camino deseado frente al camino real ejecutado por el controlador.

## Instructivo básico
* No hace falta abrirlos manualmente; se cargan solos al ejecutar `ros2 launch scara_kinematics rviz.launch.py`.