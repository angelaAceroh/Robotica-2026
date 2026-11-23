# Archivos de Arranque (`launch/`)

Aquí se encuentran los lanzadores de ROS 2 (`.launch.py`) que encienden todos los nodos, simuladores y ventanas gráficas con un solo comando.

## Archivos importantes
* **`control_gui.launch.py`** *(Simulación completa)*: Abre Gazebo, los nodos de control cinemático y la interfaz gráfica.
* **`control_placa.launch.py`** *(Brazo real)*: Conecta el control cinemático con la placa ESP32 por USB (acepta `simular:=true` para probar sin robot).
* **`traycho_garra.launch.py`** *(Secuencia y garra)*: Inicia el sistema del brazo real con movimientos por juntas, secuencia multipunto y control de la garra.
* **`tray_gui.launch.py`**: Abre el diseñador visual de trayectorias (rectas, círculos y arcos) en Gazebo.
* **`brazo.launch.py` / `rviz.launch.py` / `gazebo.launch.py`**: Encienden la cadena base de los 6 nodos y la ventana `brazo_hw` con o sin vista 3D.
* **`auditar.launch.py`**: Arranca el sistema junto con el nodo `auditor` para medir tiempos y pérdida de mensajes.

## Instructivo básico
* Ejecuta desde `ros2_ws` (tras hacer `source install/setup.bash`):
  `ros2 launch scara_kinematics control_gui.launch.py`