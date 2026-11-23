# Paquete Principal de Control y Nodos (`scara_kinematics`)

Este es el paquete principal del proyecto. Aquí está toda la programación que comunica el computador con la placa ESP32, calcula los movimientos y muestra las ventanas de control.

## ¿Dónde está cada cosa?
* **`scara_kinematics/`** *(Código principal)*: Todos los programas en Python (nodos de ROS 2, cinemática, control e interfaces gráficas).
* **`launch/`**: Archivos `.launch.py` para encender varios programas juntos con un solo comando.
* **`config/`**: Archivos `.yaml` con las medidas físicas del brazo y la calibración de los motores.
* **`firmware/`**: Programas que van grabados dentro de la placa **ESP32** (`micropython/` y `scara_esp32/`).
* **`rviz/` y `rqt/`**: Configuraciones de pantalla 3D y paneles de gráficas en vivo.
* **`test/`**: Pruebas automáticas para verificar matemáticas y comunicaciones sin encender el robot.

## Archivos importantes en esta raíz
* **`setup.py` y `package.xml`**: Registran todos los comandos ejecutables de ROS 2 y las dependencias del paquete.
* **`NODOS.md`, `CONTROL.md`, `CONTROL_PLACA.md`, `GARRA.md`, `TRAYECTORIAS.md`**: Documentación técnica detallada de cada parte del sistema.

## Instructivo básico
* Para iniciar la ventana principal de control en simulación: `ros2 launch scara_kinematics control_gui.launch.py`
* Para iniciar el sistema conectado al robot real con garra: `ros2 launch scara_kinematics traycho_garra.launch.py`