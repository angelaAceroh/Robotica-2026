# Proyecto Robot SCARA en ROS 2 (`ros2_ws`)

Este es el espacio de trabajo principal del robot **SCARA de 3 grados de libertad** (hombro **Alvin**, codo **Simón** y eje vertical **Teodoro**) con **garra servocontrolada**, conectado por USB a una placa **ESP32**.

---

## Instructivo básico para el usuario

### 1. Requisitos previos
* **Sistema operativo:** Ubuntu con **ROS 2** (Jazzy / Humble), **Gazebo** y **RViz2**.
* **Librerías de Python:** `pyserial`, `numpy`, `matplotlib`, `tkinter` (`python3-tk`, `python3-serial`).
* **Hardware (para mover el robot real):** Placa ESP32 conectada por cable USB (`/dev/ttyUSB0`) y fuente de **12 V** encendida para los motores.

### 2. Compilar y activar el proyecto
Abre una terminal dentro de `ros2_ws` y ejecuta:
* `colcon build --packages-select scara_kinematics scara_urdf`
* `source install/setup.bash`

*(Nota: Ejecuta `source install/setup.bash` cada vez que abras una terminal nueva).*

### 3. Encender el sistema
* **Usando el script rápido (`abrir.sh`):**
  * `./abrir.sh` — Abre Gazebo + RViz + los 6 nodos + la ventana de control.
  * `./abrir.sh rviz` — Abre solo la vista 3D en RViz (sin simulación física).
  * `./abrir.sh -sh rviz` — Abre en modo simulación sin buscar la placa ESP32 por USB.
  * `./abrir.sh estado` — Muestra qué programas están activos.
  * `./abrir.sh parar` — Cierra todos los procesos y ventanas del robot.
* **Usando comandos directos de ROS 2 (`ros2 launch`):**
  * Simulación con control cinemático y ventana gráfica: `ros2 launch scara_kinematics control_gui.launch.py`
  * Brazo real con interfaz gráfica, rutinas y garra: `ros2 launch scara_kinematics traycho_garra.launch.py`
  * Probar la interfaz del brazo real sin conectarlo: `ros2 launch scara_kinematics control_placa.launch.py simular:=true`

### 4. Operación segura con el robot real
1. Con los motores apagados, lleva manualmente el brazo a la posición **HOME**.
2. En la ventana gráfica pulsa **Fijar cero** para guardar esa posición como referencia.
3. Pulsa **Dar potencia** para energizar los motores.
4. Selecciona un destino `(X, Y, Z)`, ejecuta una trayectoria o usa **Abrir / Cerrar garra**.
5. **Emergencia:** Presiona la tecla **`Esc`** o el botón **PARAR** en cualquier momento para cortar la corriente de inmediato. Al cerrar la ventana también se apagan los motores automáticamente.

---

## Estructura de carpetas

| Carpeta / Archivo | Función |
| :--- | :--- |
| **`abrir.sh`** *(Importante)* | Script todo-en-uno para compilar, iniciar, revisar y apagar el sistema. |
| **`src/`** *(Código fuente)* | **Aquí está todo el código del proyecto**, dividido en `scara_kinematics` y `scara_urdf`. |
| **`build/`** | Archivos temporales generados automáticamente al compilar (`colcon build`). |
| **`install/`** | Programas compilados listos para ejecutarse con ROS 2. |
| **`log/`** | Historial de compilaciones y registros de sesión del robot. |