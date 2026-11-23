# Código Fuente en Python (`scara_kinematics/scara_kinematics/`)

**Aquí está el código en sí** de todos los nodos de ROS 2, algoritmos de cinemática y ventanas gráficas del robot SCARA.

## Archivos importantes por categoría

### 1. Comunicación con la placa ESP32 y motores (6 nodos base)
* **`esp.py`** *(Clave — La Aduana)*: Es el único nodo que abre el cable USB de la ESP32. Lee sensores a 25 Hz, revisa que no haya errores de transmisión y envía consignas a 50 Hz.
* **`alvin.py`**, **`simon.py`**, **`teodoro.py`** *(Claves)*: Controlan cada articulación (hombro, codo y eje vertical Z) y vigilan si algún motor se atasca o llega a un tope.
* **`cc.py`**: Une los datos de las tres juntas, calcula la posición `(X, Y, Z)` de la punta del robot (cinemática directa) y actualiza el modelo 3D.
* **`esp_rec.py`**: Muestra las coordenadas finales en texto legible y permite grabarlas en un archivo CSV.
* **`protocolo.py`**: Define el formato de mensajes serie y la tabla de pines.

### 2. Cinemática, trayectorias y control
* **`model.py`**: Ecuaciones matemáticas de cinemática directa e inversa y área alcanzable del robot.
* **`control_cinematico.py` y `control_nodo.py`** *(Claves)*: Lazo cerrado de control por Jacobiano que corrige errores de posición en tiempo real.
* **`puente_placa.py` y `puente_garra.py`**: Conectan las órdenes de ROS 2 con la placa ESP32 real y el servo de la garra.
* **`trayectoria.py` y `tray.py`**: Planificadores de trayectorias suaves (líneas rectas, círculos y curvas).
* **`ruta.py`, `ruta_nodo.py`, `juntas.py`, `juntas_nodo.py`, `traycho.py`, `garra_traycho.py`**: Ejecución de rutinas multipunto y secuencias automáticas de agarre.
* **`auditor.py`**: Vigila en segundo plano que la comunicación entre nodos sea rápida y sin pérdida de datos.
* **`placa_simulada.py`**: Simula la placa ESP32 en software para hacer pruebas sin hardware.

### 3. Interfaces gráficas (ventanas en Tkinter)
* **`brazo_hw.py`**: Panel básico para fijar el cero, dar/cortar energía y probar motores individualmente.
* **`control_gui.py` y `control_placa_gui.py`** *(Importantes)*: Ventanas principales con mapa 2D del brazo, regla de altura Z, gráfica de error en vivo y parada de emergencia con `Esc`.
* **`tray_gui.py`**, **`traycho_gui.py`**, **`juntas_gui.py`**, **`garra_gui.py`**: Ventanas para diseñar trayectorias, lanzar secuencias multipunto y abrir o cerrar la garra.

## Instructivo básico
* Normalmente estos archivos se lanzan automáticamente desde la carpeta `launch/`.
* Si deseas correr un nodo suelto desde la terminal, pásale su archivo de parámetros:
  `ros2 run scara_kinematics cc --ros-args --params-file install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml`