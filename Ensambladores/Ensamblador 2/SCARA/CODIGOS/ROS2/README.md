# Paquetes y Nodos de ROS 2 Jazzy — Ensamblador 2
**Universidad Militar Nueva Granada — Laboratorio de Robótica 2026**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este directorio contiene los paquetes desarrollados bajo **ROS 2 Jazzy Jalisco** para el control, simulación, visualización e integración en planta del robot SCARA.

---

## 1. Paquete `scara_control`

Es el paquete central de control cinemático, supervisión y comunicación serial.

### Nodos Principales:
1. **`scara_serial_node`:**
   - Lee el puerto serial `/dev/ttyUSB0` a 115200 baudios.
   - Publica el estado articular del robot en el tópico estándar `/joint_states` (`sensor_msgs/msg/JointState`) convirtiendo ticks a radianes y metros.
   - Se suscribe a `/cmd_joint_vel` y envía los paquetes `V,w1,w2,w3\n` a la ESP32 con `flush()` inmediato.
   - Se suscribe a `/cmd_gripper` para el envío de órdenes de apertura/cierre.
   - Provee el servicio `/calibrate_zero` (`std_srvs/srv/Trigger`) para reiniciar contadores a cero en $+Y$.
2. **`scara_kinematics_node` (Modo Standalone):**
   - Ejecuta cinemática directa (FK) e inversa (IK) con selección de codo.
   - Planifica trayectorias cartesianas en línea recta mediante curvas suaves en S (Minimum Jerk).
   - Calcula automáticamente el tiempo de recorrido óptimo en función de la distancia euclidiana y velocidades máximas.
   - Ejecuta el lazo de control con velocidad Feedforward + Feedback articular PI con anti-windup.
   - Ofrece el servicio `/run_pick_and_place` para ejecutar la secuencia autónoma de 12 pasos con cierre final de pinza en reposo.
   - Publica la posición articular a Gazebo en `/position_controller/commands` a 20 Hz (espejo 1:1).
3. **`scara_kinematics_planta_node` (Modo Planta / Contrato Maestro):**
   - Implementa todas las funciones cinemáticas y añade el **Contrato de Comunicación de la Línea de Empacado**.
   - Publica permanentemente en `/ensamblador2/estado` (`"listo"`, `"ocupado"`, `"error"`) a 2 Hz ininterrumpidamente.
   - Escucha `/ensamblador2/orden`. Al recibir `"ensamblar"`, pasa a `"ocupado"`, ejecuta el ciclo de empaque y retorna a `"listo"`. Es inmune a órdenes repetidas mientras está ocupado.
   - Escucha `/parada` (`std_msgs/msg/Bool`). Si es `True`, apaga los motores y pasa a `"error"`.
   - Provee el servicio `/ensamblador2/reset_estado` para rearme.
4. **`scara_gui_node` y `scara_gui_planta_node`:**
   - Interfaces gráficas interactivas desarrolladas en Tkinter.
   - Telemetría en tiempo real de articulaciones, coordenadas cartesianas y error residual.
   - Controles manuales y automáticos, selectores de codo, ajuste de pinza y monitoreo de la planta.

---

## 2. Tópicos y Servicios del Sistema

### Tópicos Publicados y Suscritos:
| Tópico | Tipo de Mensaje | Dirección | Descripción |
| :--- | :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Publicado por `scara_serial_node` | Ángulos y elongación real medida ($q_1, q_2, q_3$) |
| `/cmd_joint_vel` | `std_msgs/msg/Float64MultiArray`| Publicado por cinemática $\rightarrow$ Suscrito por serial | Consignas de velocidad de motores ($\omega_1, \omega_2, \omega_3$) |
| `/cmd_cartesian_target` | `std_msgs/msg/Float64MultiArray` | Publicado por GUI $\rightarrow$ Suscrito por cinemática | Destino cartesiano $[X, Y, Z, T]$ |
| `/cmd_gripper` | `std_msgs/msg/String` | Publicado por GUI/cinemática $\rightarrow$ Suscrito por serial | Orden de pinza (`open` / `close`, potencia, duración) |
| `/ensamblador2/estado` | `std_msgs/msg/String` | Publicado por cinemática planta $\rightarrow$ Maestro | Heartbeat a 2 Hz (`listo`, `ocupado`, `error`) |
| `/ensamblador2/orden` | `std_msgs/msg/String` | Publicado por Maestro $\rightarrow$ Suscrito por planta | Orden de trabajo (`ensamblar`) |
| `/parada` | `std_msgs/msg/Bool` | Publicado por Maestro $\rightarrow$ Suscrito por planta | Parada de emergencia de la línea |
| `/position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Publicado por cinemática $\rightarrow$ Gazebo | Espejo de posición articular a 20 Hz |

### Servicios ROS 2:
| Servicio | Tipo | Proveedor | Descripción |
| :--- | :--- | :--- | :--- |
| `/calibrate_zero` | `std_srvs/srv/Trigger` | `scara_serial_node` | Puesta a cero en configuración $+Y$ (Home) |
| `/run_pick_and_place` | `std_srvs/srv/Trigger` | `scara_kinematics_node` | Inicio de rutina Pick & Place en solitario |
| `/ensamblador2/reset_estado` | `std_srvs/srv/Trigger` | `scara_kinematics_planta_node`| Rearme manual de estado de planta a `"listo"` |
| `/emergency_stop` | `std_srvs/srv/Trigger` | `scara_kinematics_node` | Parada inmediata de trayectoria |

---

## 3. Paquete `ENSAMBLE` (Simulación y Modelado)

Contiene la descripción cinemática y dinámica para simulación en RViz y Gazebo:
- **`urdf/Ensamble.urdf`:** Archivo URDF con definición de eslabones (`base_link`, `J1`, `J2`, `J3`, gripper), articulaciones rotacionales continuas y prismáticas, límites de esfuerzo y mallas visuales/colisión.
- **`meshes/`:** Mallas tridimensionales STL de alta precisión generadas desde SolidWorks.
- **`launch/display.launch.py`:** Lanzador de visualización en RViz 2 con `robot_state_publisher` y `joint_state_publisher_gui`.
- **`launch/gazebo.launch.py`:** Lanzador del simulador Gazebo con controladores articulares `ros2_control`.

---

## 4. Scripts de Lanzamiento Rápido

- **`1_lanzar_gazebo.sh`:** Inicializa la simulación física en Gazebo.
- **`2_lanzar_control_esp32.sh`:** Lanza el driver serial y el nodo cinemático local.
- **`3_lanzar_gui.sh`:** Lanza la interfaz gráfica de supervisión y control local.
- **`4_lanzar_planta_ensamblador2.sh`:** Lanza el sistema completo de planta con `ROS_DOMAIN_ID=42`.
- **`simulador_maestro.py`:** Herramienta interactiva de terminal para simular el comportamiento del nodo Maestro de la celda de empaque.
