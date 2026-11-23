# Manual de Usuario e Instructivo de Operación — Robot SCARA (Ensamblador 2)
**Universidad Militar Nueva Granada — Laboratorio de Robótica**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este manual describe el procedimiento para la puesta en marcha, calibración, operación en modo solitario (Standalone) y operación integrada a la celda de empaque coordinada por el nodo Maestro de la planta.

---

## 1. Requisitos Previos del Sistema

### 1.1. Entorno de Software
- **Sistema Operativo:** Ubuntu 24.04 LTS (o superior).
- **Middleware:** ROS 2 Jazzy Jalisco.
- **Python:** Versión 3.10+ con paquetes requeridos:
  ```bash
  pip install pyserial
  ```
- **Microcontrolador:** ESP32 flasheada con MicroPython oficial (firmware `main.py` cargado en la raíz).

### 1.2. Conexiones Físicas y Hardware
1. **Conexión USB:** Conectar la placa ESP32 a la estación de trabajo mediante cable micro-USB. Verificar que el puerto serial sea reconocido:
   ```bash
   ls -l /dev/ttyUSB0
   ```
   *Nota: Si el puerto aparece con permisos restringidos, habilitar acceso con:*
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # O añadir el usuario al grupo dialout de forma permanente:
   sudo usermod -a -G dialout $USER
   ```
2. **Alimentación de Potencia:**
   - Fuente regulada de 12V DC (capacidad mínima de 5A).
   - Conectar la tierra común (GND) de la fuente de 12V con la tierra de la ESP32 (GND) y los dos módulos L298N.

---

## 2. Procedimiento de Puesta a Cero (Calibración Inicial Home)

Antes de iniciar cualquier secuencia de movimiento cartesiano:
1. Con la fuente de 12V apagada o los motores desenergizados, posicionar manualmente el brazo en la configuración **Home (+Y)**:
   - **Eslabón 1 (Hombro):** Alineado exactamente a $90^\circ$ respecto a la base (apuntando hacia $+Y$).
   - **Eslabón 2 (Codo):** Totalmente alineado en línea recta con el eslabón 1 ($q_2 = 0^\circ$).
   - **Eje Prismático Z:** Carro vertical totalmente retraído en el tope superior mecánico ($Z = 8.0\text{ cm} = 0.080\text{ m}$).
2. Encender la fuente de poder de 12V.
3. Al iniciar la interfaz gráfica de usuario (GUI), presionar el botón **"🎯 Calibrar Cero (+Y)"**.
   - El sistema enviará la señal de sincronización a la ESP32, restableciendo los contadores de encoder a cero absoluto.

---

## 3. Modo de Operación en Solitario (Standalone)

Para realizar pruebas cinemáticas, validación de trayectorias cartesianas y ejecución de rutinas locales:

### Paso 1: Lanzar el Control ROS 2 y Driver Serial
Abrir una terminal en el workspace de ROS 2 y ejecutar:
```bash
cd ~/urdf_ws
./2_lanzar_control_esp32.sh
```
*Este comando inicia el nodo `scara_serial_node` y el nodo cinemático `scara_kinematics_node`.*

### Paso 2: Lanzar la Interfaz Gráfica de Control
En una segunda terminal:
```bash
cd ~/urdf_ws
./3_lanzar_gui.sh
```
*Se desplegará la ventana interactiva `SCARA ROBOT CONTROL & MONITOR`.*

### Paso 3: Funcionalidades de la GUI Standalone
- **Telemetría en Vivo:** Visualización de posiciones articulares ($J_1, J_2$ en radianes y grados, $J_3$ en cm y mm), posición cartesiana $(X, Y, Z)$ y error residual al objetivo.
- **Configuración de Codo:** Selector de codo Izquierdo (`elbow_sign = -1.0`, físico real) o Derecho.
- **Movimiento Cartesiano XYZ:** Ingreso de destino en metros (ej. `0.100`) o centímetros para Z (ej. `12.0`). Si se deja el tiempo en `0.0`, el sistema calcula automáticamente la duración óptima por distancia euclidiana y velocidades máximas.
- **Control de Pinza (Gripper):** Botones manuales de Abrir y Cerrar con deslizadores de potencia PWM (10% a 30%) y duración de pulso en ms.
- **Secuencia Pick & Place:** Configuración de coordenadas de recogida (Pick), entrega (Place), reposo final y ejecución autónoma de los 12 pasos con cierre final de pinza en reposo.

---

## 4. Modo de Operación en Planta con el Nodo Maestro (Ensamblador 2)

El robot Ensamblador 2 opera de forma totalmente coordinada bajo el **Contrato de Comunicación de la Línea de Empacado**.

### Paso 1: Configurar el Dominio de Red
Asegurarse de que todas las computadoras de la celda de manufactura compartan el mismo dominio ROS 2:
```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### Paso 2: Lanzar el Nodo de Planta
Ejecutar el script automatizado:
```bash
cd ~/urdf_ws
./4_lanzar_planta_ensamblador2.sh
```
*Este script inicializa la comunicación serie con la ESP32, el nodo de planta `scara_kinematics_planta_node` y la GUI de supervisión de planta.*

### Paso 3: Contrato de Comunicación Activo
1. **Heartbeat Continuo:** El nodo publica permanentemente en `/ensamblador2/estado` cada 0.5 s (2 Hz):
   - `"listo"`: Disponible para procesar chocolate.
   - `"ocupado"`: En ejecución activa del ciclo Pick & Place.
   - `"error"`: Estado de parada de emergencia o anomalía de hardware.
2. **Recepción de Órdenes:** Al recibir el mensaje `"ensamblar"` en `/ensamblador2/orden`, el robot pasa inmediatamente a `"ocupado"`, realiza la secuencia completa de recogida, colocación y retorno, y vuelve automáticamente a `"listo"`.
3. **Parada de Emergencia:** Al recibir `True` en el tópico `/parada`, todos los actuadores se detienen en menos de 10 ms y el estado se fija en `"error"`.
4. **Rearme:** Mediante el servicio `/ensamblador2/reset_estado` o el botón visual de rearme en la GUI.

---

## 5. Simulación en Gazebo y RViz

Para visualizar el robot en el gemelo digital en tiempo real:
```bash
cd ~/urdf_ws
./1_lanzar_gazebo.sh
```
*Gazebo se sincroniza en modo espejo 1:1 a 20 Hz con los encoders reales del robot físico.*

---

## 6. Guía de Solución de Problemas (Troubleshooting)

| Síntoma | Causa Posible | Solución |
| :--- | :--- | :--- |
| **Error al abrir puerto `/dev/ttyUSB0`** | Permisos insuficientes o puerto ocupado por otra terminal. | Ejecutar `pkill -f scara` para cerrar procesos huérfanos. Verificar permisos con `sudo chmod 666 /dev/ttyUSB0`. |
| **El robot no se mueve tras enviar orden** | Fuente de 12V apagada o tierra desconectada. | Verificar que la fuente de 12V esté entregando corriente y que la tierra (GND) esté unificada entre ESP32, fuentes y drivers L298N. |
| **El codo flexiona hacia el lado contrario** | Convención de codo invertida. | Verificar en la GUI que esté seleccionado **"Codo Izquierda (-1.0)"**. |
| **Error cartesiano grande al calibrar** | Calibración efectuada fuera de posición Home. | Colocar manualmente el robot en $+Y$ recto antes de presionar "Calibrar Cero (+Y)". |
| **El gripper no abre o no cierra** | Cables de inversión en pinza invertidos. | Comprobar que en el segundo L298N el canal esté conectado a los GPIO 14 (Apertura) y GPIO 12 (Cierre). |
