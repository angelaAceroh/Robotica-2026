# Firmware MicroPython para Microcontrolador ESP32
**Robot SCARA RRP — Ensamblador 2**  
**Universidad Militar Nueva Granada — Laboratorio de Robótica 2026**

Este directorio contiene el firmware embebido en MicroPython (`main.py`) encargado del control en tiempo real de bajo nivel del manipulador SCARA.

---

## 1. Arquitectura y Módulos del Firmware

El firmware ejecuta las siguientes tareas en tiempo real determinista:
1. **Lectura de Encoders por Interrupciones de Hardware:**
   - Monitoreo en cuadratura de los canales A y B de los 3 encoders magnéticos JGA25-370 mediante rutinas de servicio de interrupción (ISR) asociadas al flanco de subida (`Pin.IRQ_RISING`).
   - Conteo atómico bidireccional sin pérdida de pulsos en `encoder_count[0..2]`.
2. **Lazo de Control Periódico a 100 Hz (10 ms):**
   - Ejecutado mediante el temporizador de hardware `Timer(0)` de la ESP32.
   - Cálculo de la velocidad articular real medida $\omega_{\text{med}} = \frac{\Delta \text{ticks}}{\Delta t} \cdot \frac{2\pi}{990}$.
   - **Feedforward Directo:** Inyección instantánea de ciclo útil proporcional a la velocidad angular comandada $\omega_{\text{ref}}$, sumando el umbral de fricción estática `MIN_PWM`.
   - **Feedback PI:** Corrección del error de velocidad $e = \omega_{\text{ref}} - \omega_{\text{med}}$ con ganancias $K_p$ y $K_i$ y anti-windup integral.
3. **Control Bipolar del Gripper N20:**
   - Salida PWM independiente en GPIO 14 (Apertura) y GPIO 12 (Cierre) a 5000 Hz.
   - Límite de ciclo útil a máximo 30% para asegurar operación por debajo de 3.0V DC.
   - Temporizador de desconexión automática por software para evitar calentamiento de la bobina al hacer tope.
4. **Watchdog de Seguridad en Comunicaciones:**
   - Si no se reciben comandos de velocidad válidos desde ROS 2 en un intervalo mayor a 400 ms, los motores se detienen automáticamente para evitar movimientos descontrolados por desconexión.
5. **Parser Serie No Bloqueante (`uselect.poll`):**
   - Lectura asíncrona de caracteres en `sys.stdin` sin bloquear la ejecución del lazo de control periódico.

---

## 2. Mapa de Asignación de Pines GPIO

| Eje / Mecanismo | Pin PWM | Pin IN1 / Canal 1 | Pin IN2 / Canal 2 | Pin Enc A | Pin Enc B |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Motor 1 (Hombro J1)** | GPIO 27 (1000 Hz) | GPIO 25 | GPIO 26 | GPIO 18 (IRQ) | GPIO 19 |
| **Motor 2 (Codo J2)** | GPIO 23 (1000 Hz) | GPIO 33 | GPIO 32 | GPIO 21 (IRQ) | GPIO 22 |
| **Motor 3 (Prismático J3)**| GPIO 13 (1000 Hz) | GPIO 16 | GPIO 17 | GPIO 34 (IRQ) | GPIO 35 |
| **Motor 4 (Gripper N20)** | N/A | GPIO 14 (PWM Open) | GPIO 12 (PWM Close)| N/A | N/A |

---

## 3. Protocolo de Comunicación Serial (115200 Baudios)

### Comandos Recibidos desde ROS 2:
- **`V,w1,w2,w3\n`**: Consigna de velocidad en radianes/segundo para los motores 1, 2 y 3.
  - Ejemplo: `V,1.5708,-0.5000,0.0000`
- **`G,accion,pwm,dur\n`**: Comando de accionamiento de pinza.
  - `accion`: `OPEN` o `CLOSE`.
  - `pwm`: Potencia porcentual (ej. 25 para abrir, 18 para cerrar).
  - `dur`: Duración del pulso en milisegundos (ej. 400 ms).
  - Ejemplo: `G,OPEN,25,400`
- **`Z\n`**: Puesta a cero absoluta de los contadores de encoder en posición Home.
- **`S\n`**: Parada inmediata de emergencia de todos los actuadores.

### Telemetría Emitida hacia ROS 2 (a 50 Hz):
- **`P,count1,count2,count3\n`**: Conteo bruto acumulado de ticks de los tres encoders de posición.
  - Ejemplo: `P,1250,-840,4320`

---

## 4. Instrucciones para Flashear el Firmware

1. Conectar la ESP32 por USB.
2. Usar `mpremote`, `ampy` o `Thonny`:
   ```bash
   pip install mpremote
   mpremote connect /dev/ttyUSB0 cp main.py :main.py
   mpremote connect /dev/ttyUSB0 reset
   ```
