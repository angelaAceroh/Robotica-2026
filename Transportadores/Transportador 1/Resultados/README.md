# Manual de usuario — Transportador 1

Cómo instalar, encender, enseñar y ejecutar trayectorias con el robot físico, conectarlo
a la línea con el nodo maestro, usar la parada de emergencia y lanzar la simulación.

> **Regla de oro.** Al abrir el puerto serie, la ESP32 se reinicia y **da por hecho que el
> brazo está en el HOME (J1 = 0°, J2 = 90°, J3 = −90°)**: el brazo en L, con el antebrazo
> horizontal. Antes de abrir el panel, lanzar una trayectoria o arrancar el nodo, **lleva el
> brazo a mano al home**. Si no, todas las posiciones quedan desplazadas.

---

## 1. Lo que hace falta

| | |
|---|---|
| Robot | brazo montado según `documentacion/4_planos/electricos/` (ESP32, 2 × L298N, fuente de 12 V) |
| PC | Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic** (`ros_gz`) |
| Python 3 | `numpy`, `pyserial`, `tkinter`, `matplotlib` |
| Cable | USB de datos entre el PC y la ESP32 |

## 2. Instalación

```bash
# paquetes ROS 2 (simulación, nodo maestro y línea)
cd codigo/ros2_ws
colcon build
source install/setup.bash

# programas del robot físico: se ejecutan desde su carpeta
cd ../pc_robot_fisico
python3 config.py            # imprime la configuración: sirve para comprobar que todo importa bien
```

Si el puerto serie da "permiso denegado": `sudo usermod -a -G dialout $USER` y vuelve a
iniciar sesión.

## 3. Subir el firmware a la ESP32 (solo si se cambia)

El firmware está en `codigo/firmware_esp32/`: `main.py` lo arranca al encender y
`control_vel.py` es el control. Se sube con la herramienta `esp_repl.py`, que además
comprueba la suma del archivo:

```bash
cd codigo/pc_robot_fisico/herramientas
python3 - <<'FIN'
from esp_repl import ESP, suma_local
datos = open('../../firmware_esp32/control_vel.py', 'rb').read()
e = ESP('/dev/ttyUSB0')
e.escribir('control_vel.py', datos)
assert e.suma('control_vel.py') == suma_local(datos), 'no coincide'
e.reiniciar(); e.cerrar()
print('firmware subido')
FIN
```

Al arrancar, la ESP32 manda `# control de VELOCIDAD listo` y deja los motores
**apagados** hasta que llega la primera consigna. Encenderla no mueve nada.

## 4. Encendido seguro

1. Brazo **en el home** (en L), palas recogidas.
2. Enciende la **fuente de 12 V** de los L298N.
3. Conecta el USB al PC. Con `ls /dev/ttyUSB*` debe aparecer el puerto. A veces pasa de
   `ttyUSB0` a `ttyUSB1`; los programas usan el primer `/dev/ttyUSB*` o `/dev/ttyACM*`
   que encuentran.

## 5. Mover el robot y enseñar puntos: la interfaz de sliders

```bash
cd codigo/pc_robot_fisico
python3 panel_juntas.py
```

- **Sliders J1–J3**: objetivo de cada junta, en grados. El panel cierra el lazo de posición
  y respeta los límites medidos.
- **Escala de velocidad**: limita la velocidad máxima de los movimientos.
- **PARAR**: detiene el brazo y deja el objetivo donde está.
- **Palas (J4)**: primero pulsa **"Referenciar palas"**. Hasta que la etiqueta diga
  *referenciado*, el slider de las palas no hace nada; es a propósito. Luego usa
  "Cerrar palas" (70 cuentas), "Abrir palas" (510) o el slider.
- **Guardar punto**: apunta la postura actual (J1–J3 y la apertura de las palas).
- **Guardar archivo**: escribe todos los puntos en `~/Desktop/trayectoria_ensenada.txt`.
  Para usarlos como trayectoria, se copian a `CICLO_ENSENADO` y `APERTURA_POR_PUNTO` en
  `trayectorias.py`.

## 6. Ejecutar una trayectoria

```bash
cd codigo/pc_robot_fisico
python3 mover_robot_vel.py --trayectoria ensenado --perfil trapezoidal --vel 35 --log ~/registro.csv
```

| Opción | Qué hace |
|---|---|
| `--trayectoria ensenado` | la trayectoria enseñada (`CICLO_ENSENADO`), con las palas |
| `--perfil trapezoidal` / `cubico` | perfil de velocidad |
| `--vel 35 --acel 80` | velocidad (°/s) y aceleración (°/s²) máximas |
| `--sin-palas` | no mueve las palas |
| `--solo-plan` | solo calcula y revisa el plan, sin robot |
| `--simulado` | ensayo en seco, sin abrir el puerto |
| `--log archivo.csv` | guarda plan y medidas; se grafica con `herramientas/graficar_log.py` |

**Qué hace, en orden:** lee la postura, añade un tramo de aproximación desde donde esté
el brazo, referencia las palas contra su tope y recorre los puntos. Al llegar a P0, P1 y P6
el plan **se detiene** hasta que el brazo está quieto en el punto, y entonces mueve las
palas. Al final escribe `hecho.`.

**Se para sola** (y escribe `PARADO` y el motivo) si:
- las palas no llegan a ±30 cuentas de su objetivo;
- las palas no se pueden referenciar;
- pasan 0,5 s sin una medida fiable de la ESP32;
- la ESP32 se reinicia a mitad de la trayectoria.

En todos los casos el brazo **se queda quieto donde está**. Revisa la causa, llévalo al home
y vuelve a lanzar.

## 7. Trabajar en la línea con el nodo maestro

Todos los equipos en **la misma red WiFi** y con el mismo dominio de ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
source codigo/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

| Quién | Comando |
|---|---|
| Nuestro robot (físico) | `cd codigo/pc_robot_fisico && python3 nodo_transportador1.py` |
| … y verlo además en la línea de Gazebo | `python3 nodo_transportador1.py --gazebo` |
| Maestro, con los demás robots simulados | `ros2 launch linea_maestro prueba_maestro.launch.py simulados:=transportador2,ensamblador1,ensamblador2,ensamblador3` |
| Maestro, con los 5 robots reales | `ros2 launch linea_maestro prueba_maestro.launch.py simulados:=ninguno` |
| Ver el estado de la línea | `ros2 topic echo /linea/estado` |

El maestro espera a que los 5 robots estén en `listo` y recorre los 9 pasos (ver
`documentacion/2_diagramas_de_secuencia/`). Un robot que está conectado de verdad **no**
debe ir en `simulados`.

Si los equipos no se ven entre sí: comprueba que estén en la misma subred, desactiva el
cortafuegos y prueba con `ros2 multicast receive` en un equipo y `ros2 multicast send` en
otro. Las redes WiFi públicas de la universidad suelen aislar a los clientes; un punto de
acceso del móvil funciona.

## 8. Parada de emergencia

```bash
ros2 topic pub --once /parada std_msgs/msg/Bool "{data: true}"
```

- El **maestro** detiene la línea y deja de mandar órdenes.
- **Nuestro robot** corta la trayectoria en el siguiente ciclo (≤ 50 ms), manda velocidad 0
  y queda en `error`.
- Los demás robots se detienen y quedan en `error`.
- La ESP32 apaga los motores si deja de recibir consignas durante 500 ms. Eso también
  cubre que se caiga el PC o el cable.

Para cortar la potencia de golpe, **apaga la fuente de 12 V**.

**Rearme:** no hay reset por tópico. Revisa la causa, lleva los robots al home y vuelve a
lanzar el maestro y los nodos.

## 9. Simulación

```bash
ros2 launch planta_brazo display.launch.py                        # RViz con sliders
ros2 launch planta_brazo gazebo.launch.py rviz:=true              # Gazebo + RViz + panel
ros2 launch planta_brazo gazebo.launch.py rviz:=true demo:=true   # trayectoria de demostración
ros2 launch planta_brazo trayectoria_perfiles.launch.py           # panel de planeación de trayectorias
ros2 launch planta_brazo control_cinematico.launch.py             # control por Jacobiano inverso
ros2 launch linea_planta linea_gazebo.launch.py maestro:=true     # la línea de 5 robots
```

Lanza **una sola simulación a la vez**: dos Gazebo o dos RViz abiertos se pelean por los
mismos tópicos.

## 10. Solución de problemas

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| "no llega telemetría" | la placa tardó en arrancar | vuelve a lanzar; si persiste, desconecta y conecta el USB |
| El puerto ya no es `/dev/ttyUSB0` | el USB se reconectó | los programas usan el primer `ttyUSB*` que encuentran; si hay varios, fija `PUERTO` en `config.py` |
| La placa no responde tras usar `esp_repl.py` | se quedó en el REPL de MicroPython | `e.reiniciar()`, o desconecta y conecta el USB |
| Posiciones desplazadas unos grados | el brazo no estaba en el home al conectar | llévalo al home y vuelve a abrir el programa |
| "NO se referenciaron" / las palas no se mueven | punto muerto del motor N20 o cremallera trabada | con la fuente apagada, mide continuidad en el motor (8–16 Ω) y mueve un poco las palas a mano |
| Una junta no se mueve | contacto flojo en el driver o el conector | revisa los cables de esa junta; "conectado" no es lo mismo que "hace contacto" |
| Gazebo hace cosas raras o las medidas no cuadran | quedó un Gazebo abierto de antes | `pkill -f "gz sim"` y vuelve a lanzar |
