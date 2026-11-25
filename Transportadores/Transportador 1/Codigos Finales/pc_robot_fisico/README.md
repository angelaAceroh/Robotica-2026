# Programas del PC para el robot físico

| Archivo | Qué hace |
|---|---|
| `config.py` | **configuración única**: geometría, reducciones, encoders, pares, límites, home y puerto |
| `cinematica.py`, `jacobiano.py` | cinemática directa e inversa y Jacobiano |
| `perfiles.py` | perfiles de velocidad trapezoidal y cúbico |
| `trayectorias.py` | trayectorias, incluida la **enseñada** (`CICLO_ENSENADO`) con la apertura de las palas por punto |
| `enlace_esp.py` | enlace serie con la ESP32: envía consignas y filtra la telemetría (descarta saltos y detecta reinicios) |
| `mover_robot_vel.py` | **ejecuta una trayectoria**: plan + lazo de posición cada 50 ms + palas + paradas de seguridad |
| `panel_juntas.py` | **interfaz de usuario** (Tk): sliders por junta, palas, y enseñar y guardar puntos |
| `nodo_transportador1.py` | **nodo ROS 2** del robot: obedece al maestro (`/transportador1/orden`, `/estado`, `/parada`) |
| `espejo_ros.py` | publica la postura medida para que Gazebo copie al robot real |
| `estado_palas.py` | guarda dónde quedaron las palas para referenciarlas de forma segura |
| `herramientas/` | subir el firmware, graficar registros, monitor del encoder y cálculo de pares |

Uso: ver el [manual de usuario](../../manual_de_usuario/README.md). Lógica:
[`flujo_robot`](../../documentacion/3_diagramas_de_flujo/flujo_robot.png) y
[`flujo_ejecucion`](../../documentacion/3_diagramas_de_flujo/flujo_ejecucion.png).
