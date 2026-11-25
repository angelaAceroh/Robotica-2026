# `linea_maestro` — Nodo maestro de la línea de empacado

Paquete ROS 2 (Jazzy, `ament_python`) que **coordina a los 5 robots** de la línea:
`transportador1`, `transportador2`, `ensamblador1`, `ensamblador2` y `ensamblador3`.

El maestro **no mueve ningún robot**: no tiene cinemática ni sabe cómo es ningún brazo.
Decide **quién se mueve y cuándo**: manda a cada robot su orden por texto, espera su
confirmación y no lanza el paso siguiente hasta que el anterior terminó. Así dos robots
nunca entran a la vez en la misma posición. Si algo falla, detiene toda la línea.

<p align="center"><img src="../../../../documentacion/3_diagramas_de_flujo/flujo_maestro_linea_completa.png" width="820" alt="Flujo del nodo maestro y la línea completa"></p>

---

## Contenido del paquete

```
linea_maestro/
├── linea_maestro/
│   ├── maestro.py             el coordinador
│   └── robot_simulado.py      se hace pasar por cualquiera de los 5 robots
├── launch/
│   └── prueba_maestro.launch.py
├── package.xml · setup.py · setup.cfg
└── README.md
```

---

## Comunicación

| Tópico | Tipo | Publica | Contenido |
|---|---|---|---|
| `/<robot>/orden` | `std_msgs/String` | el maestro | la orden a ejecutar |
| `/<robot>/estado` | `std_msgs/String` | cada robot | `listo`, `ocupado` o `error` |
| `/parada` | `std_msgs/Bool` | el maestro (o un operador) | `true` = todos se detienen |
| `/linea/estado` | `std_msgs/String` | el maestro | paso actual y avisos, para monitorear |

Se usa `ROS_DOMAIN_ID=42`.

| Robot | Órdenes |
|---|---|
| `transportador1` | `P0_a_P1` · `P1_a_P2` · `home` |
| `transportador2` | `P2_a_P3` · `P3_a_P4` · `home` |
| `ensamblador1`, `ensamblador2`, `ensamblador3` | `ensamblar` |

**Lo que el maestro espera de cada robot:**
1. que publique su estado cada 0,5 s;
2. que al recibir una orden publique `ocupado` de inmediato, la ejecute y publique `listo`;
3. que ignore órdenes mientras está ocupado;
4. que con `/parada = true` se detenga y quede en `error`.

---

## La secuencia

| # | Robot | Orden | Qué pasa |
|---|---|---|---|
| 1 | transportador1 | `P0_a_P1` | recoge la caja vacía en la entrada y la lleva a la posición 1 |
| 2 | ensamblador1 | `ensamblar` | trabaja en la posición 1 |
| 3 | transportador1 | `P1_a_P2` | lleva la caja de la posición 1 a la 2 |
| 4 | transportador1 | `home` | se retira y libera la zona de la posición 2 |
| 5 | ensamblador2 | `ensamblar` | trabaja en la posición 2 |
| 6 | transportador2 | `P2_a_P3` | lleva la caja de la posición 2 a la 3 |
| 7 | ensamblador3 | `ensamblar` | trabaja en la posición 3 |
| 8 | transportador2 | `P3_a_P4` | lleva la caja a la salida (posición 4) |
| 9 | transportador2 | `home` | se retira |

El paso 4 hace falta porque el transportador 1 tiene que salirse antes de que el
ensamblador 2 entre en la posición 2.

<p align="center"><img src="../../../../documentacion/2_diagramas_de_secuencia/secuencia_ciclo_linea.png" width="720" alt="Secuencia de un ciclo de la línea"></p>

### Cómo avanza cada paso (`maestro.py`, cada 0,1 s)

1. **Arranque:** espera a que los 5 robots publiquen `listo`.
2. **Orden:** publica la orden en `/<robot>/orden` y el paso en `/linea/estado`.
3. **Confirmación:** espera `ocupado`. Si no llega en 1 s, reenvía la orden.
4. **Fin del paso:** cuando el robot vuelve a `listo`, pasa al siguiente.
5. **Vigilancia continua:** si un robot publica `error`, si un robot lleva más de 3 s sin
   publicar o si un paso dura más de 60 s, **detiene la línea**: pasa a `DETENIDO`,
   publica `/parada = true` y deja de mandar órdenes. Hace lo mismo si alguien publica
   `/parada`.
6. **Final:** tras el paso 9 termina ("La caja salió por la posición 4"), o empieza otro
   ciclo si `repetir` es `true`.

Diagrama del bucle interno: [`flujo_maestro.png`](../../../../documentacion/3_diagramas_de_flujo/flujo_maestro.png) ·
parada de emergencia: [`flujo_parada.png`](../../../../documentacion/3_diagramas_de_flujo/flujo_parada.png).

### Parámetros

| Parámetro | Defecto | Qué hace |
|---|---|---|
| `repetir` | `false` | `true` repite el ciclo sin parar |
| `timeout_paso` | `60.0` s | tiempo máximo para que un robot termine un paso |
| `timeout_vivo` | `3.0` s | sin recibir su estado, el robot se da por caído |
| `reenvio` | `1.0` s | cada cuánto reenvía una orden no confirmada |

---

## Uso

```bash
cd "codigos Finales/ros2_ws"
colcon build --packages-select linea_maestro
source install/setup.bash
export ROS_DOMAIN_ID=42
```

| Caso | Comando |
|---|---|
| Maestro + los 5 robots simulados | `ros2 launch linea_maestro prueba_maestro.launch.py` |
| Con nuestro brazo real (los otros 4 simulados) | `ros2 launch linea_maestro prueba_maestro.launch.py simulados:=transportador2,ensamblador1,ensamblador2,ensamblador3` |
| Repetir el ciclo sin parar | añadir `repetir:=true` |
| La línea completa en Gazebo | `ros2 launch linea_planta linea_gazebo.launch.py maestro:=true` |
| Seguir la línea | `ros2 topic echo /linea/estado` |
| **Parada de emergencia** | `ros2 topic pub --once /parada std_msgs/msg/Bool "{data: true}"` |

`robot_simulado.py` responde como un robot real, sin moverse. Tiene dos parámetros:
`nombre` (cuál de los 5 finge ser) y `duracion` (cuántos segundos tarda cada tarea; el
launch usa 3 s para los transportadores y 2 s para los ensambladores). Un robot conectado
de verdad no debe ir en `simulados`.

Con el brazo real, el transportador 1 lo maneja `pc_robot_fisico/nodo_transportador1.py`:
- `P0_a_P1` ejecuta la trayectoria enseñada con la caja;
- `home` lleva el brazo al home;
- con `/parada` corta la trayectoria y queda en `error`.

Secuencia: [`secuencia_orden_transportador1.png`](../../../../documentacion/2_diagramas_de_secuencia/secuencia_orden_transportador1.png).

---

## Pruebas y resultados

| Prueba | Cómo se hace | Resultado |
|---|---|---|
| Ciclo normal | `ros2 launch linea_maestro prueba_maestro.launch.py` | los 9 pasos en orden, ciclo completo en **25,0 s** |
| Robot caído | con la línea en marcha, `pkill -f sim_ensamblador2` | detectado a los **3,0 s**; la línea se detiene y los otros 4 robots obedecen la parada |
| Parada de emergencia | `ros2 topic pub --once /parada std_msgs/msg/Bool "{data: true}"` | los 5 robots se detienen **en el mismo milisegundo** |
| Línea en Gazebo | `linea_gazebo.launch.py maestro:=true` | los 9 pasos con los 5 robots en su sitio del plano |
| Nodo del transportador 1 | `nodo_transportador1.py --simulado` + los otros 4 simulados | ciclo completo; `/parada` lo detiene a mitad de la trayectoria |
