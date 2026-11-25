# planta_brazo — simulación del robot antropomórfico 3R (UMNG)

Paquete ROS 2 (Jazzy) con la descripción URDF del robot **PLANTA BRAZO**,
reconstruido a partir del CAD del laboratorio, y dos simulaciones que comparten
exactamente el mismo URDF, los mismos scripts y la misma trayectoria:

- **RViz2** — simulación *cinemática*: dibuja la pose que se le manda.
- **Gazebo** (gz sim Harmonic) — simulación *dinámica*: gravedad, inercias y un
  PID de posición por articulación; lo que se ve es lo que el control consigue.

## Origen de los datos

| Fuente | Uso |
|---|---|
| `PLANTA BRAZO - 3/PLANTA BRAZO - 22/STL/` | mallas de BASE, TAPA, BRAZO1, Brazo2, Brazo3 y las 28 piezas del gripper |
| `ENSAMBLAJEBRAZOFINALSLDASM.pdf` (explosionado) | orden de montaje y lista de piezas (BASE→TAPA→BRAZO1→Brazo2→Brazo33→gripper) |
| `CinematicaDirecta.pdf` / `Simulacion y Cinematica Directa.pdf` | convención DH del brazo antropomórfico 3R |

Los STL de SolidWorks salieron **cada uno con su propio origen local** (la
esquina de su caja envolvente) y en **milímetros**, así que no se podían
ensamblar directamente. Los ejes de las articulaciones se localizaron midiendo
los **alojamientos de rodamiento 628-2RS (Ø24 mm)** que aparecen en cada pieza:

```
BRAZO1  eje J1 (Y local)  en (x= 40.50, z= 39.67)   agujero Ø11.8 pasante
        eje J2 (X local)  en (y=214.65, z= 97.82)   Ø24 + eje Ø9
Brazo2  aloj. J2 (Z local) en (x=164.56, y=20)      Ø24 + eje Ø9
        aloj. J3 (Z local) en (x= 18.20, y=20)      Ø24 + hexágono
Brazo3  aloj. J3 (Z local) en (x=162.85, y=20)      Ø24 + hexágono
BASE    eje J1 en (x=100, z=100), cara superior y=120
TAPA    eje J1 en (x= 80, z= 80), apoya en la base (y=100 ↔ z=120)
```

De ahí salen los parámetros geométricos (`config/parametros.yaml`):

| Parámetro | Valor | Significado |
|---|---|---|
| `d1` | 342.65 mm | suelo → eje del hombro (J2) |
| `a1` |  58.15 mm | offset horizontal J1 → J2 (cuello de cisne de BRAZO1) |
| `L2` | 146.36 mm | eje J2 → eje J3 (Brazo2) |
| `L3` | 162.85 mm | eje J3 → brida de muñeca (Brazo3) |
| `Lg` | 123.00 mm | brida → punta de los dedos del gripper (TCP real) |

Alcance máximo desde el eje de giro: **367 mm** (490 mm con gripper).

## Cinemática directa

Misma DH del PDF, más el offset real `a1` del CAD:

```
i | theta | d  | a     | alpha
--+-------+----+-------+-------
1 |  q1   | d1 | a1    | +pi/2
2 |  q2   | 0  | L2    |   0
3 |  q3   | 0  | L3+Lg |   0     <- Lg solo para el TCP (dedos del gripper)

r = a1 + L2·cos(q2) + Le·cos(q2+q3)
x = r·cos(q1)     y = r·sin(q1)
z = d1 + L2·sin(q2) + Le·sin(q2+q3)
```

Con `Le = L3` se obtiene la **brida** (fin del Brazo3); con `Le = L3 + Lg`, la
**punta de los dedos del gripper**, que es lo que dibuja la traza naranja de
RViz. Con `a1 = 0` y `Lg = 0` se recupera exactamente el resultado del PDF.
En `q = (0,0,0)` el brazo queda horizontal sobre +X; `q2 > 0` lo levanta.

Es **unica**: una terna de angulos da exactamente un punto.

## Cinemática inversa

El problema contrario —dado el punto, qué ángulos hay que poner— se resuelve
en **forma cerrada** (no iterativa), separando el giro de la base del brazo
plano de dos eslabones:

```
1) giro de base      q1 = atan2(y, x)            (o q1 + pi: "hombro atrás")

2) problema plano, hombro en el origen:
       u = ±sqrt(x² + y²) − a1      (+ hombro al frente, − hombro atrás)
       v = z − d1

3) ley del coseno en el codo:
       D = (u² + v² − L2² − Le²) / (2·L2·Le)
       q3 = −acos(D)  codo ARRIBA      q3 = +acos(D)  codo ABAJO

4) hombro, restando el aporte del antebrazo:
       q2 = atan2(v, u) − atan2(Le·sin(q3), L2 + Le·cos(q3))
```

Es decir, **la inversa no es "la directa al revés"**: un mismo punto tiene
hasta **cuatro** soluciones (2 de base × 2 de codo), todas igual de válidas
geométricamente. Hay que *elegir* una, y el criterio es parte del problema.
`cinematica.py` devuelve todas, marca cuáles caben en los límites
articulares, y `resolver()` escoge según la postura pedida
(`codo_arriba`, `codo_abajo`) o la más cercana a la posición actual
(`cercana`, que es el criterio de continuidad que usan los robots reales).

Casos que el módulo marca explícitamente:

| Situación | Qué pasa | Cómo se detecta |
|---|---|---|
| punto fuera del alcance | no hay solución | `|D| > 1` |
| brazo estirado o plegado del todo | las dos soluciones de codo se funden | `|D| = 1`, marca `singular` |
| punto sobre el eje de la base | `q1` es indeterminado (infinitas soluciones) | `x = y = 0`, conserva el `q1` actual |
| solución geométrica pero inalcanzable | existe, pero topa en un límite | lista `fuera` con las juntas que se salen |

Comprobación de cierre CD → CI → CD con 20 000 posturas aleatorias válidas:
**0 fallos, error máximo por debajo de la micra** (`python3 scripts/cinematica.py`).

## Uso

```bash
cd ~/ros2_ws
colcon build --packages-select planta_brazo
source install/setup.bash

# 1) RViz + sliders (reemplazan a joint_state_publisher_gui, que no está instalado)
ros2 launch planta_brazo display.launch.py

# 2) RViz + trayectoria automática de demostración
ros2 launch planta_brazo display.launch.py demo:=true

# 3) sin RViz (solo TF + cinemática directa por consola)
ros2 launch planta_brazo display.launch.py rviz:=false

# 4) Gazebo con física + el mismo panel (ahora los sliders son consignas)
ros2 launch planta_brazo gazebo.launch.py

# 6) Gazebo recorriendo los 8 puntos de prueba y sacando la tabla comparativa
#    directa vs inversa (deja ~/comparacion_cinematicas.md y .csv)
ros2 launch planta_brazo gazebo.launch.py comparar:=true

# 5) Gazebo + RViz a la vez (RViz dibuja la traza del TCP)
ros2 launch planta_brazo gazebo.launch.py rviz:=true
```

> Los dos launch publican `/joint_states`: **no los abras a la vez**, se pelean
> por el tópico y el robot da tirones.

Tópicos publicados:

- `/joint_states` — `sensor_msgs/JointState` (J1, J2, J3)
- `/robot_description`, `/tf` — para RViz
- `/brida_pose` — `geometry_msgs/PoseStamped` de la brida (fin del Brazo3)
- `/tcp_pose` — `geometry_msgs/PoseStamped` de la punta de los dedos del gripper
- `/tcp_traza` — `visualization_msgs/Marker` con el rastro del TCP

Y escucha `/traza/limpiar` (`std_msgs/Empty`) para borrar el rastro.

Comprobación rápida de que el URDF y la cinemática directa coinciden:

```bash
ros2 run tf2_ros tf2_echo world brida    # == /brida_pose
ros2 run tf2_ros tf2_echo world tcp      # == /tcp_pose
```

## Nodos

| Ejecutable | Qué hace |
|---|---|
| `gui_articulaciones.py` | panel Tk con las **dos** cinemáticas: 3 deslizadores (directa) + casillas X/Y/Z con **Ir al punto** (inversa); muestra brida y TCP en vivo |
| `cinematica.py` | módulo puro con la **directa y la inversa**; `resolver()` elige postura y avisa de puntos fuera de alcance, singulares o fuera de límites |
| `cinematica_directa.py` | DH del PDF; publica `/brida_pose`, `/tcp_pose` y `/tcp_traza` |
| `comparacion_cinematicas.py` | recorre los puntos de prueba por las dos vías y saca la tabla comparativa en `.md` y `.csv` |
| `puntos_prueba.py` | los 8 puntos con los que se comparan las dos cinemáticas |
| `trayectoria_demo.py` | mismo recorrido, como nodo suelto (para usar sin GUI) |
| `trayectorias.py` | módulo compartido: los 10 puntos y el perfil quíntico |
| `enlace_juntas.py` | capa común: en RViz publica `/joint_states`, en Gazebo manda consignas `cmd_pos` y lee la medida real |
| `remuestreo_juntas.py` | baja `/joint_states` de los 1000 Hz de la física a 60 Hz |

El botón **Trayectoria** del panel ejecuta el recorrido desde la propia GUI,
así que no hay dos nodos publicando `/joint_states` a la vez. El deslizador
*Tiempo por tramo* ajusta la velocidad (0.5–6 s por tramo).

## Notas de implementación

- Las mallas se **reescribieron en metros** (STL binario, normales recalculadas)
  en lugar de usar el atributo `scale` del URDF: así no dependen de que el
  visualizador lo respete, y sirven igual para Gazebo.
- Las 28 piezas del gripper se fundieron en una sola malla `gripper.STL`
  (61 224 triángulos), porque comparten el mismo sistema de coordenadas del
  ensamblaje. El gripper va como eslabón fijo: si más adelante se quiere
  articular, hay que separar `Gear Link` / `link-*` / `Gripper - copia-*`.
- Los desfases laterales (`off_b2`, `off_b3`) reproducen el montaje real
  alternado de los eslabones a lado y lado de cada horquilla; la cinemática
  no los usa (los ejes siguen siendo coplanares).
- Las **inercias son estimaciones** de caja equivalente, no vienen del CAD.
  Sirven para RViz y para un arranque en Gazebo, pero si se va a hacer control
  dinámico conviene sacarlas de SolidWorks (Herramientas → Propiedades físicas).
- Límites articulares puestos a mano (J1 ±180°, J2 ±90°, J3 ±149°); ajústalos
  en `urdf/planta_brazo.urdf.xacro` cuando se midan los topes reales.


## Simulación en Gazebo

Gazebo Harmonic (`gz sim` 8.11) con los plugins **nativos** de gz-sim: no hace
falta `ros2_control`, que no está instalado en este equipo (y sin red no se
podía instalar).

```
panel / demo  --Float64-->  puente  --gz.msgs.Double-->  JointPositionController
                                                                  |
                                                               física
                                                                  |
robot_state_publisher <--JointState-- remuestreo <-- puente <--gz.msgs.Model--+
cinematica_directa.py <--------------/   60 Hz         1000 Hz
```

| Pieza | Dónde |
|---|---|
| PID de posición por junta (`JointPositionController` ×3) | `urdf/planta_brazo.gazebo.xacro` |
| publicación del estado real (`JointStatePublisher`) | `urdf/planta_brazo.gazebo.xacro` |
| suelo, luces, tarima y cámara de arranque | `worlds/lab_brazo.sdf` |
| puente ROS ↔ Gazebo, spawn y nodos | `launch/gazebo.launch.py` |

Tópicos añadidos en modo Gazebo:

- `/planta_brazo/J{1,2,3}/cmd_pos` — `std_msgs/Float64`, **consigna** de posición (rad)
- `/joint_states` — posición **real** medida por la física (60 Hz)
- `/joint_states_gz` — la misma sin remuestrear (1000 Hz, directa del puente)
- `/clock` — tiempo de simulación (todos los nodos van con `use_sim_time:=true`)

### Ajustes y por qué

- Las mallas se resuelven como `model://planta_brazo/meshes/…`; el launch pone
  `GZ_SIM_RESOURCE_PATH` apuntando a `install/planta_brazo/share`.
- El `JointStatePublisher` de gz-sim publica **una vez por paso de física**
  (1000 Hz con `max_step_size = 1 ms`). A esa cadencia `robot_state_publisher`
  inunda `/tf` sin ganar nada, por eso `remuestreo_juntas.py` lo baja a 60 Hz.
- **Gazebo sí hace cumplir `effort` y `velocity` del URDF.** Con los valores
  iniciales (2–3 N·m, 2.0 rad/s) la trayectoria pedía 2.46 rad/s en J1 y el
  robot se quedaba hasta 22° por detrás de la consigna. Subidos a 8–12 N·m y
  3.0–3.5 rad/s el seguimiento queda limpio. **No están medidos** de los servos
  reales: hay que ajustarlos con el par de bloqueo y la velocidad sin carga de
  cada motor.
- El mundo no usa ningún modelo de Fuel (suelo, tarima y luces están definidos
  a mano), así que arranca sin conexión a internet.

### Medidas de la simulación

Error **en reposo**, medido con `comparacion_cinematicas.py` sobre los 8 puntos
de prueba, en la punta de los dedos:

| | media | peor punto |
|---|---|---|
| yendo por la directa | 0.61 mm | 2.62 mm |
| yendo por la inversa | 0.52 mm | 1.58 mm |

Ese error lo fija el **controlador**, no la cinemática (ver la sección
siguiente). Para llegar ahí hubo que arreglar el término integral: con el
`i_max` original (4/10/6) el integrador se saturaba antes de compensar la
gravedad y J2 se quedaba clavado **4.35° por debajo** de la consigna con el
brazo estirado — 49 mm de caída en la punta. El detalle del ajuste está en la
cabecera de `urdf/planta_brazo.gazebo.xacro`.

En **movimiento** (trayectoria de demostración a 3 s por tramo) el rebote al
llegar a cada punto queda entre 3° y 5°. Subir `d_gain` para quitarlo lo
empeora (con el triple de derivativa el rebote pasa de 4.6° a 10.8°).

El factor de tiempo real es ≈ 1.0 (reloj de simulación a ~1000 Hz).

## Comparar las dos cinemáticas

```bash
ros2 launch planta_brazo gazebo.launch.py comparar:=true
```

Para cada punto de prueba se hacen **dos viajes independientes**, volviendo a
home entre medias para que ninguno se aproveche de dónde quedó el otro:

- **vía directa** — se le dan al robot los ángulos `q_ref`. El objetivo es
  `P_obj = CD(q_ref)`, un punto que por construcción existe y es alcanzable.
  Se mide `q_medido` en la física y el punto alcanzado es `CD(q_medido)`.
- **vía inversa** — se le da al robot solo el punto `P_obj`. La CI calcula unos
  ángulos `q_ci` y se manda esa terna. El punto alcanzado se mide igual.

Resultado (Gazebo, punta de los dedos, postura pedida a la CI: codo arriba):

| Punto | Objetivo X, Y, Z [mm] | Llegó por la DIRECTA [mm] | err | Llegó por la INVERSA [mm] | err |
|---|---|---|---|---|---|
| P1 home extendido | 490.4, 0.0, 342.7 | 490.4, −0.0, 342.3 | 0.36 | 490.4, −0.0, 342.3 | 0.36 |
| P2 frente arriba | 417.8, 0.0, 293.8 | 417.7, −0.0, 293.8 | 0.09 | 417.7, 0.0, 293.6 | 0.19 |
| P3 derecha media altura | 318.1, 318.1, 283.7 | 317.8, 318.4, 283.9 | 0.41 | 317.5, 318.1, 283.6 | 0.58 |
| P4 izquierda alta | 180.5, −312.7, 278.8 | 180.1, −312.8, 278.8 | 0.43 | 180.1, −312.8, 278.8 | 0.43 |
| P5 atrás baja | −364.8, 210.6, 133.5 | −364.3, 208.9, 131.6 | 2.62 | −364.8, 209.2, 132.8 | 1.58 |
| P6 lateral a 90 | 0.0, 327.8, 168.3 | −0.6, 327.7, 168.3 | 0.56 | −0.6, 327.7, 168.3 | 0.57 |
| P7 recogido | 178.4, 103.0, 211.6 | 178.2, 103.0, 211.6 | 0.16 | 178.2, 103.0, 211.6 | 0.16 |
| P8 codo abajo | 354.9, −204.9, 464.5 | 354.8, −205.1, 464.5 | 0.22 | 354.6, −205.0, 464.5 | 0.30 |
| | **media** | | **0.61** | | **0.52** |

Y los ángulos de cada vía:

| Punto | `q_ref` (entra a la DIRECTA) [°] | `q_ci` (sale de la INVERSA) [°] | Postura elegida | Soluciones (válidas) |
|---|---|---|---|---|
| P1 home extendido | 0.00, 0.00, 0.00 | 0.00, 0.00, −0.00 | hombro_frente/codo_arriba | 1 (1) |
| P2 frente arriba | 0.00, 40.00, −70.00 | 0.00, 40.00, −70.00 | hombro_frente/codo_arriba | 2 (2) |
| P3 derecha media altura | 45.00, 25.00, −50.00 | 45.00, 25.00, −50.00 | hombro_frente/codo_arriba | 2 (2) |
| P4 izquierda alta | −60.00, 55.00, −95.00 | −60.00, 55.00, −95.00 | hombro_frente/codo_arriba | 4 (2) |
| P5 atrás baja | 150.00, −10.00, −30.00 | 150.00, −10.00, −30.00 | hombro_frente/codo_arriba | 2 (2) |
| P6 lateral a 90 | 90.00, 30.00, −90.00 | 90.00, 30.00, −90.00 | hombro_frente/codo_arriba | 4 (1) |
| P7 recogido | 30.00, 70.00, −140.00 | 30.00, 70.00, −140.00 | hombro_frente/codo_arriba | 4 (2) |
| P8 codo abajo | −30.00, −25.00, 65.00 | **−30.00, 63.24, −65.00** | hombro_frente/codo_arriba | 2 (2) |

Tres cosas que se leen en estas tablas:

1. **Las dos vías llegan al mismo sitio**, con menos de 1 mm de diferencia
   entre ellas. Eso confirma que la CI es la inversa exacta de la CD.
2. **El error que queda no es de cinemática, es del controlador.** En modo
   RViz (cinemático puro, sin física) el mismo banco da 0.00 mm en las dos
   columnas: `ros2 run planta_brazo comparacion_cinematicas.py --ros-args -p
   modo:=rviz`. Los 0.5–2.6 mm de Gazebo son lo que le falta al PID para
   cerrar contra la gravedad en el tiempo de medida.
3. **P8 es el caso interesante**: la CI devuelve `(−30, 63.24, −65)` en vez de
   los `(−30, −25, 65)` de partida, y aun así **llega al mismo punto**. No es
   un fallo: ese punto tiene dos soluciones de codo y se pidió la de codo
   arriba. Es la demostración de que la inversa no tiene solución única y de
   que hay que elegir postura.

Los resultados quedan en `~/comparacion_cinematicas.md` y `.csv` para pegarlos
en el informe. Parámetros útiles: `preferencia` (`codo_arriba`, `codo_abajo`,
`cercana`), `extremo` (`tcp` o `brida`), `t_asentar`, `salida`.
