# Planeación de trayectorias en Gazebo

Añadido del **22 de septiembre de 2026**. Es el séptimo nodo del paquete, `tray`,
y el primero que usa la **cinemática inversa**: `model.py` traía `ik` e `ik_all`
escritos desde el principio y ningún nodo los llamaba.

**Qué se ha tocado:** nada de lo que funcionaba. Los seis nodos, el firmware, los
URDF y los YAML están igual. Lo añadido son cuatro archivos nuevos y **una línea**
en `setup.py`:

| archivo | qué es |
|---|---|
| `scara_kinematics/trayectoria.py` | la biblioteca: 5 perfiles + 6 caminos. Sin ROS y sin numpy |
| `scara_kinematics/tray.py` | el nodo: planifica, manda a Gazebo y dibuja |
| `launch/trayectoria.launch.py` | gz sim + el nodo, con `brazo:=false` |
| `TRAYECTORIAS.md` | este archivo |
| `setup.py` | +1 línea en `entry_points`: `'tray = scara_kinematics.tray:main'` |

- [1. Arranque](#1-arranque)
- [2. La idea: camino × perfil](#2-la-idea-camino--perfil)
- [3. Los cinco perfiles](#3-los-cinco-perfiles-el-tiempo)
- [4. Los seis caminos](#4-los-seis-caminos-la-forma)
- [5. Las órdenes](#5-las-órdenes)
- [6. Topics y parámetros](#6-topics-y-parámetros)
- [7. Lo medido en Gazebo](#7-lo-medido-en-gazebo)
- [8. Las tres trampas](#8-las-tres-trampas)
- [9. Tu deducción de la IK y la del código](#9-tu-deducción-de-la-ik-y-la-del-código)
- [10. Cuando conectes el brazo](#10-cuando-conectes-el-brazo)
- [11. Límites conocidos](#11-límites-conocidos)
- [12. La ventana: `tray_gui`](#12-la-ventana-tray_gui)

---

## 1. Arranque

```bash
cd ~/ros2_ws
colcon build --packages-select scara_kinematics scara_urdf
source install/setup.bash

ros2 launch scara_kinematics trayectoria.launch.py demo:=true
```

Eso abre Gazebo y recorre **los 25 métodos** solo: primero la misma recta con los
cinco perfiles (cambia el tiempo, no la forma) y después los cinco caminos con el
mismo perfil (cambia la forma, no el tiempo). Son unos dos minutos.

```bash
ros2 launch scara_kinematics trayectoria.launch.py demo:=true rviz:=true   # + el rastro
ros2 launch scara_kinematics trayectoria.launch.py demo:=true lazo:=true   # sin parar
ros2 launch scara_kinematics trayectoria.launch.py perfil:=trapecio        # sin demo
ros2 launch scara_kinematics trayectoria.launch.py registro:=~/tray.csv    # a CSV
ros2 launch scara_kinematics trayectoria.launch.py gui:=false              # gz sin ventana
```

Y con el brazo de la mesa conectado, en vez de la simulación
([sección 10](#10-cuando-conectes-el-brazo)):

```bash
ros2 launch scara_kinematics trayectoria.launch.py destino:=placa
```

Y sin `demo:=true` se queda esperando órdenes ([sección 5](#5-las-órdenes)):

```bash
ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'demo'"
ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'circulo 0.30 0 0.08 0.15 codo=down'"
```

**Las dos topologías.** El argumento `destino` elige entre las dos únicas
combinaciones que no se pelean por un topic:

| | `destino:=gz` (defecto) | `destino:=placa` |
|---|---|---|
| qué levanta | `gazebo.launch.py brazo:=false` | `gazebo.launch.py brazo:=true` (los seis nodos) |
| `tray` manda a | `/scara_urdf/cmd/<junta>` (Float64) | `scara/hw/cmd` (JointState) → la aduana |
| quién mueve gz | `tray` | `cc`, como espejo del brazo real |
| `/joint_states` | lo publica gz | lo publica `cc` (el brazo real) |
| techo de velocidad | `max_vel` × `factor_vel` | `max_vel_hw` (husillo **0.008 m/s**) |
| enclavamiento | ninguno | no se mueve sin `scara/hw/ok` |

Con `destino:=placa`, `tray` **no toca** los mandos de Gazebo: publica la consigna
en `scara/hw/cmd` y la aduana la convierte ella sola en la línea `T q1 q2 q3` del
protocolo. El espejo en gz lo sigue haciendo `cc`, que es su trabajo desde
siempre. `esp.py` no se ha tocado: ese topic ya existía.

**Por qué `brazo:=false` con `destino:=gz`.** El launch incluye `gazebo.launch.py` con la cadena de
los seis nodos apagada. No es por prudencia: `cc`, con `publicar_gz`, escribe en
los **mismos** `/scara_urdf/cmd/<junta>` que escribe `tray`. Los dos a la vez sobre
el mismo controlador es una pelea que gana el último que publique. Con
`brazo:=false` tampoco se abre `/dev/ttyUSB0`, así que se puede planificar con la
placa desconectada, y `/joint_states` lo publica Gazebo — que es justo lo que
`tray` lee para dibujar lo que la simulación hizo **de verdad**.

---

## 2. La idea: camino × perfil

Una trayectoria son siempre **dos cosas independientes**, y en `trayectoria.py`
están separadas a propósito:

```
    CAMINO   la forma: por dónde pasa la punta      u ∈ [0,1] ──► q
    PERFIL   el tiempo: cómo se recorre esa forma   t ∈ [0,T] ──► u

                        q(t) = camino( perfil(t/T) )
```

Separarlas es lo que permite combinar **5 perfiles × 5 caminos = 25 métodos** sin
escribir 25 funciones. El perfil no sabe si el camino es una recta o un círculo, y
el camino no sabe si se recorre en 2 o en 20 segundos.

```
  scara/tray/orden  ──►  parser  ──►  Camino  ─┐
        (texto)                                ├──►  Trayectoria  ──► Float64 ──► gz
                              Perfil  ─────────┘      (tabla de 400
                                                       poses + reloj)
```

---

## 3. Los cinco perfiles (el tiempo)

Todos van de `s(0)=0` a `s(1)=1` con velocidad nula en los dos extremos. Lo que
los distingue es **cuántas derivadas dejan continuas** y cuánto pico de velocidad
piden a cambio.

| perfil | fórmula | `kv` | `ka` | continuidad | para qué |
|---|---|---|---|---|---|
| `trapecio` | LSPB, rampa–crucero–rampa | 1.333 | 5.33 | posición, velocidad | el más rápido para un techo de velocidad dado; el par salta al arrancar |
| `cubico` | `3u² − 2u³` | 1.500 | 6.00 | posición, velocidad | el clásico de tres coeficientes |
| `scurve` | doble S, jerk acotado | 1.538 | 7.69 | + jerk acotado | cuando el brazo lleva algo que no puede dar un tirón |
| `quintico` | `10u³ − 15u⁴ + 6u⁵` | 1.875 | 5.77 | + aceleración | **el de por defecto**: arranca y para sin escalón de par |
| `septico` | `35u⁴ − 84u⁵ + 70u⁶ − 20u⁷` | 2.188 | 7.51 | + jerk | el más suave; el que mejor sigue Gazebo |

`kv` es el **pico de velocidad normalizada**: con un recorrido `D` y una duración
`T`, la velocidad máxima vale `kv·D/T`. Es exactamente lo que usa `duracion_auto`
para elegir `T` sin pasarse del `max_vel` del YAML:

```
    T = máx_j ( pico_j · kv / max_vel_j )
```

Y es lo que explica la tabla de tiempos: **la misma recta** sale en 0.82 s con
`trapecio` y en 1.34 s con `septico`, porque el séptico gasta el presupuesto de
velocidad en un pico más alto y estrecho. No es que uno sea mejor: es el canje.

Los dos parámetricos se pueden afinar desde el código:
`PerfilTrapecio(beta=0.25)` (fracción de rampa) y `PerfilScurve(tj=0.15, ta=0.35)`
(rampa de jerk y fase de aceleración).

---

## 4. Los seis caminos (la forma)

| camino | qué es | IK | puede fallar |
|---|---|---|---|
| `junta` | interpolación articular, junta a junta | no | nunca |
| `recta` | línea recta cartesiana | sí | si se sale del anillo |
| `arco` | arco, círculo o hélice; también **por tres puntos** | sí | sí |
| `poligonal` | segmentos rectos por N puntos vía | sí | sí |
| `spline` | spline cúbica natural por N puntos vía | sí | sí |
| `spline_junta` | la misma spline, pero sobre las juntas | no | nunca |

**`junta` frente a `recta`** es la distinción de fondo. `junta` no resuelve nada:
interpola los tres números y ya está, así que nunca falla y nunca cruza una
singularidad — pero la punta describe una curva que no es nada reconocible.
`recta` hace que la punta vaya recta, que es lo que hace falta para seguir un
borde, a costa de resolver la IK en cada muestra y de poder quedarse sin solución
a mitad.

**`poligonal` frente a `spline`**: las dos pasan por todos los puntos vía. La
poligonal deja una esquina viva en cada uno (la velocidad cambia de dirección de
golpe); la spline no, pero se sale del segmento recto entre puntos, así que no
sirve si hay un obstáculo pegado a la línea.

Los dos usan **parametrización por longitud de cuerda**: el parámetro avanza con
la distancia, no con el índice del punto, para que un tramo largo y uno corto no
se recorran en el mismo tiempo.

---

## 5. Las órdenes

Un solo topic de texto, `scara/tray/orden` (`std_msgs/String`). Sin mensajes
nuevos: no hace falta un paquete de `.msg` ni tocar `package.xml`.

| orden | argumentos | qué hace |
|---|---|---|
| `junta` | `q1 q2 d3` | interpolación articular (rad, rad, m) |
| `recta` | `x y z` | recta cartesiana desde donde esté |
| `arco` | `cx cy r a0 a1 [z]` | arco; **a0 y a1 en GRADOS** |
| `circulo` | `cx cy r [z] [vueltas]` | círculo completo |
| `tres` | `x1 y1 x2 y2 x3 y3 [z]` | arco por tres puntos |
| `poligonal` | `x1 y1 z1 x2 y2 z2 …` | tramos rectos por N vía |
| `spline` | `x1 y1 z1 x2 y2 z2 …` | spline cúbica por N vía |
| `home` | — | vuelve a la pose de reposo |
| `perfiles` | — | la misma recta con los cinco perfiles |
| `caminos` | — | los cinco caminos con el perfil de ahora |
| `demo` | — | las dos anteriores, seguidas |
| `parar` | — | corta y se queda donde esté |

En **cualquier** orden y en **cualquier sitio** se pueden meter estas opciones:

```
    perfil=cubico|quintico|septico|trapecio|scurve
    t=3.5          duración fija en segundos (por defecto la calcula el max_vel)
    codo=up|down|nearest|auto
```

```bash
ros2 topic pub --once /scara/tray/orden std_msgs/String \
  "data: 'spline 0.26 -0.09 0.14 0.36 -0.09 0.17 0.36 0.09 0.11 perfil=septico t=8'"
```

El `demo` no tiene un camino de código propio: es literalmente una lista de estas
órdenes (`GUIA_PERFILES` y `GUIA_CAMINOS` en `tray.py`) que pasa por el mismo
parser. Lo que se ve en el demo se puede teclear.

También escucha `scara/tray/meta` (`geometry_msgs/PoseStamped`), que equivale a
una orden `recta` a esa posición.

---

## 6. Topics y parámetros

**Envía**

| topic | tipo | qué |
|---|---|---|
| `<cmd_prefix><junta>` | `std_msgs/Float64` | la consigna, a 1/`dt` Hz, a los controladores de gz |
| `scara/tray/plan` | `nav_msgs/Path` | el camino planificado entero, **antes** de moverse |
| `scara/tray/real` | `nav_msgs/Path` | lo que hizo Gazebo, leído de `/joint_states` |
| `scara/tray/via` | `visualization_msgs/Marker` | los puntos vía (naranja) y el nombre del método |
| `scara/tray/estado` | `std_msgs/String` | qué corre y por qué porcentaje va |

**Recibe:** `joint_states` (de gz), `scara/tray/orden`, `scara/tray/meta`.

**Parámetros propios** (además de los del modelo, que declara igual que los seis):

| parámetro | defecto | qué |
|---|---|---|
| `destino` | `gz` | `gz`, `placa` o `ambos`: a quién van las consignas |
| `max_vel_hw` | `[1.5, 1.5, 0.008]` | el techo real del hardware, el mismo que declara `esp.py` |
| `perfil` | `quintico` | el perfil por defecto |
| `codo` | `auto` | `auto` prueba `nearest`, `up` y `down` y se queda con la que aguante el camino entero |
| `duracion` | `0.0` | segundos por movimiento; `0` = lo calcula el `max_vel` |
| `factor_vel` | `0.35` | fracción del `max_vel` del YAML que Gazebo sigue de verdad ([sección 8](#8-las-tres-trampas)) |
| `dt` | `0.02` | periodo de la consigna (50 Hz) |
| `muestras` | `400` | puntos de la tabla de geometría |
| `aprox` | `0.02` | salto articular a partir del cual se mete un tramo de aproximación |
| `home` | `[0.30, 0.0, 0.15]` | la pose de reposo, en cartesiano |
| `demo` / `lazo` | `false` | la visita guiada, y si se repite |
| `espera` | `6.0` | margen para que gz cargue el robot antes de empezar el demo |
| `registro` | `''` | CSV con lo pedido y lo que hizo gz |

El CSV lleva, por muestra: `t`, `metodo`, las tres juntas pedidas, las tres
medidas, y el TCP pedido y el medido. Es lo que hay que graficar para el informe.

---

## 7. Lo medido en Gazebo

Demo entero, `gui:=false`, 5.512 muestras a 50 Hz, `factor_vel:=0.35`. Error del
TCP entre lo pedido y lo que hizo la simulación:

| método | muestras | error medio | error pico |
|---|---|---|---|
| `spline` + quíntico | 457 | **3.06 mm** | 7.42 mm |
| `junta` + quíntico | 631 | 3.87 mm | 7.97 mm |
| `poligonal` + quíntico | 578 | 6.20 mm | 13.16 mm |
| `arco` + quíntico | 625 | 7.90 mm | 19.27 mm |
| `recta` + **séptico** | 388 | **7.88 mm** | 19.14 mm |
| `recta` + quíntico | 1361 | 9.56 mm | 23.64 mm |
| `recta` + scurve | 274 | 11.32 mm | 19.69 mm |
| `recta` + cúbico | 266 | 11.56 mm | 19.15 mm |
| `recta` + **trapecio** | 238 | **13.75 mm** | 19.98 mm |
| **global** | 5.512 | **7.72 mm** | 34.06 mm |

Las cinco filas de `recta` son **la misma recta**, planificada igual, y solo cambia
el perfil. Salen ordenadas por suavidad: séptico < quíntico < scurve < cúbico <
trapecio. Cuanto más continuo el perfil, mejor lo sigue el PID de Gazebo. Eso es
el resultado del ejercicio y está medido, no supuesto.

Y el barrido de `factor_vel` sobre el demo entero:

| `factor_vel` | duración del demo | error medio | error pico |
|---|---|---|---|
| 0.60 | 55 s | 14.31 mm | 49.25 mm |
| **0.35** | 122 s | **7.38 mm** | 34.06 mm |

El error escala con la velocidad pedida, como cabe esperar de un retraso de lazo.

---

## 8. Las tres trampas

Las tres se encontraron ejecutando esto en Gazebo, y las tres están resueltas en
el código. Se dejan escritas porque ninguna es obvia.

### 1. El salto de rama de la IK

Un SCARA tiene **dos** soluciones (codo arriba y codo abajo). `model.ik` se queda
con las que caben en los límites de junta. Si la rama que se venía siguiendo se
sale —el codo topa con `q2_max` a mitad de un círculo— devuelve la otra **sin
avisar**, y entre dos muestras seguidas el codo se da la vuelta entera.

Medido: en un círculo de r = 0.08 m centrado en (0.30, 0), en `u = 0.435` la junta
2 saltaba **4.38 rad en 20 ms**. Con eso, `duracion_auto` pedía T = 1095 s para no
pasarse de velocidad, y el error del TCP llegaba a **353 mm**.

**Cómo está resuelto:** `Trayectoria` tabula la geometría **entera** en el
constructor, antes de mover nada, y `_sin_saltos` se planta con un `IKError` que
dice en qué `u` y qué junta. Y `planificar` prueba las tres ramas y se queda con
la primera que llega al final. En ese círculo elige `codo=down` y sale en 3.12 s
con 7.90 mm de error.

### 2. El camino no empieza donde está el brazo

`recta` y `junta` arrancan donde esté el robot, pero un `arco`, una `spline` o una
`poligonal` empiezan **en su primer punto vía**, que puede estar en la otra punta
del espacio de trabajo y además en la otra rama del codo. Mandado tal cual, la
primera consigna es un escalón de varios radianes.

**Cómo está resuelto:** si el salto pasa de `aprox`, `tray` mete solo un tramo de
**aproximación en espacio articular** hasta `tabla[0]`. En articular, porque va
exactamente a esa pose sin resolver IK y por tanto no puede equivocarse de rama.
Se anuncia en el log: `aproximacion: 0.985 de la pose de ahora al principio de arco`.

### 3. El `max_vel` del YAML no es el de Gazebo

`max_vel: [1.5, 1.5, 0.10]` es el techo del **modelo**. El
`JointPositionController` de gz, con las ganancias que trae el URDF
(`p_gain` 60 y 30, `cmd_max` 12 y 8 N·m), no lo sigue: pedirle 1.5 rad/s deja
**40 mm** de retraso en la punta.

**Cómo está resuelto:** el parámetro `factor_vel` (0.35) escala ese techo. No se
ha tocado el URDF ni el YAML: subir las ganancias del controlador sería la otra
solución, pero eso es cambiar el robot, y el robot no se toca. Con `factor_vel:=1`
se ve el efecto: el mismo demo en 39 s y con 34 mm de error medio.

---

## 9. Tu deducción de la IK y la del código

La deducción del documento (`c₂ = (Pₓ² + P_y² − L₂² − L₃²)/2L₂L₃`, `θ₂ = atan2(±√(1−c₂²), c₂)`,
`θ₁ = atan2(k₁P_y − k₂Pₓ, k₁Pₓ + k₂P_y)`) es **exactamente** la que implementa
`model.ik_all`. Comprobado sobre 40.000 puntos al azar y las dos ramas, poniendo
`b1 = b2 = 0`: la peor diferencia en las juntas es **1.1e-15 rad**, y el error de
cierre `FK(IK(P)) − P` es **cero** en doble precisión. Son la misma fórmula escrita
de dos maneras.

Dos avisos, por si esos números van a un informe:

**Los largos del documento no son los de este robot.**

| | documento | `scara_urdf3.yaml` (el robot montado) |
|---|---|---|
| brazo 1 | L₂ = 0.150 m | l1 = **0.2500** m |
| brazo 2 | L₃ = 0.120 m | l2 = **0.2500** m |
| alcance | 0.270 m | **0.500 m** |
| husillo | Pz = L₁ − d₃ (baja) | z = 0.06551 **+** d₃ (sube) |

**Y el robot real no es el 2R de libro.** Al salir de SolidWorks, los ejes 1 y 2
no arrancan alineados con el eje X de la base: el YAML lleva `b1 = +0.2269 rad` y
`b2 = −0.2029 rad`, que son el giro fijo de cada `<origin>` del URDF. Por eso
`model.py` escribe

```
    x = l1·cos(q1 + b1) + l2·cos(q1 + q2 + b2)
```

y no `L₂cos θ₁ + L₃cos(θ₁+θ₂)`. Con `b1 = b2 = 0` las dos se vuelven la misma, que
es justo lo que comprueba el párrafo de arriba. Si el informe lleva la fórmula sin
los offsets, la punta se va unos 13° de donde dice.

---

## 10. Cuando conectes el brazo

El camino de `tray` a la placa está escrito y probado **contra una aduana falsa**,
no contra la ESP32: 917 consignas por `scara/hw/cmd`, husillo clavado en 0.008 m/s
y salto máximo entre muestras de 0.030 rad. Lo que no ha visto nunca es la placa.
Por eso el nodo no se mueve solo.

**El orden de arranque**

```bash
cd ~/ros2_ws && colcon build --packages-select scara_kinematics scara_urdf
source install/setup.bash
ros2 launch scara_kinematics trayectoria.launch.py destino:=placa
```

1. **`scara/hw/ok` tiene que ser `true`.** La aduana solo lo pone en true si la
   placa contesta, está habilitada y tiene el cero hecho. Los tres se piden desde
   la ventana `brazo_hw`, que este launch abre (o a mano: los servicios
   `scara/hw/enable` y `scara/hw/zero`). Mientras sea false, `tray` rechaza cada
   orden con el motivo escrito y **no publica nada**.

   ```bash
   ros2 topic echo /scara/hw/ok        # tiene que decir data: true
   ```

2. **Comprobar de quién viene `/joint_states`.** Con `destino:=placa` lo publica
   `cc`, o sea el brazo real; lo de Gazebo sale por `gz/joint_states`. Si están al
   revés, la trayectoria se planificaría desde la pose del robot simulado.

   ```bash
   ros2 topic info /joint_states --verbose | grep -i node
   ```

3. **El primer movimiento, corto y en articular.** `junta` no resuelve IK, así que
   no puede equivocarse de rama, y con `t=` largo se ve venir:

   ```bash
   ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'junta -1.15 2.28 0.085 t=8'"
   ```

   Después ya en cartesiano, y el `demo` al final, que es el que mueve de verdad.

4. **Mirar los vigilantes mientras se mueve.** Son los de los nodos de motor y ya
   existen: `scara/<motor>/atascado` (PWM sin movimiento) y `scara/<motor>/tope`.

   ```bash
   ros2 topic echo /scara/alvin/atascado &
   ros2 topic echo /scara/simon/atascado &
   ros2 topic echo /scara/teodoro/atascado &
   ```

5. **`parar` corta en cualquier momento**, y si `scara/hw/ok` se cae a mitad de un
   movimiento el nodo aborta solo y lo dice.

**Lo que va a sorprender: el husillo.** `max_vel_hw` dice 0.008 m/s frente a los
0.10 del YAML — **12,5 veces más lento**. Una recta que en Gazebo tarda 1.15 s
puede tardar 12,8 s en la mesa, y no es un fallo: es que bajar un centímetro son
1,25 s de husillo. Si hace falta más, ese número está en `brazo.launch.py`
(`MAX_VEL_HW`) y en `esp.py`, y hay que medirlo antes de subirlo.

**Y `factor_vel` no se aplica a la placa.** Es una corrección del PID de Gazebo.
Para el brazo real manda `max_vel_hw`, que ya es el límite medido.

---

## 11. Límites conocidos

- **El camino a la placa no se ha probado contra la ESP32.** Está escrito y
  verificado contra una aduana falsa ([sección 10](#10-cuando-conectes-el-brazo)),
  pero nunca ha movido un motor. Todo lo demás de este archivo sí está medido en
  Gazebo.
- **Sin evitación de obstáculos.** Ninguno de los seis caminos mira si hay algo en
  medio. Solo comprueba alcance, límites de junta y continuidad de rama.
- **`tray` y `cc` nunca mandan a la vez.** Con `destino:=gz` manda `tray` y `cc`
  no existe; con `destino:=placa` manda `tray` a la placa y `cc` solo hace el
  espejo en gz. El `destino:=ambos` del nodo existe pero el launch no lo ofrece:
  ahí `tray` escribiría en los mandos de gz al mismo tiempo que `cc`.
- **La duración mínima es 0.5 s** (`t_min` en `duracion_auto`), aunque el
  movimiento sea de un milímetro.
- **El husillo no limita la duración en los caminos planos.** Un arco a z constante
  no mueve `vertiacal1`, así que su `max_vel` no entra en el cálculo de T.
- **`manita` no existe** en el brazo de 3 GDL: Gazebo la deja en su cero, igual que
  con los seis nodos.

---

## 12. La ventana: `tray_gui`

Añadido del **25 de septiembre de 2026**. Es la cara de `tray`: en vez de teclear
`ros2 topic pub ... "data: 'circulo 0.30 0 0.08 0.15'"`, se elige el camino, se
pinchan los puntos sobre la planta y se pulsa Enter.

```bash
ros2 launch scara_kinematics tray_gui.launch.py            # gz + tray + la ventana
ros2 launch scara_kinematics tray_gui.launch.py rviz:=true  # y además RViz
```

Todos los argumentos de `trayectoria.launch.py` valen igual (`perfil:=`, `codo:=`,
`factor_vel:=`, `gui:=false`...): el launch nuevo lo **incluye** tal cual y añade la
ventana. **Cerrar la ventana cierra todo** (Gazebo, `tray` y el puente).

**Qué se ha tocado:** otra vez nada de lo que funcionaba. `tray.py`,
`trayectoria.py`, los seis nodos y los launch que había están igual.

| archivo | qué es |
|---|---|
| `scara_kinematics/tray_gui.py` | la ventana (tkinter, los colores de `estilo.py`) |
| `launch/tray_gui.launch.py` | incluye `trayectoria.launch.py` y añade la ventana |
| `setup.py` | +1 línea en `entry_points`: `'tray_gui = scara_kinematics.tray_gui:main'` |
| `package.xml` | +2 `exec_depend`: `rcl_interfaces` (`/rosout`) y `rosgraph_msgs` (`/clock`) |

### Qué hay en ella

```
+--------------------------------+-------------------------+-------------------+
| Planta, vista desde arriba  |z | Nueva trayectoria       | Atajos            |
|                             |  |  [Recta][Junta][Arco].. |  Home Perfiles .. |
|   zona sombreada = alcanza  |r |  campos del camino      +-------------------+
|   brazo gris = Gazebo ahora |e |  Perfil  Codo  t        | Ahora             |
|   azul ┅ = borrador         |g |  Orden: circulo 0.30 .. |  método, barra,   |
|   naranja = plan de tray    |l |  ✓ sale: T = 8.92 s ... |  juntas, punta,   |
|   verde = la punta real     |a |  [Ejecutar]  [PARAR]    |  desvío del plan  |
|                             |  |                         +-------------------+
|                             |  |                         | Mensajes de tray  |
+--------------------------------+-------------------------+-------------------+
```

- **La planta.** La zona sombreada es donde llega la punta *de verdad*, con los
  límites de junta (q1 no pasa de ±120°); más oscuro, donde llegan las dos ramas del
  codo. Al pasar el ratón dice x, y, r y si se alcanza.
- **El clic** pone el punto, y lo que signifique depende del camino: el destino de
  la recta; el centro y luego el borde del círculo; centro, inicio y final del arco;
  los tres puntos de `tres`; un punto más en `poligonal` y `spline`. En `junta`, el
  clic saca q1 y q2 por IK. El clic derecho deshace. Se redondea a 5 mm (un pixel
  son unos 2.5 mm); para algo exacto, se teclea en los campos.
- **La regla de z**, a la derecha: la barra es el recorrido del husillo, el
  triángulo verde es dónde está la punta, y el azul/naranja lo que ocupa en z el
  borrador/el plan. Un clic en ella pone la z.
- **El borrador** (azul discontinuo) es lo que haría `tray` si mandases la orden
  *ahora*, con su duración y su rama del codo. Si la IK no llega, sale en **rojo**
  por donde iría el camino (se ve dónde se sale de la zona sombreada), el motivo
  debajo, y **Ejecutar** se apaga.
- **La orden** que se manda está escrita debajo, tal cual se teclearía a mano.
- **Ahora**: el método y el % que publica `tray`, las juntas y la punta que mide
  Gazebo, y el **desvío**: cuánto se separa la punta verde del plan naranja
  (el círculo del demo: 2.8 mm de máximo).
- **Pausar Gazebo**, arriba: pausa la física con `gz service`. Como `tray` corre
  con el reloj de la simulación, se queda congelado a mitad de trayectoria y sigue
  donde estaba al reanudar.

| tecla | qué hace |
|---|---|
| Enter | manda la orden (o la pone en cola si `tray` está ocupado) |
| Esc | `parar` |
| clic / clic dcho. | pone el punto / deshace |

### Por qué el borrador es de fiar

La ventana **no tiene su propio parser**. Compone el texto, y lo pasa por
`Planificador._partir`, `Planificador._camino` y `Planificador._techo` de `tray.py`
(llamados con un objeto `_Sombra` que tiene lo poco que miran de `self`) y por el
mismo `trayectoria.planificar`. Es decir: el borrador y lo que ejecuta `tray` salen
del mismo código, y no hay dos traducciones de `'circulo 0.30 0 0.08'` que puedan
divergir. Tarda unos 13 ms por orden.

Lo que **no** puede ver: la cola. Si `tray` está ocupado, el borrador sale de la
pose de ahora, pero la orden se ejecutará desde donde acabe la anterior. La
ventana lo avisa.

### Qué escucha

| topic | para qué |
|---|---|
| `scara/tray/orden` (sale) | lo único que manda |
| `joint_states` | el brazo gris, la punta verde y el origen del borrador |
| `/clock` | Gazebo corre, está en pausa (llega pero no avanza) o no está |
| `scara/tray/estado` | qué hace `tray` y por dónde va |
| `scara/tray/plan`, `scara/tray/via` | el camino naranja y sus puntos |
| `/rosout` | los mensajes de `tray` (incluidos los «no sale») |

**Una trampa de la pausa:** con Gazebo en pausa, `/clock` sigue llegando (a unos
500 Hz, con el mismo valor) pero `/joint_states` se para. Por eso la ventana
distingue «en pausa» de «sin Gazebo» mirando si el reloj *avanza*, no si llega.

Solo la ventana, contra una simulación que ya corre:

```bash
ros2 run scara_kinematics tray_gui --ros-args --params-file \
    install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
```

---

Ver también: [NODOS.md](NODOS.md) (los seis nodos) y
[../scara_urdf/URDF.md](../scara_urdf/URDF.md) (el dibujo y los plugins de gz).
