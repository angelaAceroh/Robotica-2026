# Los seis nodos del SCARA

Referencia **única** de `scara_kinematics`: el 15 de septiembre de 2026 el
`README.md` del paquete se fundió aquí, así que este es el solo archivo que hay
que leer. Está todo: qué hace cada carpeta y qué pasa si la borras, cada topic,
cada parámetro, cada servicio, cada aviso, el protocolo con la placa y por qué
está montado así. El paquete quedó repartido en seis nodos el **14 de septiembre
de 2026**, y el **28 de septiembre de 2026** se revisó cómo se comunican: cada
motor lleva su propio código y cada muestra de la placa viaja etiquetada, para
que `cc` junte las tres juntas del mismo instante ([sección 1](#cómo-viaja-una-muestra)).

- [1. El mapa](#1-el-mapa)
- [2. El paquete por dentro](#2-el-paquete-por-dentro-qué-hace-cada-cosa-y-qué-pasa-si-la-borras)
- [3. Arranque](#3-arranque)
- [4. Los nodos, uno a uno](#4-los-nodos-uno-a-uno)
- [5. Tabla completa de topics](#5-tabla-completa-de-topics)
- [6. El protocolo con la ESP32](#6-el-protocolo-con-la-esp32)
- [7. Los motores y el cableado](#7-los-motores-y-el-cableado)
- [8. Los parámetros del modelo](#8-los-parámetros-del-modelo)
- [9. Las bibliotecas](#9-las-bibliotecas)
- [10. Los launch](#10-los-launch)
- [11. Diagnóstico](#11-diagnóstico)
- [12. Límites conocidos](#12-límites-conocidos)

---

## 1. El mapa

```
                         ┌──────────────────────────────────────┐
   ESP32 ──USB serie──▶  │  5. esp          LA ADUANA           │
                         │  el único que abre /dev/ttyUSB0      │
                         └───┬──────────┬──────────┬────────────┘
        scara/alvin/aduana ──┘          │          └── scara/teodoro/aduana
                                        │
                        scara/simon/aduana
                │                       │                      │
       ┌────────▼───────┐     ┌─────────▼──────┐     ┌─────────▼────────┐
       │  2. alvin      │     │  3. simon      │     │  4. teodoro      │
       │  juntura1      │     │  juntura2      │     │  vertiacal1      │
       │  hombro        │     │  codo          │     │  husillo         │
       └────────┬───────┘     └─────────┬──────┘     └─────────┬────────┘
                │   scara/<motor>/junta │                      │
                └───────────────┬───────┴──────────────────────┘
                                │
                   ┌────────────▼─────────────┐
                   │  1. cc                   │
                   │  cinemática directa      │
                   │  el único que envía Y    │
                   │  recibe                  │
                   └──┬────────┬──────────┬───┘
         joint_states │        │          │ Float64 (solo con publicar_gz)
                      │        │          └────────────▶ Gazebo
      robot_state_publisher    │ scara/ee_pose
              │                └──────────────▶ ┌──────────────────┐
              ▼                                 │  6. esp_rec      │
            RViz                                │  solo recibe     │
                                                └──────────────────┘
```

Y aparte, la única ventana: **`brazo_hw`**, la interfaz del brazo. No calcula
nada; mira topics y pide cero y potencia por servicio.

| # | ejecutable | qué es | recibe | envía |
|---|---|---|---|---|
| 1 | `cc` | cinemática directa | las tres juntas | `/joint_states`, `scara/ee_pose` |
| 2 | `alvin` | el hombro (`juntura1`) | `scara/alvin/aduana` | `scara/alvin/junta` |
| 3 | `simon` | el codo (`juntura2`) | `scara/simon/aduana` | `scara/simon/junta` |
| 4 | `teodoro` | el husillo (`vertiacal1`) | `scara/teodoro/aduana` | `scara/teodoro/junta` |
| 5 | `esp` | la **aduana** de la ESP32 | el puerto serie | un topic por motor |
| 6 | `esp_rec` | el receptor | `scara/ee_pose` de `cc` | texto, CSV, y la consigna si se le pide |

**Tres reglas que explican el diseño:**

1. **La aduana es la única que abre el puerto serie.** No es un gusto: dos
   procesos no pueden tener `/dev/ttyUSB0` abierto a la vez. Por eso `esp_rec`
   no le escribe a la placa, sino que le pasa la consigna a la aduana por
   `scara/hw/cmd`, y ella la escribe.
2. **`cc` es el único que recibe y envía.** Los tres motores solo reciben (de la
   aduana) y `esp_rec` solo recibe (de `cc`).
3. **Un nodo por motor** para que, cuando algo falle, se sepa cuál.

### Cómo viaja una muestra

Cada línea de telemetría de la placa es **una muestra**: el vector
`[q1 q2 d3]` medido en el mismo instante. Por el camino se parte en tres piezas,
una por motor, y `cc` la vuelve a juntar. Para que junte las piezas del **mismo**
instante, cada pieza lleva una etiqueta:

```
ESP32   > 12040 0.20071 -0.34907 0.09000 0.150 -0.120 0.000 7*3D
          │     └───────── q ──────────┘ └────── u ───────┘ │ └ checksum
          └ reloj de la placa (ms)                          └ flags

esp     comprueba el checksum (si no cuadra, tira la línea entera)
        le pone número:  muestra=1234 placa_ms=12040 cero=1
        saca la velocidad con el reloj de la PLACA
        ├─ scara/alvin/aduana    [juntura1]   ┐
        ├─ scara/simon/aduana    [juntura2]   ├ mismo sello, misma etiqueta
        └─ scara/teodoro/aduana  [vertiacal1] ┘

alvin, simon, teodoro   ¿es mía?, ¿entera?, ¿en orden? → scara/<motor>/junta, tal cual

cc      espera las tres piezas de la muestra 1234
        → UN joint_states [q1 q2 d3] con el sello de esa muestra
```

- La etiqueta va en `header.frame_id`, que en un `JointState` no usa nadie: una
  junta no tiene marco de referencia. Se ve con
  `ros2 topic echo /scara/alvin/aduana --field header`.
- **El sello no basta** para juntar: con `use_sim_time` el reloj es el de
  Gazebo, que vale 0 hasta que arranca y se para con la simulación, y muestras
  distintas saldrían con el mismo sello. El número de muestra no se repite nunca.
- **Una muestra, un `joint_states`**: sale a la frecuencia de la telemetría
  (25 Hz), ni repetido ni mezclando juntas de dos instantes.
- Medido contra una placa falsa que metía líneas rotas, muestras saltadas,
  repetidas y ráfagas de USB: antes del cambio, en 30 s, 783 `joint_states`
  repetían la medida anterior, 24 traían un valor que la placa **nunca midió**
  (líneas rotas que se colaban) y la velocidad llegaba a 12.7 rad/s con una
  real de 0.38. Después: 0, 0 y 0.377 rad/s.

---

## 2. El paquete por dentro: qué hace cada cosa y qué pasa si la borras

Arriba está el robot; abajo, el pegamento que ROS 2 exige para reconocer el
paquete. Ninguna de las dos mitades sobra.

### El workspace `~/ros2_ws`

| | qué es | si lo borras |
|---|---|---|
| `src/` | los dos paquetes: `scara_kinematics` (los nodos) y `scara_urdf` (el dibujo). **Aquí vive todo el código** | te quedas sin robot: los seis nodos *son* los `.py` de aquí dentro |
| `build/` `install/` `log/` | las crea `colcon build` él solo. `install/` es la copia compilada que usan `ros2 run` y `ros2 launch`; `log/` son los registros de cada compilación | nada permanente: reaparecen en cuanto vuelvas a compilar. Pero hasta que compiles, no arranca nada |
| `abrir.sh` | el atajo que compila, lanza y para todo ([sección 3](#3-arranque)) | hay que lanzar y matar a mano |

**El nombre repetido no es un error.** Los nodos están en
`src/scara_kinematics/scara_kinematics/`: la primera carpeta es el paquete (con
`package.xml`, `setup.py`, `launch/`…) y la segunda es el módulo de Python. Lo
exige `ament_python`, porque `setup.py` importa los nodos como
`scara_kinematics.cc`: el código tiene que vivir en una carpeta que se llame
igual que el paquete. Si se aplana, `ros2 run scara_kinematics cc` deja de
encontrarlos.

### Dentro de `scara_kinematics/`: el robot

| | qué es | si lo borras |
|---|---|---|
| `scara_kinematics/` (la de dentro) | **el código**: los seis nodos, `brazo_hw.py` y las bibliotecas que comparten ([sección 9](#9-las-bibliotecas)). Alvin, Simón y Teodoro llevan cada uno su código entero |  te quedas sin robot |
| `config/` | los números del brazo. `scara_urdf3.yaml` **es el que se usa** (3 GDL, el brazo montado); `scara_urdf.yaml` es el de 4 GDL, con muñeca. Largos `l1`/`l2`, topes, `max_vel` ([sección 8](#8-los-parámetros-del-modelo)) | los nodos arrancan con los `DEFAULT_PARAMS` de `model.py`, que son de otro robot: no fallan, mienten |
| `launch/` | los tres arranques: `brazo`, `rviz` y `gazebo` ([sección 10](#10-los-launch)) | hay que levantar los seis nodos a mano, uno por terminal |
| `firmware/` | lo que corre **dentro de la ESP32**, no en la PC: `scara_esp32.ino` y su README. `.ino.bak4gdl` es el respaldo de la versión de 4 GDL | pierdes el programa de la placa, que es quien implementa el protocolo de la [sección 6](#6-el-protocolo-con-la-esp32) |
| `rviz/` | `scara.rviz`: qué se dibuja y cómo queda colocada la ventana de RViz | RViz abre vacío y hay que rehacer la vista a mano |
| `test/` | `test_kinematics.py`, que sostiene a `cc`, y `test_protocolo.py`, que sostiene a la aduana ([sección 9](#9-las-bibliotecas)) | no se rompe nada: pierdes la red de seguridad |
| `NODOS.md` | este archivo | nada, es texto |

### Los cuatro que exige ROS 2

| | qué hace | si lo borras |
|---|---|---|
| `package.xml` | el carné del paquete: nombre, versión, mantenedor y las dependencias (`rclpy`, `python3-serial` para el puerto, `python3-tk` para la ventana, `ros_gz_sim`…). Declara `ament_python`, que le dice a colcon cómo compilarlo | colcon no ve la carpeta como paquete: deja de compilar |
| `setup.py` | la receta. `entry_points` convierte cada archivo en un comando —la línea `'cc = scara_kinematics.cc:main'` es lo que hace funcionar `ros2 run scara_kinematics cc`— y `data_files` dice qué se copia a `install/share` (launch, config, rviz, firmware) | te quedas sin ejecutables y sin launch ni config instalados |
| `setup.cfg` | cuatro líneas: que los ejecutables se instalen en `lib/scara_kinematics`, que es exactamente donde `ros2 run` los busca | se instalan donde no toca y `ros2 run` no los encuentra |
| `resource/scara_kinematics` | un archivo **vacío, de 0 bytes**: la marca en el índice de ament | `ros2 pkg list` no ve el paquete y `colcon build` se para con un error. Parece basura y no lo es |

### El otro paquete: `src/scara_urdf/`

Es el robot **dibujado** (el export de SolidWorks), no el que se mueve. Compila
con `ament_cmake`, no con Python, por eso lleva `CMakeLists.txt` en vez de
`setup.py`. Tiene su propia referencia: [`scara_urdf/URDF.md`](../scara_urdf/URDF.md).

| | qué es | si lo borras |
|---|---|---|
| `urdf/` | `scara_urdf.urdf` (las 4 juntas, tal cual salió de SolidWorks) y `scara_urdf_gz.urdf`, el mismo con los plugins de gz | no hay robot que dibujar: se caen RViz, Gazebo y `robot_state_publisher` |
| `meshes/` | los `.STL` de cada eslabón: Alvin, Simon, Teodoro, Ian y Dave | el robot sale invisible; la cinemática sigue bien |
| `launch/` | `display.launch.py` (RViz con sliders, sin nodos) y `gazebo.launch.py` (gz sim solo) | pierdes esos dos atajos; los launch de `scara_kinematics` siguen funcionando |
| `rviz/`, `config/` | vistas de RViz y los nombres de junta del export | vistas por defecto, hay que recolocarlas |
| `env-hooks/` | una línea que añade `share` al `GZ_SIM_RESOURCE_PATH` | Gazebo no encuentra las mallas y el robot sale sin cuerpo |
| `CMakeLists.txt`, `package.xml` | lo que exige `ament_cmake` | no compila |

---

## 3. Arranque

```bash
cd ~/ros2_ws
colcon build --packages-select scara_kinematics scara_urdf
source install/setup.bash

ros2 launch scara_kinematics brazo.launch.py      # los 6 nodos + la ventana
ros2 launch scara_kinematics rviz.launch.py       # lo anterior + RViz
ros2 launch scara_kinematics gazebo.launch.py     # lo anterior + Gazebo de espejo
```

Los argumentos que más se usan (la lista completa, en la
[sección 10](#10-los-launch)):

```bash
ros2 launch scara_kinematics brazo.launch.py port:=/dev/ttyUSB0
ros2 launch scara_kinematics brazo.launch.py hw:=false        # sin placa
ros2 launch scara_kinematics brazo.launch.py ventana:=false   # sin la interfaz
ros2 launch scara_kinematics brazo.launch.py registro:=~/scara.csv
ros2 launch scara_kinematics gazebo.launch.py brazo:=false    # simulación sola
ros2 launch scara_kinematics gazebo.launch.py gui:=false      # gz sin ventana
```

O con el atajo, que además compila y sabe parar lo que dejó abierto:

```bash
./abrir.sh              # gazebo + rviz + los 6 nodos + la ventana
./abrir.sh rviz         # RViz, sin física
./abrir.sh brazo        # solo los 6 nodos, sin dibujo
./abrir.sh -sh rviz     # sin la placa (no abre el puerto serie)
./abrir.sh -sv gazebo   # sin la ventana del brazo
./abrir.sh -b           # en segundo plano: espera a los 6 nodos y, si falta alguno, dice cuál
./abrir.sh parar        # cierra todo, incluidos los gz huérfanos
./abrir.sh estado       # qué corre ahora y dónde está la punta
```

Un nodo suelto, para depurar:

```bash
P=install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
ros2 run scara_kinematics cc --ros-args --params-file $P
```

> **Todos los nodos necesitan el `--params-file`.** Sin él arrancan con los
> `DEFAULT_PARAMS` de `model.py`, que son de otro robot (l1 = 0.165 m en vez de
> 0.25, y las juntas se llaman `Simon_joint`…). No fallan: mienten.

---

## 4. Los nodos, uno a uno

Todos declaran, además de lo suyo, **los parámetros del modelo**
([sección 8](#8-los-parámetros-del-modelo)), porque todos llaman a
`declare_model_params()`.

---

### 1. `cc` — cinemática directa

`scara_kinematics/cc.py` · ejecutable `cc` · nodo `/cc`

Recibe las tres juntas de Alvin, Simón y Teodoro, arma el brazo entero y resuelve
dónde está la punta. Es el único de los seis que recibe **y** envía.

**Recibe**

| topic | tipo | qué |
|---|---|---|
| `scara/alvin/junta` | `sensor_msgs/JointState` | `juntura1`, una sola junta |
| `scara/simon/junta` | `sensor_msgs/JointState` | `juntura2` |
| `scara/teodoro/junta` | `sensor_msgs/JointState` | `vertiacal1` |

Se suscribe por motor, no a un topic común: el índice del vector se cierra en el
callback, así que cada mensaje sabe qué hueco rellena. Y junta **por muestra**:
guarda las piezas por su etiqueta (`muestra=k`) y solo cuando tiene las tres de
la misma `k` arma el vector y publica. Si una pieza no trae etiqueta (la sonda
del auditor), junta por el sello.

**Envía**

| topic | tipo | frecuencia | qué |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | una por muestra (25 Hz) | el brazo entero: posición, velocidad y esfuerzo de las tres, con el sello y la etiqueta de la muestra |
| `scara/ee_pose` | `geometry_msgs/PoseStamped` | una por muestra | la punta (TCP) en el marco `world`, con el mismo sello |
| `scara/ee_path` | `nav_msgs/Path` | una por muestra | el rastro, hasta `trail_len` puntos |
| `scara/ee_marker` | `visualization_msgs/Marker` | una por muestra | esfera verde de 2 cm en la punta, para RViz |
| `<cmd_prefix><junta>` | `std_msgs/Float64` | una por muestra | **solo con `publicar_gz:=true`**: el espejo en Gazebo |

`joint_states` es un nombre relativo a propósito: se remapea por launch.

**Parámetros propios**

| parámetro | tipo | defecto | qué |
|---|---|---|---|
| `base_frame` | string | `world` | marco de la pose y del rastro |
| `trail_len` | int | `500` | puntos del rastro antes de tirar los viejos |
| `singular_threshold` | double | `5e-4` | por debajo de esta manipulabilidad, avisa de singularidad |
| `t_junta` | double | `1.0` | segundos sin noticias de una junta antes de quejarse |
| `publicar_gz` | bool | `false` | reflejar las juntas en los controladores de gz |
| `cmd_prefix` | string | `/scara_urdf/cmd/` | raíz de los topics de mando de gz |

**Qué hace por dentro**

- **Publica una vez por muestra, y solo con sus tres piezas.** Una junta sin
  motor (`manita` en el YAML de 4 GDL) va fija a `0.0`, porque no va a llegar
  nunca. Las piezas a medio juntar se guardan como mucho 25 muestras (un
  segundo); una muestra a la que le falta una pieza no se publica nunca.
- La FK es la fórmula cerrada de `model.py`, no la cadena del URDF:
  `x = l1·cos(q1+b1) + l2·cos(q1+q2+b2)`, y análogo para `y`;
  `z = z_ref + z_dir·d3`; `yaw = q1 + q2 + yaw_offset`. **Medido** sobre este
  brazo con 3000 poses al azar, se aparta de la cadena completa del URDF como
  mucho **0.002 mm**; el test lo acota en 0.5 mm, que deja margen de sobra.
- La orientación se publica como cuaternión solo con `z` y `w`: el SCARA gira
  únicamente sobre el eje vertical.

**De qué avisa**

| aviso | cuándo |
|---|---|
| `sin noticias de <motor>` (WARN, cada 10 s) | esa junta no ha llegado ni una vez |
| `<motor> lleva N s callado` (WARN, cada 10 s) | llegó y dejó de llegar hace más de `t_junta`: no se completa ninguna muestra |
| `N muestra(s) incompleta(s)… piezas que faltaron: <motor> M` (WARN, cada 10 s como mucho) | muestras que no se publicaron porque les faltó la pieza de ese motor |
| `el brazo está en una singularidad` (WARN, una vez por entrada) | estirado o plegado del todo: ahí un milímetro de punta pide muchísimo giro de codo |

**Si un motor se calla, el brazo se queda dibujado donde estaba**, y es
deliberado: seguir dibujándolo con la última posición de ese motor sería mezclar
dos instantes y hacer pasar por medida algo que ya no lo es. Por eso no publica
y además se queja, con el nombre del que falta. (Hasta el 28/09/2026 publicaba a
50 Hz con lo último que tuviera de cada junta.)

---

### 2, 3 y 4. `alvin`, `simon`, `teodoro` — un nodo por motor

`scara_kinematics/{alvin,simon,teodoro}.py` · ejecutables del mismo nombre

Cada fichero lleva **su nodo entero** (las clases `Alvin`, `Simon` y `Teodoro`):
hasta el 28/09/2026 eran de cuatro líneas y el cuerpo estaba en `motor.py`, que
se borró. Hacen lo mismo con distinta junta; lo que cambia en cada uno son sus
unidades (grados o mm), sus márgenes y lo que dice cuando algo falla. La junta y
los pines los sacan de la tabla `MOTORES` de `protocolo.py`, que es la misma que
leen la aduana, `cc` y la ventana: un cambio de cableado se hace en un sitio.

| nodo | motor | junta | qué mueve | tipo |
|---|---|---|---|---|
| `alvin` | Alvin | `juntura1` | hombro | giro (rad → grados) |
| `simon` | Simón | `juntura2` | codo | giro (rad → grados) |
| `teodoro` | Teodoro | `vertiacal1` | husillo | lineal (m → mm) |

**Recibe**

| topic | tipo | qué |
|---|---|---|
| `scara/<motor>/aduana` | `sensor_msgs/JointState` | su junta, medida, tal y como la repartió la aduana |

**Envía**

| topic | tipo | QoS | qué |
|---|---|---|---|
| `scara/<motor>/junta` | `sensor_msgs/JointState` | normal | su junta, republicada para `cc` |
| `scara/<motor>/valor` | `std_msgs/Float64` | normal | la posición en unidades de taller: **grados**, o **mm** en Teodoro |
| `scara/<motor>/tope` | `std_msgs/Bool` | normal | `true` si está a menos de `margen_tope` de un extremo |
| `scara/<motor>/atascado` | `std_msgs/Bool` | **enganchado** | `true` si recibe PWM y no se mueve |

`atascado` va con QoS *transient local* (profundidad 1, fiable) porque solo se
publica cuando cambia: quien se suscriba tarde —la terminal, un `ros2 topic
echo`— tiene que ver el estado actual y no quedarse esperando a la siguiente
transición.

**Parámetros propios**

| parámetro | tipo | defecto giro | defecto Teodoro | qué |
|---|---|---|---|---|
| `margen_tope` | double | `0.0087` (0.5°) | `0.0005` (0.5 mm) | margen para dar la junta por topada |
| `u_minimo` | double | `0.15` | `0.15` | por debajo de este PWM no se le puede exigir que se mueva |
| `t_mudo` | double | `1.0` | `1.0` | segundos empujando sin moverse antes de cantarlo |
| `eps` | double | `0.0035` (0.2°) | `0.0002` (0.2 mm) | movimiento por debajo del cual se considera quieto |

**Lo que hace con cada pieza que le llega**, en este orden:

1. Comprueba que es suya y que viene entera: su junta, un valor, finito. Si no,
   la tira.
2. Comprueba que va en orden por el número de muestra: si falta alguna la cuenta
   (se perdió entre la aduana y él), y si llega repetida la tira, porque `cc` la
   juntaría dos veces.
3. Se la pasa a `cc` por `scara/<motor>/junta` **sin tocar un solo número**, con
   el mismo sello y la misma etiqueta.
4. La publica en unidades de taller, mira el tope y vigila el encoder.

**El vigilante de encoder**

Es la razón de peso para tener un nodo por motor. La lógica:

1. Si `|esfuerzo| < u_minimo` → sin empuje no se concluye nada. Reinicia la
   cuenta y baja `atascado` si estaba alto.
2. Si la posición se ha movido más de `eps` → reinicia la cuenta y baja
   `atascado`.
3. Si lleva más de `t_mudo` segundos empujando y quieto → sube `atascado` y
   escribe un ERROR que dice **por dónde mirar, en orden**: los 12 V del L298N,
   el encoder en sus pines, y que el eje no esté topado.

El orden no es casual. En este brazo ha pasado varias veces que un motor recibía
PWM y no contaba —los 12 V desenchufados, un canal de encoder muerto, el freno
por arriba bloqueando el puente— y desde fuera solo se veía «el brazo no va», sin
saber cuál de los tres fallaba. **El LED del L298N no demuestra que haya 12 V:**
se enciende con los 5 V del USB a través de la ESP32.

**De qué avisan**

| aviso | cuándo |
|---|---|
| `<Motor> en el tope (valor)` (WARN, cada 5 s) | a menos de `margen_tope` de un extremo |
| `me faltan N muestra(s) entre la A y la B` (WARN, cada 5 s) | se perdieron entre la aduana y el motor: ¿máquina sobrecargada? |
| `la muestra k me ha llegado dos veces` (WARN, cada 5 s) | la tira: `cc` la juntaría dos veces |
| `me llega una pieza que no es mía o no viene entera` (WARN, cada 5 s) | alguien más publica en `scara/<motor>/aduana` |
| `<Motor> no se mueve con N % de PWM durante T s` (ERROR) | el vigilante, con la lista de dónde mirar |
| banner al arrancar (INFO) | junta, índice, recorrido y pines: `IN 19/18 ENA 5 enc 12 (un solo canal)` |

Si `motor.junta` no está en `joint_names`, el nodo **sale con error** en vez de
arrancar callado: casi siempre significa que falta el `--params-file`.

---

### 5. `esp` — la aduana

`scara_kinematics/esp.py` · ejecutable `esp` · nodo `/esp`

Todo lo que entra o sale de la placa pasa por aquí, de ahí el nombre. Abre el
puerto, lo reconecta solo, y lo que llega no lo publica de golpe: lo reparte
junta a junta, un topic por motor.

**Recibe**

| topic | tipo | qué |
|---|---|---|
| `scara/hw/cmd` | `sensor_msgs/JointState` | consigna de posición; se traduce a la orden `T` y se manda a `rate` Hz |
| `scara/hw/raw` | `std_msgs/String` | una orden del protocolo, tal cual. Es la vía por la que la ventana llega a la placa |

**Envía**

| topic | tipo | qué |
|---|---|---|
| `scara/alvin/aduana` | `sensor_msgs/JointState` | `juntura1` medida: posición, velocidad y esfuerzo, con la etiqueta de la muestra en `frame_id` |
| `scara/simon/aduana` | `sensor_msgs/JointState` | `juntura2` medida |
| `scara/teodoro/aduana` | `sensor_msgs/JointState` | `vertiacal1` medida |
| `scara/hw/ok` | `std_msgs/Bool` | 4 Hz. Enlace vivo **y** con cero **y** habilitado |
| `scara/hw/flags` | `std_msgs/UInt8` | 4 Hz. La palabra de estado del firmware; `0` si no hay enlace |
| `scara/hw/log` | `std_msgs/String` | lo que dice la placa: líneas `!` y `#` |

**Servicios**

| servicio | tipo | qué hace |
|---|---|---|
| `scara/hw/zero` | `std_srvs/Trigger` | manda `Z`: la pose actual pasa a ser el cero |
| `scara/hw/enable` | `std_srvs/SetBool` | manda `E 0/1`: quita o da potencia |

`enable` **rechaza dar potencia sin cero**, con el motivo escrito en la
respuesta. El firmware también lo rechaza, pero conviene explicarlo aquí: con
encoders incrementales, habilitar sin cero es mover el brazo hacia una posición
que la placa cree conocer y no conoce.

**Parámetros propios**

| parámetro | tipo | defecto | qué |
|---|---|---|---|
| `port` | string | `''` | puerto de la ESP32. Vacío = buscarlo por el VID del conversor |
| `baud` | int | `115200` | baudios. **No subir**: el CH340 de esta placa es ruidoso |
| `rate` | double | `50.0` | Hz a los que se manda la `T` |
| `telemetria_hz` | double | `25.0` | Hz que se le piden a la placa (orden `F`) |
| `max_vel_hw` | double[] | `[1.5, 1.5, 0.008, 2.0]` | techo real del hardware, en rad/s y m/s |

`max_vel_hw` no es el `max_vel` del YAML. El del YAML está pensado para la
simulación, donde el husillo puede ir a 0.10 m/s; con un JGA25-370 de 60 RPM
sobre un husillo de 8 mm el máximo real son **0.008 m/s**. Si no se corrige, la
consigna se adelanta al brazo y lo que se dibuja miente.

**Cómo busca el puerto**

Sin `port`, coge el primer puerto cuyo conversor USB-serie tenga un VID conocido:
`0x10C4` (CP210x), `0x1A86` (**CH34x, el de esta placa**), `0x0403` (FTDI),
`0x303A` (Espressif).

**Qué hace al conectar**, en este orden:

1. Pone DTR y RTS a `false` **antes** de abrir, para intentar no disparar el
   auto-reset de la placa. Es un intento, no una garantía: **medido** con el
   CH340 de esta placa, el driver levanta las dos líneas igualmente y la ESP32 se
   reinicia, perdiendo el cero. Cada reconexión obliga a referenciar otra vez.
   Visto de otro modo no es mala noticia: un corte de USB deja la placa en un
   estado seguro que exige confirmar la referencia a mano, en vez de seguir
   moviéndose creyendo un cero que ya no vale.
2. Espera 0.3 s y vacía el buffer de entrada.
3. Manda `R j qmin qmax vmax` por cada junta: los límites del YAML pisan a los
   que la placa trae compilados.
4. Manda `F <telemetria_hz>`.
5. Manda `?` para que vuelque su configuración al log.

**Hilo de lectura**: un hilo aparte hace `readline()` y reparte según el primer
carácter (`>` telemetría, `!` aviso, `#` traza). Si el enlace se cae, reconecta
solo cada 2 s. Al salir manda `E 0`: si el nodo muere y los motores se quedan
dando corriente, nadie va a estar mirando el brazo.

**Cómo reparte la telemetría**

La línea `> ms q1 q2 q3 u1 u2 u3 flags*CS` se comprueba antes de nada: si el
checksum no cuadra, si no trae, si le sobran o le faltan campos, o si algún
valor no es un número finito, **se tira entera**. Una línea rota no se puede
arreglar, y lo que no hay que hacer es dejar que un número falso entre en la
cadena como si fuera una medida (hasta el 28/09/2026 el checksum no se miraba).
La misma medida dos veces (mismo `ms`) también se tira.

Luego se le pone número (`muestra=k`, 1, 2, 3… sin huecos) y se corta en tres
`JointState` de una junta cada uno, con el mismo sello y la misma etiqueta
`muestra=k placa_ms=ms cero=N` en `header.frame_id`. `cero=N` cuenta los ceros
que ha confirmado la placa (su «! cero establecido») desde que arrancó la
aduana: al fijar el cero la lectura salta a 0 sin que el brazo se mueva, así
que entre dos muestras con distinto `cero` no se calcula velocidad (sale 0) y el
auditor no lo cuenta como salto imposible. El sello siempre crece, aunque el
reloj no lo haga (con `use_sim_time` vale 0 hasta que arranca Gazebo). El reparto se indexa por la **posición de la junta en `joint_names`**,
no por el orden de la tabla de motores, para que el `manita` del YAML de 4 GDL
—que no tiene motor— no descoloque a los otros tres. La velocidad no la manda el
firmware: se deriva aquí por diferencias, **con el reloj de la placa** (`ms`), no
con la hora de llegada. El USB entrega a ráfagas, y dos líneas pegadas daban un
`dt` de casi cero y velocidades de decenas de rad/s.

El reloj de la placa sirve también para contar lo que no llega: si entre dos
muestras pasa más de 1.5 periodos, faltan muestras. El periodo lo saca de lo que
contesta la placa a la `F` (`! telemetria 25.0 Hz`), así que sigue valiendo
aunque alguien cambie la frecuencia a mano. Y si el reloj de la placa va hacia
atrás, es que la placa se ha reiniciado. El `effort` es en realidad **el ciclo de trabajo con signo
(−1…1)**: no son newtons, pero en un motor de continua es lo más parecido al par
que hay sin medir corriente.

**De qué avisa**

| aviso | cuándo |
|---|---|
| `no se pudo abrir <puerto>` (WARN, cada 10 s) | el puerto no existe o está ocupado |
| `enlace perdido, reintentando` (WARN) | falló un `readline` con el nodo en marcha (al cerrarlo con Ctrl+C ya no sale: el puerto lo cierra el propio nodo) |
| `enlace con la placa: N línea(s) tirada(s) (…) y M muestra(s) que nunca llegaron` (WARN, cada 10 s) | líneas rotas (checksum), repetidas, y huecos en el reloj de la placa. Si crece deprisa: el cable USB, y no subir los baudios |
| `la placa se ha reiniciado (su reloj ha vuelto de A a B ms)` (WARN) | ha perdido el cero: hay que volver a fijarlo |
| `scara/hw/cmd trae un valor no finito` (ERROR, cada 5 s) | un `nan` no se manda: el PID perseguiría un número que no existe |
| `alguna junta está en su tope` (WARN, cada 5 s) | bit `F_LIMITE` |
| `el PID está saturando` (WARN, cada 5 s) | bit `F_SATURACION` |
| `la placa cortó la potencia por bloqueo` (ERROR, cada 5 s) | bit `F_FALLO`. La placa ya cortó por su cuenta |

---

### 6. `esp_rec` — el receptor

`scara_kinematics/esp_rec.py` · ejecutable `esp_rec` · nodo `/esp_rec`

El final de la cadena: recoge lo que saca `cc`.

**Recibe**

| topic | tipo | qué |
|---|---|---|
| `scara/ee_pose` | `geometry_msgs/PoseStamped` | la punta, de `cc` |
| `joint_states` | `sensor_msgs/JointState` | las tres juntas, de `cc` |

**Envía**

| topic | tipo | qué |
|---|---|---|
| `scara/esp_rec/linea` | `std_msgs/String` | a `hz` Hz: la punta en texto legible, con las tres juntas en grados y mm |
| `scara/hw/cmd` | `sensor_msgs/JointState` | **solo con `reenviar:=true`**: la consigna de vuelta a la aduana |

Ejemplo de `scara/esp_rec/linea`:

```
TCP  x   462.2  y    17.1  z   155.5 mm  yaw   -8.6°  |  Alvin +11.5°  Simón -20.1°  Teodoro 90.0 mm
```

**Parámetros propios**

| parámetro | tipo | defecto | qué |
|---|---|---|---|
| `reenviar` | bool | `false` | devolver la consigna a la placa a través de la aduana |
| `hz` | double | `2.0` | cada cuánto resume |
| `registro` | string | `''` | ruta de un CSV donde anotar. Admite `~` |

**El CSV**, si se le da `registro`:

```
t,x,y,z,yaw,juntura1,juntura2,vertiacal1
```

Con `t` en segundos de la marca de tiempo de la pose, posiciones en metros y
ángulos en radianes. La punta y las juntas llegan por dos topics, pero `cc` las
saca de la misma muestra con el mismo sello: se emparejan por ese sello, y si no
se encuentra la pareja la fila se salta antes que mezclar dos instantes. Se abre con `buffering=1` (línea a línea), así que se puede
leer mientras corre.

**Por qué `reenviar` va apagado por defecto**

El firmware **no tiene ninguna orden para «la punta está en (x, y, z)»**: su
protocolo es `T/M/A/I/E/Z/H/K/C/D/R/P/F/W/?` y nada más. Lo único que tiene
sentido devolverle es una `T`, la consigna de posición. Como `cc` mide lo que ya
hay, esa `T` es un *quédate donde estás*: útil para congelar el brazo y para ver
el lazo entero cerrándose. Pero mandar consignas a un brazo con potencia no
debería pasar solo por arrancar un nodo.

Va por `scara/hw/cmd` y no escrita a mano por `scara/hw/raw`: la aduana ya manda
la `T` como un flujo a 50 Hz con la última consigna que le llegó, y una segunda
`T` colada por `raw` a 2 Hz haría que la placa recibiera **dos objetivos
alternos** (y a 2 Hz, además, justo en el límite de los 500 ms del firmware).
Así hay una sola `T`, la de la aduana, y `esp_rec` solo cambia a dónde apunta.
Medido: 50 Hz a la placa, una consigna nueva cada 0.5 s, todas muestras reales.

**Y por qué no abre el puerto**: `/dev/ttyUSB0` solo lo puede tener abierto un
proceso, y ese es la aduana. Si hay que mandarle algo a la placa, se le pasa a
ella y ella lo escribe.

**De qué avisa**

| aviso | cuándo |
|---|---|
| `cc no está publicando` (WARN, cada 10 s) | no ha llegado ninguna pose |
| `no puedo escribir en <ruta>` (ERROR) | el CSV no se pudo abrir |

---

### La interfaz: `brazo_hw`

`scara_kinematics/brazo_hw.py` · ejecutable `brazo_hw` · nodo `/brazo_hw`

La única ventana del paquete (Tk). No calcula nada: mira topics y pide cero y
potencia por servicio. Necesita `DISPLAY`; si no lo hay, lo dice y sale.

**Recibe**

| topic | tipo | columna |
|---|---|---|
| `scara/hw/cmd` | `sensor_msgs/JointState` | **Consigna** — lo que la aduana escribe en la placa |
| `gz/joint_states` | `sensor_msgs/JointState` | **Gazebo** — la simulación, si corre |
| `joint_states` | `sensor_msgs/JointState` | **Real** — lo que arma `cc` con los encoders |
| `scara/hw/flags` | `std_msgs/UInt8` | la barra de estado y los avisos |
| `scara/hw/log` | `std_msgs/String` | el registro de abajo |

**Envía**: `scara/hw/raw` (órdenes a mano y las pruebas de motor) y
`scara/hw/cmd` (al sincronizar antes de dar potencia).
**Llama a**: `scara/hw/zero` y `scara/hw/enable`.

**Los mandos**

| mando | qué hace | cuándo está activo |
|---|---|---|
| **Fijar cero** | `scara/hw/zero`. Con el brazo a mano en su reposo | con enlace y sin potencia |
| **Dar potencia** | sincroniza la consigna y luego `enable(true)` | con enlace y con cero |
| **Cortar potencia** / **PARAR** / `Esc` | `enable(false)` | con enlace |
| **◀ ▶ Probar motor** | `A <j> <±u> <ms>`: empujón en lazo abierto | con enlace y **sin** potencia |
| **Enviar** | teclea cualquier orden del protocolo | con enlace |

**Al dar potencia, la consigna salta primero a donde está el brazo real** (se
publica en `scara/hw/cmd` y 150 ms después se habilita). Si no, en cuanto el
firmware cerrase el lazo, el brazo iría de golpe hacia donde estuviera la
consigna vieja.

Al cerrar la ventana manda `E 0`: sin la ventana, nadie está mirando el brazo.

---

## 5. Tabla completa de topics

| topic | tipo | lo publica | lo escucha |
|---|---|---|---|
| `scara/alvin/aduana` | `JointState` | `esp` | `alvin` |
| `scara/simon/aduana` | `JointState` | `esp` | `simon` |
| `scara/teodoro/aduana` | `JointState` | `esp` | `teodoro` |
| `scara/alvin/junta` | `JointState` | `alvin` | `cc` |
| `scara/simon/junta` | `JointState` | `simon` | `cc` |
| `scara/teodoro/junta` | `JointState` | `teodoro` | `cc` |
| `scara/<motor>/valor` | `Float64` | cada motor | nadie (para mirar) |
| `scara/<motor>/tope` | `Bool` | cada motor | nadie (para mirar) |
| `scara/<motor>/atascado` | `Bool` *enganchado* | cada motor | nadie (para mirar) |
| `joint_states` | `JointState` | `cc` | `robot_state_publisher`, `esp_rec`, `brazo_hw` |
| `scara/ee_pose` | `PoseStamped` | `cc` | `esp_rec`, RViz |
| `scara/ee_path` | `Path` | `cc` | RViz |
| `scara/ee_marker` | `Marker` | `cc` | RViz |
| `/scara_urdf/cmd/<junta>` | `Float64` | `cc` (con `publicar_gz`) | el puente de gz |
| `gz/joint_states` | `JointState` | el puente de gz | `brazo_hw` |
| `scara/hw/cmd` | `JointState` | `brazo_hw`, `esp_rec` (con `reenviar`), `tray` (con `destino:=placa`) | `esp` |
| `scara/hw/raw` | `String` | `brazo_hw` | `esp` |
| `scara/hw/ok` | `Bool` | `esp` | nadie (para mirar) |
| `scara/hw/flags` | `UInt8` | `esp` | `brazo_hw` |
| `scara/hw/log` | `String` | `esp` | `brazo_hw` |
| `scara/esp_rec/linea` | `String` | `esp_rec` | nadie (para mirar) |

**Servicios**: `scara/hw/zero` (`Trigger`) y `scara/hw/enable` (`SetBool`), los
dos de `esp`.

---

## 6. El protocolo con la ESP32

Líneas de texto terminadas en `\n`, con checksum XOR **opcional** en `*XX`
hexadecimal. Es opcional a propósito: así se puede teclear `E 1` o `?` a mano en
el monitor serie del IDE para depurar sin calcular nada.

### PC → ESP32

| orden | qué hace |
|---|---|
| `T q1 q2 d3` | consigna de junta (rad, rad, m) |
| `M j q` | lleva **solo** la junta `j` a `q` (rad, o m si `j = 2`) |
| `A j u ms` | prueba en lazo abierto: ciclo `u` (−1…1) durante `ms` |
| `I j +1\|-1` | invierte el motor por software (como cruzar sus cables) |
| `E 0\|1` | etapa de potencia (`0` = frena y abre el lazo) |
| `Z` | toma la pose actual como cero |
| `H` | vuelve a la pose de referencia, por rampa |
| `K j kp ki kd kv` | ganancias de la junta `j` |
| `C j cuentas` | cuentas de encoder por rad (por metro si `j = 2`) |
| `D j sentido` | `+1` / `-1`, invierte el encoder de la junta `j` |
| `R j qmin qmax vmax` | recorrido y velocidad máxima |
| `P j pwmmin pwmmax` | PWM de arranque y tope, 0…1 |
| `F hz` | frecuencia de telemetría (`0` = apagada) |
| `W hz` | frecuencia del PWM de los motores (con `E 0`) |
| `?` | vuelca toda la configuración |

De estas, la aduana usa `T`, `E`, `Z`, `R`, `F` y `?` por su cuenta. **Las demás
se mandan a mano** desde la caja de texto de `brazo_hw` o publicando en
`scara/hw/raw`.

> `T` es un **flujo**: se manda a 50 Hz y, si deja de llegar 500 ms, la
> referencia se congela. `M` y `H` son **metas**: se teclean una vez y la junta
> llega aunque no llegue nada más. La siguiente `T` vuelve a mandar.

### ESP32 → PC

| línea | qué es |
|---|---|
| `> ms q1 q2 q3 u1 u2 u3 flags*CS` | telemetría. `ms` es el reloj de la placa desde que arrancó. **Siempre** trae checksum, y la aduana tira la línea si no cuadra |
| `! texto` | confirmación de un comando |
| `# texto` | mensaje informativo |

### La palabra de estado

| bit | constante | significa |
|---|---|---|
| `0x01` | `F_HABILITADO` | hay potencia |
| `0x02` | `F_ENLACE` | la placa recibe órdenes |
| `0x04` | `F_CERO` | hay referencia: se puede creer la posición |
| `0x08` | `F_LIMITE` | alguna junta está en su tope |
| `0x10` | `F_SATURACION` | el PID está saturando |
| `0x20` | `F_FALLO` | la placa cortó la potencia por bloqueo |

Están en `protocolo.py` y en el `.ino`; si se toca uno hay que tocar el otro.

---

## 7. Los motores y el cableado

El mapeo es **motor = eslabón que lo aloja**, no el que mueve. Por eso el orden
no es el que parece.

| nodo | motor | junta | mueve | IN1 | IN2 | ENA | encoder |
|---|---|---|---|---|---|---|---|
| `alvin` | Alvin | `juntura1` | hombro | 16 | 4 | 17 | 26 / 25 |
| `simon` | Simón | `juntura2` | codo | 19 | 18 | 5 | **12, un solo canal** |
| `teodoro` | Teodoro | `vertiacal1` | husillo | 22 | 21 | 23 | 14 / 27 |

Son los pines del firmware (`PINES_J1..J3` de `scara_esp32.ino`), no los del
esquema teórico.

**Simón va con un canal.** Su GPIO 13 no cuenta nunca, así que el firmware lleva
`SIN_PIN` en el canal A y solo mira el 12: cada flanco es un paso y el sentido lo
pone el último empuje del motor. Media resolución y algo de deriva en cada cambio
de sentido. Poner «12/13» en la tabla mandaba a mirar un pin que el firmware ni
lee.

**GPIO 12 es strapping (MTDI)** y fija en el arranque la tensión de la flash. En
esta placa está **neutralizado por eFuse**: se quemó `XPD_SDIO` a 3.3 V, es
irreversible, y por eso arranca aunque la ROM siga diciendo `boot:0x33`.

**Topología L298N clásico**: dos pines de dirección y un PWM de *enable*.

```
u > 0  →  IN1=1, IN2=0, ENA=duty
u < 0  →  IN1=0, IN2=1, ENA=duty
freno  →  IN1=IN2=0, ENA=máximo      (motor en corto por los transistores de abajo)
```

El freno es **por abajo**, y eso importa: frenar por arriba (`IN1=IN2=1`,
bornes a +12 V) llegó a bloquear los demás motores. PWM a 1 kHz: el L298N es
lento y a 20 kHz se calienta sin dar más par.

**Los encoders son de 5 V en la mayoría de módulos y el ESP32 no tolera 5 V.**
Aliméntalos desde 3.3 V o mete un divisor 10k/20k en cada canal.

---

## 8. Los parámetros del modelo

Los declaran **todos** los nodos, vía `declare_model_params()`. Vienen de
`config/scara_urdf3.yaml`, que es el brazo montado: 3 GDL, sin muñeca.

| parámetro | valor | qué |
|---|---|---|
| `joint_names` | `['juntura1', 'juntura2', 'vertiacal1']` | el orden manda: define el índice de todo |
| `l1` / `b1` | `0.2500032` / `0.2268643` | módulo y argumento del vector hombro→codo |
| `l2` / `b2` | `0.2499997` / `-0.2029018` | ídem codo→husillo |
| `z_ref` / `z_dir` | `0.06551` / `1.0` | `z = z_ref + z_dir · d3` |
| `q1_min` / `q1_max` | `-2.0944` / `2.0944` | ±120° |
| `q2_min` / `q2_max` | `-2.618` / `2.618` | ±150° |
| `d3_min` / `d3_max` | `0.02` / `0.17` | recorrido del husillo, en metros |
| `has_wrist` | `false` | sin muñeca: el yaw lo fija el brazo, no es libre |
| `max_vel` | `[1.5, 1.5, 0.10]` | de la **simulación**. El techo real lo pone `max_vel_hw` |
| `base_frame` | `world` | |
| `cmd_prefix` | `/scara_urdf/cmd/` | raíz de los mandos de gz |

`b1` y `b2` no son cero porque los dos SCARA salen de SolidWorks y sus ejes 1 y 2
no arrancan alineados con el eje X de la base: cada junta trae un giro fijo en su
`<origin>`, y el codo además está desplazado lateralmente. Por eso el modelo no
es el 2R de libro.

Existe también `config/scara_urdf.yaml`, el mismo robot con `has_wrist: true` y
la junta `manita`. Los nodos lo aguantan: `manita` no tiene motor, así que `cc`
la da por `0.0` y la aduana no la reparte.

---

## 9. Las bibliotecas

No son nodos: no tienen `main`.

| fichero | qué es |
|---|---|
| `model.py` | el modelo cinemático: `fk`, `ik`, `ik_all`, `jacobian`, `manipulability`, `reach`, límites. Tiene IK, pero **ningún nodo la usa** |
| `protocolo.py` | los bits de estado, el `checksum`, la búsqueda de puerto, la tabla `MOTORES`, `leer_telemetria()` (la línea de la placa, comprobada y en vectores) y la etiqueta de muestra (`etiqueta()` / `leer_etiqueta()`) |
| `estilo.py` | los colores de la ventana (era la paleta del panel) |
| `urdf_fk.py` | recorre la cadena del URDF. **Solo lo usa el test** |

### El test

```bash
cd ~/ros2_ws && source install/setup.bash
python3 -m pytest src/scara_kinematics/test -q       # 15 pasan
```

`test_kinematics.py` comprueba que `model.fk()` reproduce la cadena real del URDF a menos de 0.5 mm
(de hecho se queda en 0.002 mm), que la IK invierte la FK, las dos ramas de codo, los puntos fuera de alcance, el
jacobiano contra diferencias finitas, la singularidad y el anillo de `reach()`.

`test_protocolo.py` comprueba lo que decide qué entra en la cadena: que una
línea buena sale en vectores, que un byte cambiado, una línea sin checksum, con
campos de más o con un `nan` se tiran, y que la etiqueta de muestra se lee de
vuelta igual.

Es lo que sostiene a `cc`: si la FK fallara, la punta que publica estaría mal y
no habría con qué compararla en el brazo de verdad.

> `colcon test` **no** descubre estos tests (sale «NO TESTS RAN»). Ya pasaba
> antes del reparto y no es cosa del test: colcon acaba llamando al `setup.py
> test` de setuptools en vez de a pytest.

---

## 10. Los launch

### `brazo.launch.py` — los 6 nodos y la ventana

Es la base: los otros dos lo incluyen, así que la cadena es siempre esta.

| argumento | defecto | qué |
|---|---|---|
| `port` | `''` | puerto de la ESP32; vacío = buscarlo solo |
| `hw` | `true` | `false` = sin aduana (sin placa) |
| `ventana` | `true` | la interfaz del brazo |
| `rsp` | `true` | `robot_state_publisher`, para TF y RViz |
| `gz` | `false` | `true` = `cc` refleja las juntas en Gazebo |
| `sim` | `false` | `use_sim_time`; lo pone `gazebo.launch.py` |
| `registro` | `''` | CSV donde `esp_rec` anota |
| `reenviar` | `false` | `esp_rec` devuelve la consigna a la placa (por `scara/hw/cmd`) |

Fija `max_vel_hw = [1.5, 1.5, 0.008]` en la aduana y carga
`config/scara_urdf3.yaml` en los seis.

### `rviz.launch.py` — lo anterior más RViz

`brazo.launch.py` + `rviz2` con `rviz/scara.rviz`. Argumentos: `port`, `hw`,
`ventana`, `registro`, `rvizconfig`.

Lo que se ve es el brazo **de verdad**: `cc` arma `/joint_states` con lo que miden
los encoders, `robot_state_publisher` lo convierte en TF y RViz lo dibuja. Sin
física y sin simulación, así que si el dibujo no cuadra con el brazo, el problema
está en los encoders o en el cero, no en un PID.

La vista trae el suelo, el robot, TF, y los tres topics de `cc`. Los displays del
marcador arrastrable, el objetivo de IK, el espacio de trabajo y el esqueleto se
quitaron con los nodos que los publicaban.

### `gazebo.launch.py` — la simulación de espejo

| argumento | defecto | qué |
|---|---|---|
| `world` | `empty.sdf` | |
| `gui` | `true` | `false` = gz sin ventana (solo servidor) |
| `rviz` | `false` | |
| `brazo` | `true` | `false` = simulación sola, sin los seis nodos |
| `port`, `hw`, `ventana`, `registro` | | se reenvían a `brazo.launch.py` |
| `spawn_delay` | `3.0` | margen para que gz cargue sus sistemas antes de meter el robot |

Con `brazo:=true` (lo normal), `cc` refleja las tres juntas en los controladores
de gz: **el robot simulado hace lo mismo que el de la mesa**, y la diferencia
entre los dos se ve de un vistazo.

**Quién publica `/joint_states`** cambia según el modo, y conviene tenerlo claro:

| modo | `/joint_states` | `gz/joint_states` |
|---|---|---|
| `brazo:=true` | `cc` (el brazo real) | el puente de gz |
| `brazo:=false` | el puente de gz | — |

El `spawn_delay` no es paranoia: `create` solo espera a que gz conteste la lista
de mundos, y el servicio que de verdad hace falta (`/world/<w>/create`, del
sistema `UserCommands`) se anuncia cerca de un segundo después.

El puente se configura por YAML y no por `topic@tipo` porque los topics del
`JointPositionController` llevan un segmento numérico (`.../0/cmd_pos`) que ROS 2
no acepta como nombre.

> En este workspace **no hay `ros2_control` ni Gazebo Classic**: el control de
> juntas en simulación son los plugins nativos de gz
> (`gz-sim-joint-position-controller-system`) puenteados como `std_msgs/Float64`.
> Cualquier receta basada en `ros2_control`, `joint_trajectory_controller` o
> MoveIt necesitaría instalarlos antes.

### `scara_urdf/launch/gazebo.launch.py`

Reenvía a `scara_kinematics/gazebo.launch.py`. Se mantiene porque es el nombre
por el que se lanzaba desde ROS 1.

### `scara_urdf/launch/display.launch.py`

Independiente de todo esto: `robot_state_publisher` +
`joint_state_publisher_gui` (los deslizadores) + RViz. Para mirar el URDF solo,
sin placa ni nodos.

---

## 11. Diagnóstico

**Ver la punta en tiempo real**

```bash
ros2 topic echo /scara/esp_rec/linea
```

**Saber si un motor está fallando**

```bash
ros2 topic echo /scara/alvin/atascado      # está enganchado: contesta al momento
ros2 topic echo /scara/simon/tope
ros2 topic echo /scara/teodoro/valor       # en mm
```

**Ver una muestra y su etiqueta**

```bash
ros2 topic echo /scara/alvin/aduana --field header     # muestra=k placa_ms=… cero=N
ros2 topic hz /joint_states                            # 25 Hz: una por muestra
```

**Ver el estado de la placa**

```bash
ros2 topic echo /scara/hw/flags    # la palabra de bits de la sección 6
ros2 topic echo /scara/hw/ok       # true = vivo + con cero + con potencia
ros2 topic echo /scara/hw/log      # lo que dice el firmware
```

**Hablarle a la placa a mano**

```bash
ros2 topic pub --once /scara/hw/raw std_msgs/msg/String "{data: '?'}"
ros2 topic pub --once /scara/hw/raw std_msgs/msg/String "{data: 'A 2 0.7 300'}"   # Teodoro
ros2 service call /scara/hw/zero std_srvs/srv/Trigger
ros2 service call /scara/hw/enable std_srvs/srv/SetBool "{data: false}"
```

### Síntomas

| lo que ves | mira esto |
|---|---|
| `cc` dice «sin noticias de X» | ese motor no recibe de la aduana: ¿arrancó el nodo? ¿la aduana tiene puerto? |
| `cc` dice «muestras incompletas… faltaron: X» | a esas muestras les faltó la pieza de X: mira los avisos de X |
| RViz con el brazo congelado y `cc` quejándose de X | es a propósito: sin la pieza de X no se dibuja una mezcla de instantes |
| `esp` dice «N líneas tiradas» | llegan líneas rotas por el USB: el cable, y no subir los baudios |
| `esp` dice «la placa se ha reiniciado» | ha perdido el cero: *Fijar cero* otra vez |
| Los tres motores dicen «atascado» | casi seguro los **12 V** del L298N. El LED no demuestra nada, mide el borne |
| Uno solo dice «atascado» | su encoder, o su eje topado |
| «sin cero» en la ventana | pon el brazo en reposo a mano y pulsa *Fijar cero* |
| `enable` contesta «sin cero» | es correcto: no se da potencia sin referencia |
| La ventana no abre | ¿hay `DISPLAY`? |
| Gazebo vacío, solo el suelo | el spawn le ganó la carrera: sube `spawn_delay:=12` |
| gz no publica `/joint_states` | mira si quedó un `gz-sim-main` huérfano de otro launch. `./abrir.sh parar` los barre |
| Todo arranca pero los números no cuadran | ¿pusiste el `--params-file`? |

---

## 12. Límites conocidos

**No hay cinemática inversa.** No estaba en los seis nodos, así que `ik_node`
se borró. `model.py` conserva `ik()` e `ik_all()`, pero **ningún nodo las llama**:
ahora mismo no hay forma de mandar el brazo a un punto (x, y, z). Lo único que
mueve motores desde ROS es la consigna de junta en `scara/hw/cmd` y los empujones
en lazo abierto de la ventana.

**Cada reconexión de USB pierde el cero.** El auto-reset del CH340 se dispara al
abrir el puerto, aunque se bajen DTR y RTS antes. Hay que volver a referenciar.

**El encoder de Simón es de un canal**, así que tiene media resolución y deriva
algo en cada cambio de sentido.

**No mover a Alvin sin revisarlo**: se sospecha que un cable de su motor toca
masa. Frenarlo por arriba llegó a bloquear a los demás.

**`colcon test` no descubre los tests** (ver sección 9).

**`joint_states` va a la frecuencia de la telemetría** (25 Hz), no a 50: sale una
por muestra. Si hace falta más, se sube en la placa (`telemetria_hz` de la
aduana), no repitiendo muestras.

**La etiqueta viaja en `header.frame_id`.** Es un campo que en `JointState` no
usa nadie, pero si algún día un nodo nuevo lo necesitara para otra cosa, habría
que llevar la etiqueta a otro sitio.

### Qué se borró el 14/09/2026

Respaldo previo completo en `~/respaldo_ros2_ws_src_2026-09-14.tar.gz`.

- **El paquete `UrdfR3` entero**, el otro SCARA (3 GDL, RRP).
- De `scara_kinematics`: `ik_node`, `fk_node`, `panel`, `simon_gui`,
  `teodoro_gui`, `esp32_bridge`, `target_marker`, `kin_viz`, `monitor`,
  `analisis`, `goto`, `demo` (`trajectory_demo`) y el antiguo `teodoro`.
- Los launch `hardware.launch.py`, `simon.launch.py`, `teodoro.launch.py` y el
  config `urdfr3.yaml`.
- De `scara_urdf/urdf`: `squashfs-root`, `ros2_ws.zip`, el `.bak` y
  `scara_urdf_corregido.urdf`.

Sobrevivieron la ventana `brazo_hw`, Gazebo y RViz, que era lo que había que
conservar.
