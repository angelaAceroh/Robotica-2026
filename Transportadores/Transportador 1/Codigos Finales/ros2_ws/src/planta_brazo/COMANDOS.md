# PLANTA BRAZO — chuleta de comandos

Terminal nueva: tu `~/.bashrc` ya hace `source /opt/ros/jazzy/setup.bash`,
así que solo falta el workspace.

Hay **dos simuladores** y no se pueden usar a la vez (los dos publican
`/joint_states`):

| | RViz2 | Gazebo |
|---|---|---|
| qué hace | dibuja la pose que le mandas | **simula la física**: gravedad, inercia, pares |
| lo que ves | exactamente la consigna | lo que el controlador **consigue** alcanzar |
| launch | `display.launch.py` | `gazebo.launch.py` |
| para qué | comprobar cinemática, medir el TCP | ver si el brazo aguanta, tiempos reales |

---

## 1. Lanzar en RViz (ligero)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch planta_brazo display.launch.py
```

Abre **RViz2** + el **panel de control** (sliders J1/J2/J3, botones
Trayectoria / Home / Borrar traza, y lectura de brida y TCP en mm).

```bash
ros2 launch planta_brazo display.launch.py demo:=true    # se mueve solo, sin panel
ros2 launch planta_brazo display.launch.py rviz:=false   # sin ventana 3D
```

---

## 2. Lanzar en Gazebo (con física)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch planta_brazo gazebo.launch.py
```

Abre **Gazebo** + el **mismo panel**, pero ahora los sliders mandan
*consignas* y el panel muestra además la posición **real** medida por la
física y el error de seguimiento de cada junta.

```bash
# Gazebo + RViz a la vez (RViz dibuja la traza naranja del TCP)
ros2 launch planta_brazo gazebo.launch.py rviz:=true

# recorre la trayectoria sola, sin panel
ros2 launch planta_brazo gazebo.launch.py demo:=true

# sin panel de sliders (para publicar consignas a mano)
ros2 launch planta_brazo gazebo.launch.py panel:=false

# Gazebo sin ventana: solo el servidor de física
ros2 launch planta_brazo gazebo.launch.py gui:=false

# recorre los 8 puntos de prueba por las dos cinemáticas y saca la tabla
ros2 launch planta_brazo gazebo.launch.py comparar:=true
```

Dentro de la ventana de Gazebo: rueda del ratón para zoom, botón izquierdo
para girar, botón central para desplazar. El botón ▶/⏸ de abajo a la
izquierda pausa la física.

### Las dos cinemáticas en el panel

El panel tiene las dos mitades:

- **arriba, cinemática directa**: mueves los tres sliders (ángulos) y abajo lee
  dónde quedan la brida y la punta de los dedos.
- **abajo, cinemática inversa**: escribes el punto `X Y Z` en mm, eliges la
  postura (*codo arriba*, *codo abajo*, *la más cercana*) y pulsas
  **Ir al punto (inversa)**. Los sliders se colocan solos en la solución y el
  recuadro dice cuántas soluciones tenía ese punto y cuántas caben en los
  límites. **Copiar TCP actual** rellena las casillas con donde está ahora, y
  el desplegable **punto de prueba** carga uno de los ocho puntos del informe.

Si el punto no se puede alcanzar, el aviso lo dice y **no mueve el robot**:
distingue "fuera del alcance" (no existe solución geométrica) de "topa en J2"
(la solución existe pero se sale de los límites articulares).

En RViz aparece además una **esfera verde** en el punto pedido a la inversa, y
el panel muestra la distancia que queda entre la punta de los dedos y esa
esfera.

### Comparar directa contra inversa

```bash
# lo normal: sale la tabla por consola y en ~/comparacion_cinematicas.md y .csv
ros2 launch planta_brazo gazebo.launch.py comparar:=true

# pedirle a la inversa la otra postura, o la más cercana a donde está
ros2 launch planta_brazo gazebo.launch.py comparar:=true gui:=false   # sin ventana
ros2 run planta_brazo comparacion_cinematicas.py --ros-args \
  -p modo:=gazebo -p preferencia:=codo_abajo

# comparar en la brida en vez de en la punta de los dedos
ros2 run planta_brazo comparacion_cinematicas.py --ros-args -p extremo:=brida

# la misma tabla sin física (RViz puro): las dos columnas dan 0.00 mm,
# que es la forma de demostrar que el error de Gazebo es del PID, no de la CI
ros2 run planta_brazo comparacion_cinematicas.py --ros-args -p modo:=rviz

# autoprueba de la CI: 20 000 posturas aleatorias CD -> CI -> CD
python3 ~/ros2_ws/src/planta_brazo/scripts/cinematica.py
```

Parámetros del banco: `t_viaje` (2.5 s), `t_asentar` (3.0 s, lo que tarda el
PID en cerrar contra la gravedad), `t_medir` (0.5 s promediando), `extremo`,
`preferencia` y `salida` (ruta base de los ficheros).

### Mover el robot en Gazebo desde la terminal

En Gazebo **no** se publica `/joint_states` a mano: se mandan consignas de
posición, una por junta, en radianes.

```bash
ros2 topic pub --once /planta_brazo/J1/cmd_pos std_msgs/msg/Float64 "{data: 0.5236}"
ros2 topic pub --once /planta_brazo/J2/cmd_pos std_msgs/msg/Float64 "{data: 0.6109}"
ros2 topic pub --once /planta_brazo/J3/cmd_pos std_msgs/msg/Float64 "{data: -1.0472}"

# volver a home
for j in J1 J2 J3; do
  ros2 topic pub --once /planta_brazo/$j/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
done
```

### Tópicos propios de Gazebo

```bash
ros2 topic echo /joint_states       # posición REAL medida por la física (60 Hz)
ros2 topic echo /joint_states_gz    # la misma, sin remuestrear (1000 Hz)
ros2 topic echo /clock              # tiempo de simulación
ros2 topic hz   /clock              # ~1000 Hz => la sim va a tiempo real
```

---

## 3. Compilar

Solo hace falta si editas el URDF, los scripts, el mundo o el launch.
`~/ros2_ws/install/` sobrevive al reinicio.

```bash
cd ~/ros2_ws
colcon build --packages-select planta_brazo
source install/setup.bash
```

```bash
# recompilar de cero si algo se queda pegado
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select planta_brazo
source install/setup.bash
```

---

## 4. Nodos por separado

```bash
ros2 run planta_brazo gui_articulaciones.py     # panel con sliders (modo RViz)
ros2 run planta_brazo trayectoria_demo.py       # trayectoria como nodo suelto
ros2 run planta_brazo cinematica_directa.py     # cinemática directa + traza
ros2 run planta_brazo remuestreo_juntas.py      # baja /joint_states_gz a 60 Hz

# el panel o la trayectoria en modo Gazebo (mandan consignas, no /joint_states)
ros2 run planta_brazo gui_articulaciones.py --ros-args -p modo:=gazebo
ros2 run planta_brazo trayectoria_demo.py  --ros-args -p modo:=gazebo -p periodo:=3.0

# apagar la traza naranja o la impresión por consola
ros2 run planta_brazo cinematica_directa.py --ros-args -p traza:=false
ros2 run planta_brazo cinematica_directa.py --ros-args -p imprimir:=false
```

---

## 5. Mover el robot desde la terminal (solo RViz)

```bash
# una pose fija (radianes): q = 30°, 35°, -60°
ros2 topic pub -r 30 /joint_states sensor_msgs/msg/JointState \
  "{name: [J1,J2,J3], position: [0.5236, 0.6109, -1.0472]}"

# volver a home
ros2 topic pub -r 30 /joint_states sensor_msgs/msg/JointState \
  "{name: [J1,J2,J3], position: [0.0, 0.0, 0.0]}"

# borrar el rastro naranja (vale en los dos simuladores)
ros2 topic pub --once /traza/limpiar std_msgs/msg/Empty "{}"
```

> Ojo: si el panel está abierto, él también publica `/joint_states` y los dos
> se pelean. Cierra el panel antes de publicar a mano.
> En Gazebo esto no vale: `/joint_states` lo publica la física. Usa `cmd_pos`.

---

## 6. Ver qué está pasando

```bash
ros2 topic list
ros2 node list

ros2 topic echo /joint_states           # ángulos actuales (rad)
ros2 topic echo /tcp_pose               # punta de los dedos del gripper
ros2 topic echo /brida_pose             # fin del Brazo3
ros2 topic hz /joint_states             # frecuencia de publicación

# posiciones directamente del modelo (deben coincidir con los tópicos)
ros2 run tf2_ros tf2_echo world tcp
ros2 run tf2_ros tf2_echo world brida
ros2 run tf2_ros tf2_echo world link3

# árbol de frames a PDF
ros2 run tf2_tools view_frames
```

Del lado de Gazebo (gz transport, no ROS):

```bash
gz topic -l                             # tópicos internos de Gazebo
gz model --list                         # modelos cargados en el mundo
gz model -m planta_brazo --pose         # pose de cada eslabón
```

---

## 7. Revisar el URDF y el mundo

```bash
# generar el URDF final a partir del xacro
xacro ~/ros2_ws/install/planta_brazo/share/planta_brazo/urdf/planta_brazo.urdf.xacro > /tmp/planta_brazo.urdf

# validar la cadena cinemática
check_urdf /tmp/planta_brazo.urdf

# diagrama del árbol de links/joints
urdf_to_graphiz /tmp/planta_brazo.urdf     # genera planta_brazo.pdf

# ver el SDF que Gazebo genera a partir del URDF (mallas, inercias, plugins)
gz sdf -p /tmp/planta_brazo.urdf | less

# validar el mundo
gz sdf -k ~/ros2_ws/install/planta_brazo/share/planta_brazo/worlds/lab_brazo.sdf
```

---

## 8. Cuando algo va raro

Casi siempre es que quedaron dos lanzamientos abiertos peleándose
por `/joint_states`. **Cierra uno antes de abrir el otro.**

```bash
ps -ef | grep -E "rviz2|gz sim|robot_state_publisher|planta_brazo" | grep -v grep

pkill -f planta_brazo          # nodos del paquete
pkill -f rviz2                 # RViz
pkill -f "gz sim"              # Gazebo (servidor y ventana)
pkill -f parameter_bridge      # el puente ROS <-> Gazebo
ros2 daemon stop && ros2 daemon start   # reinicia el descubrimiento
```

Para probar sin tocar una sesión que ya esté corriendo, usa otro dominio:

```bash
ROS_DOMAIN_ID=42 ros2 launch planta_brazo gazebo.launch.py
```

> **Ojo: con Gazebo el dominio no basta.** `ROS_DOMAIN_ID` separa los tópicos
> de ROS, pero **gz-transport no lo respeta**: dos `gz sim` en la misma máquina
> comparten bus, se mezclan los `/clock` y las consignas van a los dos mundos.
> Síntoma típico: el robot aparece en una postura absurda al arrancar y luego
> se queda congelado. Antes de lanzar otro Gazebo, mata el anterior de verdad
> (`pkill -f "gz sim"`) y espera un par de segundos.

**Gazebo abre pero el robot no aparece** — casi siempre es que no encuentra
las mallas. Comprueba la ruta de recursos:

```bash
echo $GZ_SIM_RESOURCE_PATH        # el launch la pone solo; debe acabar en .../install/planta_brazo/share
ls ~/ros2_ws/install/planta_brazo/share/planta_brazo/meshes/
```

**El brazo tiembla o se cae** — son las ganancias del PID o los límites de
par. Están en `urdf/planta_brazo.gazebo.xacro` (`p_gain`, `d_gain`, `i_gain`)
y en `<limit effort= velocity=>` de `urdf/planta_brazo.urdf.xacro`.

**El brazo se queda un poco por debajo de la consigna y no sube más** — es el
integrador saturado, no la P. Sube `i_max` (no `p_gain`): con `i_max` corto el
término integral llega al tope antes de compensar la gravedad y el error se
queda fijo. Se ve clarísimo mandando home con el brazo estirado y mirando J2.

**`executable '...' not found on the libexec directory`** después de añadir un
script nuevo — con `--symlink-install` se hereda el permiso del fuente:

```bash
chmod +x ~/ros2_ws/src/planta_brazo/scripts/*.py
colcon build --symlink-install --packages-select planta_brazo
```

---

## 9. Dónde está cada cosa

```
~/ros2_ws/src/planta_brazo/
├── urdf/
│   ├── planta_brazo.urdf.xacro     <- geometría, ejes y límites articulares
│   └── planta_brazo.gazebo.xacro   <- plugins de Gazebo (PID por junta, estado)
├── worlds/lab_brazo.sdf            <- suelo, luces y cámara de arranque
├── config/parametros.yaml          <- d1, a1, L2, L3, Lg
├── meshes/                         <- STL en metros (BASE, TAPA, BRAZO1, Brazo2, Brazo3, gripper)
├── scripts/
│   ├── gui_articulaciones.py       <- panel: sliders (directa) + ir a punto (inversa)
│   ├── cinematica.py               <- CD y CI en forma cerrada + autoprueba
│   ├── enlace_juntas.py            <- capa común RViz / Gazebo
│   ├── remuestreo_juntas.py        <- baja /joint_states de 1000 a 60 Hz
│   ├── cinematica_directa.py       <- DH, /tcp_pose, /brida_pose, traza
│   ├── comparacion_cinematicas.py  <- banco directa vs inversa -> tabla .md/.csv
│   ├── puntos_prueba.py            <- los 8 puntos de la comparación
│   ├── trayectoria_demo.py         <- trayectoria como nodo suelto
│   └── trayectorias.py             <- los 10 puntos + perfil quíntico
├── launch/
│   ├── display.launch.py           <- RViz
│   └── gazebo.launch.py            <- Gazebo (+ RViz opcional)
├── rviz/planta_brazo.rviz
├── README.md                       <- de dónde salió cada medida
└── COMANDOS.md                     <- este archivo
```

**Editar la trayectoria:** cambia la lista `PUNTOS` en `scripts/trayectorias.py`
(grados, `(J1, J2, J3)`) y recompila. La usan el botón del panel y el nodo
suelto, en los dos simuladores.

**Editar los puntos de la comparación:** la lista `PUNTOS_PRUEBA` de
`scripts/puntos_prueba.py`, en grados. El objetivo cartesiano se calcula solo
con la directa, así que siempre es alcanzable.

**Editar los límites articulares:** `<limit lower= upper=>` en
`urdf/planta_brazo.urdf.xacro` (en radianes) y en `JUNTAS` de
`scripts/gui_articulaciones.py` (en grados).

**Editar el aspecto de Gazebo:** luces, color del suelo y posición inicial de
la cámara están en `worlds/lab_brazo.sdf`.
