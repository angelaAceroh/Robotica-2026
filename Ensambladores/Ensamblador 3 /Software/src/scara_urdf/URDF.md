# El SCARA dibujado — paquete `scara_urdf`

Referencia **única** de `scara_urdf`: el 15 de septiembre de 2026 se fundieron
aquí los dos archivos que había (`README.md`, el paso a paso de RViz, y
`README_ROS2.md`, las notas de la migración desde ROS 1), corrigiendo de paso lo
que ya no era verdad. Está todo: qué hace cada carpeta y qué pasa si la borras,
la cadena de eslabones con sus medidas, los dos URDF y en qué se diferencian, los
plugins de Gazebo y lo que no cuadra con el CAD.

Este paquete **no tiene nodos ni código**: es la descripción del robot. Los seis
nodos viven en `scara_kinematics` y están documentados en
[`NODOS.md`](../scara_kinematics/NODOS.md).

- [1. Qué es este paquete](#1-qué-es-este-paquete)
- [2. El paquete por dentro](#2-el-paquete-por-dentro-qué-hace-cada-cosa-y-qué-pasa-si-la-borras)
- [3. Arranque](#3-arranque)
- [4. La cadena: eslabones y juntas](#4-la-cadena-eslabones-y-juntas)
- [5. Los dos URDF](#5-los-dos-urdf)
- [6. Gazebo: el anclaje y los plugins](#6-gazebo-el-anclaje-y-los-plugins)
- [7. Las vistas de RViz](#7-las-vistas-de-rviz)
- [8. Diagnóstico](#8-diagnóstico)
- [9. Lo que no cuadra con el CAD](#9-lo-que-no-cuadra-con-el-cad)
- [10. Qué se borró](#10-qué-se-borró)

---

## 1. Qué es este paquete

Un paquete **de solo datos**: cinco mallas `.STL`, dos URDF, dos vistas de RViz,
dos launch y la configuración del exportador. No compila nada —no hay `.cpp` ni
`.py` de nodos—, pero **hay que compilarlo igual**: `colcon` es quien copia el
URDF y las mallas a `install/share/scara_urdf/`, que es de donde los leen el
launch, RViz y Gazebo.

Viene del exportador de SolidWorks para ROS 1 (catkin) y se migró a ROS 2
(`ament_cmake`): `package.xml` a formato 3, `CMakeLists.txt` reescrito,
`display.launch` de XML a `display.launch.py`, `rviz` a `rviz2` y los finales de
línea CRLF a LF.

```
world ──fija── Alvin ──juntura1── Simon ──juntura2── Teodoro ──vertiacal1── Dave ──manita── Ian
                base            hombro            codo               husillo          muñeca
```

---

## 2. El paquete por dentro: qué hace cada cosa y qué pasa si la borras

| | qué es | si lo borras |
|---|---|---|
| `urdf/scara_urdf.urdf` | el export de SolidWorks, **intacto**: 4 juntas `continuous`/`prismatic`, `effort=0`. Es el que abre `display.launch.py` | no hay robot que dibujar en RViz |
| `urdf/scara_urdf_gz.urdf` | el mismo robot **preparado para simular**: juntas `revolute` con topes reales, `effort` distinto de cero, anclaje al mundo y los plugins de gz. Es el que usa Gazebo | la simulación se cae: sin anclaje el robot es un cuerpo libre y sin `effort` no ejerce par |
| `meshes/` | los cinco `.STL`, 41 KB en total: `Alvin`, `Simon`, `Teodoro`, `Dave` e `Ian`. El URDF los busca por `package://scara_urdf/meshes/…` | el robot sale invisible (solo ejes); la cinemática sigue bien |
| `launch/display.launch.py` | RViz + `robot_state_publisher` + deslizadores, **sin física y sin nodos** | pierdes el atajo para mirar el modelo suelto |
| `launch/gazebo.launch.py` | un reenvío a `scara_kinematics/gazebo.launch.py`; se mantiene porque es el nombre por el que se lanzaba desde ROS 1 | nada: el de `scara_kinematics` hace lo mismo |
| `rviz/urdf.rviz` | la vista que abre `display.launch.py` por defecto | RViz abre vacío y hay que rehacerla |
| `rviz/scara.rviz` | la otra vista, prácticamente igual (rejilla, robot, TF) | lo mismo |
| `config/joint_names_scara_urdf.yaml` | `controller_joint_names`, una lista que dejó el exportador de ROS 1. **Hoy no la lee nadie** en este workspace | nada |
| `env-hooks/gz_resource_path.dsv.in` | una línea: añade `share` al `GZ_SIM_RESOURCE_PATH` al hacer `source` | Gazebo no resuelve los `package://` y el robot sale sin cuerpo |
| `CMakeLists.txt` | instala `config launch meshes rviz urdf` en `share/scara_urdf/` y engancha el env-hook | no se copia nada a `install/`: el launch no encuentra ni el URDF ni las mallas |
| `package.xml` | el carné: `ament_cmake`, `architecture_independent`, y las dependencias (`robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `ros_gz_sim`, `ros_gz_bridge`, `scara_kinematics`) | colcon no lo reconoce como paquete |
| `URDF.md` | este archivo | nada, es texto |

Este paquete **no lleva `setup.py`, `setup.cfg` ni `resource/`**: eso es de los
paquetes de Python (`ament_python`), como `scara_kinematics`. Aquí el equivalente
es `CMakeLists.txt`.

---

## 3. Arranque

```bash
cd ~/ros2_ws
colcon build --packages-select scara_urdf
source install/setup.bash

ros2 launch scara_urdf display.launch.py        # RViz + deslizadores, sin física
```

Se abren dos ventanas: RViz con el robot y el panel con un deslizador por junta.
Moviendo `juntura1`, `juntura2` y `vertiacal1` se articula el brazo.

| argumento | defecto | qué |
|---|---|---|
| `model` | `share/scara_urdf/urdf/scara_urdf.urdf` | ruta absoluta a otro URDF o `.xacro` |
| `rvizconfig` | `share/scara_urdf/rviz/urdf.rviz` | otra vista |
| `gui` | `true` | `false` = `joint_state_publisher` sin ventana |

Con física, y con los seis nodos del brazo real de espejo:

```bash
ros2 launch scara_urdf gazebo.launch.py                # = scara_kinematics/gazebo.launch.py
ros2 launch scara_urdf gazebo.launch.py brazo:=false   # simulación sola, sin placa
```

Los tres nodos que hacen falta para `display.launch.py` no son de este paquete;
si falta alguno:

```bash
sudo apt install -y ros-$ROS_DISTRO-robot-state-publisher \
                    ros-$ROS_DISTRO-joint-state-publisher-gui \
                    ros-$ROS_DISTRO-rviz2
```

- **`robot_state_publisher`** lee el URDF y publica las transformadas en `/tf`:
  es quien arma el robot.
- **`joint_state_publisher_gui`** es la ventana de deslizadores; publica en
  `/joint_states`.
- **`rviz2`** solo dibuja lo que los otros dos publican.

Ese reparto ahorra tiempo al depurar: si el robot aparece pero no se mueve, el
problema está en el segundo; si no aparece nada, en el primero.

---

## 4. La cadena: eslabones y juntas

| eslabón | malla | masa | qué es |
|---|---|---|---|
| `world` | — | — | el suelo; no existe en el export original |
| `Alvin` | `Alvin.STL` | 2.356 kg | la base / columna |
| `Simon` | `Simon.STL` | 1.935 kg | el brazo del hombro |
| `Teodoro` | `Teodoro.STL` | 2.135 kg | el brazo del codo |
| `Dave` | `Dave.STL` | 0.770 kg | el husillo |
| `Ian` | `Ian.STL` | 0.102 kg | la pinza |

| junta | tipo (gz) | padre → hijo | origen `xyz` | recorrido |
|---|---|---|---|---|
| `juntura1` | revolute | `Alvin` → `Simon` | `-0.14505 -0.77092 1.4844` | ±2.0944 rad (±120°) |
| `juntura2` | revolute | `Simon` → `Teodoro` | `0.24366 0.05 -0.055959` | ±2.618 rad (±150°) |
| `vertiacal1` | prismatic | `Teodoro` → `Dave` | `0.24492 0 0.05014` | 0.02 – 0.17 m (150 mm) |
| `manita` | revolute | `Dave` → `Ian` | `0 -0.11649 0` | ±3.1416 rad |
| `world_to_base` | fixed | `world` → `Alvin` | `0.14505 0.77092 -1.2344` | — |

El export gira la base 90° en X (`rpy="1.5708 0 0.0011194"` en `juntura1`), así
que aunque los ejes locales sean `0 1 0`, los tres primeros acaban siendo Z del
mundo y la muñeca −Z. El nombre `vertiacal1` lleva la errata del CAD; cambiarlo
obliga a tocar los YAML de `scara_kinematics`, así que se dejó.

**`manita` no existe en el brazo montado.** El robot de la mesa son 3 GDL: el
nodo `cc` no publica esa junta, así que `robot_state_publisher` la dibuja en su
cero y `Ian` se queda quieto. El URDF no hay que tocarlo para eso.

---

## 5. Los dos URDF

Mismo robot, misma geometría, mismas mallas. Lo que cambia:

| | `scara_urdf.urdf` | `scara_urdf_gz.urdf` |
|---|---|---|
| origen | export de SolidWorks, intacto | generado a partir de él |
| `juntura1`, `juntura2`, `manita` | `continuous` (sin topes) | `revolute` con topes reales |
| `effort` | **0** en las cuatro | 12 / 8 / 500 / 8 |
| `velocity` | 1.5 / 1.5 / 0.1 / 1.5 | 2.0 / 2.0 / 0.15 / 2.0 |
| anclaje al mundo | la junta `world_to_base` | la misma, y va primero |
| plugins de gz | no | sí, cinco |
| lo usa | `display.launch.py` | Gazebo, y el test de `scara_kinematics` |

**`effort="0"` no es un detalle.** Significa "este actuador no puede ejercer
par": para RViz da igual, porque solo dibuja, pero en Gazebo el robot se
desploma sin moverse. Por eso la versión `_gz` existe.

Los materiales van sin nombre (`<material name="">`, gris 0.894) en los dos
archivos: es como los deja el exportador.

---

## 6. Gazebo: el anclaje y los plugins

Sin `world_to_base` el robot es un cuerpo libre y se cae. El desplazamiento
`0.14505 0.77092 -1.2344` compensa el origen del ensamblaje de SolidWorks para
que el eje de `juntura1` quede en **(0, 0, 0.25)** del mundo.

`scara_urdf_gz.urdf` lleva cinco plugins nativos de gz (**no hay `ros2_control`
ni Gazebo Classic** en este workspace):

| plugin | juntas | ganancias |
|---|---|---|
| `JointStatePublisher` | las cuatro | — |
| `JointPositionController` | `juntura1` | P 60, I 0.5, D 8, cmd ±12 |
| `JointPositionController` | `juntura2` | P 30, I 0.3, D 4, cmd ±8 |
| `JointPositionController` | `vertiacal1` | P 4000, I 800, D 200, cmd ±400 |
| `JointPositionController` | `manita` | P 60, I 25, D 1, cmd ±8 |

El husillo lleva esas ganancias tan altas porque carga contra la gravedad: sin
integral se queda por debajo de la consigna.

El puente ROS ↔ gz **no está aquí**, lo monta `scara_kinematics/gazebo.launch.py`:

- consigna: `/scara_urdf/cmd/<junta>` (`std_msgs/Float64`) →
  `/model/scara_urdf/joint/<junta>/0/cmd_pos`
- medida: `/world/<mundo>/model/scara_urdf/joint_state` → `/gz/joint_states`
  (o `/joint_states` si se lanza con `brazo:=false`)

Mover una junta a mano, con la simulación sola:

```bash
ros2 topic pub /scara_urdf/cmd/juntura1 std_msgs/msg/Float64 "{data: 0.5}" --once
```

---

## 7. Las vistas de RViz

Las dos traen lo mismo —rejilla, `RobotModel` y `TF`— y las dos usan
**`Fixed Frame: Alvin`**. `urdf.rviz` es la que abre `display.launch.py`;
`scara.rviz` es la alternativa. La vista con la cadena completa de la simulación
es la de `scara_kinematics/rviz/scara.rviz`, que es otra.

---

## 8. Diagnóstico

| síntoma | causa |
|---|---|
| `Package 'scara_urdf' not found` | faltó `source install/setup.bash`, o `colcon build` falló |
| RViz abre vacío, **Status: Error** | el *Fixed Frame* tiene que ser `Alvin` (o `world`) |
| `No transform from [Simon] to [Alvin]` | no está corriendo `robot_state_publisher` |
| el robot sale sin mallas, solo ejes | RViz no encuentra los STL: recompila, se resuelven por `package://` desde `install/` |
| en Gazebo el robot sale sin cuerpo | falta el env-hook: `source install/setup.bash` otra vez, o no se instaló `share/` |
| aparece pero no se mueve | la ventana de deslizadores está minimizada, o falta `joint_state_publisher_gui` |
| el robot aparece lejos del centro de la rejilla | es el origen del ensamblaje, ver [sección 9](#9-lo-que-no-cuadra-con-el-cad) |
| en Gazebo se desploma | estás usando `scara_urdf.urdf` en vez de `scara_urdf_gz.urdf` (`effort=0`) |

```bash
ros2 node list                          # los tres nodos del display
ros2 topic echo /joint_states --once    # las juntas que se están publicando
ros2 run tf2_tools view_frames          # el árbol de TF en PDF
```

---

## 9. Lo que no cuadra con el CAD

Nada de esto impide visualizar ni simular, pero afecta al informe y a la celda.

**1. El modelo tiene 4 GDL, no 3.** El árbol es `Alvin → Simon → Teodoro → Dave →
Ian` con dos rotacionales, una prismática y `manita`, otra rotacional: es un
RRPR, no el RRP del informe. El brazo montado sí son 3 GDL, así que o se declara
`manita` como `fixed`, o el informe pasa a RRPR reconociendo el grado extra.

**2. Las longitudes no son las del informe.** Medidas sobre los orígenes de las
juntas del propio URDF:

| | CAD | informe |
|---|---|---|
| L₁ | 250 mm | 220 mm |
| L₂ | 250 mm | 180 mm |
| alcance | 500 mm | 400 mm |
| carrera del husillo | 150 mm | 150 mm ✓ |

Con las bases separadas 325 mm, 500 mm de alcance cambia el solapamiento entre
robots contiguos: o se ajusta el CAD, o se rehace la verificación de
interferencias.

**3. El origen del modelo no está en la base.** `juntura1` está en
`xyz="-0.14505 -0.77092 1.4844"`, a metro y medio del origen: se exportó desde el
ensamble de la celda, no desde el del robot. En RViz se ve como que el robot está
desplazado de la rejilla; en el URDF de Gazebo se compensa con `world_to_base`.
Lo correcto sería reexportar con el origen en la base de la columna.

**4. Quedan desalineaciones residuales en los `rpy`**: 0.064°, 0.120° y 0.056°,
de relaciones de posición no exactas en el CAD. Despreciables para dibujar; para
cinemática fina conviene dejarlas en cero.

**Los nombres de los eslabones** (`Alvin`, `Simon`, `Teodoro`, `Dave`, `Ian`)
funcionan, pero no son legibles en un informe. Si se cambian, hay que tocar el
URDF, el `Fixed Frame` de las dos vistas de RViz y los `joint_names` de los YAML
de `scara_kinematics`. Los nombres de las mallas son independientes y no hace
falta tocarlos.

---

## 10. Qué se borró

El 14 de septiembre de 2026, al limpiar el workspace (respaldo completo en
`~/respaldo_ros2_ws_src_2026-09-14.tar.gz`):

- `urdf/scara_urdf_corregido.urdf` — el paso intermedio entre el export y el
  `_gz`. Ya no existe: `display.launch.py model:=…corregido.urdf` **no funciona**.
- `urdf/*.bak`, `urdf/squashfs-root/`, `urdf/ros2_ws.zip` y el `export.log` de
  1.3 MB del exportador.
- El paquete `UrdfR3`, el otro SCARA del workspace.

Y el 15 de septiembre de 2026, `README.md` y `README_ROS2.md`, fundidos en este
archivo. De paso se corrigieron tres cosas que decían y ya no eran ciertas: el
argumento del launch es `model:=`, no `modelo:=`; no existe `robot:=scara_urdf`
ni los ejecutables `goto`/`analisis` de `scara_kinematics`; y la tabla de juntas
de `README_ROS2.md` daba como nombres de junta `Simon`, `Teodoro` y `Dave`, que
son **eslabones** — las juntas son `juntura1`, `juntura2`, `vertiacal1` y
`manita`.
