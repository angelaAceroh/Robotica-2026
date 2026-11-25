# Línea de ensamble en Gazebo

Los cinco robots de la línea sobre la mesa del plano común, y el nodo maestro.

Equipo 5 — Transportador 1 · UMNG, Robótica

## Cómo se lanza

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch linea_planta linea_gazebo.launch.py maestro:=true
```

| Argumento | Defecto | Qué hace |
|---|---|---|
| `maestro` | `false` | `true` lanza también el nodo maestro y los 5 robots simulados |
| `escala` | `1.4` | multiplica las cotas del plano (ver abajo) |
| `gui` | `true` | `false` arranca Gazebo sin ventana |

Para seguir la línea desde otra terminal (con el mismo `ROS_DOMAIN_ID` y `source`):

```bash
ros2 topic echo /linea/estado
```

## Dónde está cada robot

Del plano del repositorio común, *PLANO PLANTA ROBOTICA - LAB 1*
(`Laboratorio-1-Robotica-UMNG/media/Plano.png`). Centímetros, origen en la esquina
inferior izquierda de la mesa, X a la derecha, Y hacia arriba. El origen de Gazebo es
esa misma esquina.

| Robot | Equipo | Centro en el plano (cm) | En Gazebo con escala 1.4 (m) |
|---|---|---|---|
| `transportador1` | 5 (nosotros) | (54.53, 64.02) | (0.763, 0.896) |
| `ensamblador1` | URDF `urdf_test` | (19.62, 71.18) | (0.275, 0.997) |
| `ensamblador2` | URDF `Ensamble` | (62.94, 42.12) | (0.881, 0.590) |
| `transportador2` | URDF `urdf_2` | (54.53, 21.02) | (0.763, 0.294) |
| `ensamblador3` | URDF `scara_urdf_2` | (19.62, 12.43) | (0.275, 0.174) |

Comprobado en la simulación: el eje de la primera articulación de cada robot cae en su
punto con 0.0 mm de diferencia.

La mesa, la entrada (P.I.), la salida (P.F.), las tres bases con la caja (verde) y los
tres almacenes (magenta) se dibujan como marcas planas, sin colisión. Las posiciones de
las estaciones se midieron sobre la imagen del plano, porque no todas están acotadas.

### La escala

El plano dibuja el alcance de los transportadores como R25 y el de los ensambladores
como R26, pero los robots reales alcanzan entre 0.33 y 0.58 m según sus URDF. Con el
plano tal cual (escala 1.0) los brazos quedan metidos unos en otros. La especificación
del transportador 1 es un círculo de Ø700 mm, o sea R350 frente al R25 del plano:
**350 / 250 = 1.4**. Si el plano real es otro, basta con cambiar `escala`.

## Topics de cada robot en Gazebo

Cada articulación escucha su consigna en `/<robot>/<articulación>/cmd_pos`
(`gz.msgs.Double`, radianes o metros) y cada robot publica su estado en
`/world/linea/model/<robot>/joint_state`.

| Robot | Articulaciones |
|---|---|
| `transportador1` | `J1` `J2` `J3`, en `/planta_brazo/J*/cmd_pos` (los de siempre) |
| `ensamblador1` | `JOINT1` `JOINT2` `JOINT3` |
| `ensamblador2` | `J1` `J2` `J3` (J3 prismática) |
| `transportador2` | `joint_1_base` `joint_2_hombro` `joint_3_brazo` `joint_4_muneca` |
| `ensamblador3` | `Simon_joint` `Teodoro_joint` `Dave_joint` (Dave prismática) |

Ejemplo, desde una terminal con Gazebo abierto:

```bash
gz topic -t /ensamblador1/JOINT1/cmd_pos -m gz.msgs.Double -p "data: 0.6"
```

**El nodo maestro todavía no mueve los robots de Gazebo.** Coordina por
`/<robot>/orden` y `/<robot>/estado`, y quien contesta son los `robot_simulado`. El paso
siguiente es que el nodo de cada equipo escuche su orden y mande estas consignas.

## Qué se cambió en los URDF de los otros equipos, y por qué

Los originales están sin tocar en `originales/`. Los que usa Gazebo, en `urdf/`, los
genera `herramientas/preparar_urdf.py`; si un equipo manda una versión nueva, se copia
a `originales/` y se vuelve a ejecutar la herramienta.

**A los cuatro:**

- **Se quitó `ros2_control`**. Este equipo no tiene instalado `gz_ros2_control`, así que
  se usan los controladores nativos de Gazebo, igual que en nuestro robot. Las ganancias
  son las mismas para todos y sirven para sostener y mover los brazos. **No son las de
  cada equipo**: las suyas estaban en archivos YAML que no venían en los zip.
- **Se quitaron las colisiones.** Son mallas de CAD pesadas y, con cinco brazos cuyos
  alcances se tocan, los robots se engancharían entre sí.
- **Se añadió amortiguamiento** (0.5) a cada articulación. Sin él, los controladores
  oscilan.

No se tocó la geometría, los orígenes, los ejes, las masas ni las inercias.

**Ensamblador 1 (`urdf_test`):** no estaba fijado al suelo y en Gazebo se habría caído.
Se le añadió el eslabón `world`.

**Ensamblador 2 (`Ensamble`):** **el zip no trae las mallas**, solo el URDF, que pide
`package://Ensamble/meshes/{base_link,J1,J2,J3}.STL`. Mientras no lleguen, se dibuja con
formas simples del tamaño que dan sus propias articulaciones. En cuanto las manden, se
ponen en `modelos/Ensamble/meshes/` y se quita la sustitución de la herramienta.
Su URDF quita la gravedad a sus eslabones; se respetó.

**Ensamblador 3 (SCARA, `scara_urdf_2`), tres problemas:**

- Cada articulación se llamaba **igual que su eslabón** (`Simon`, `Teodoro`, `Dave`).
  En URDF es válido, pero Gazebo lo pasa a SDF, donde no se puede, y **rechazaba el
  robot entero**. Se renombraron a `Simon_joint`, `Teodoro_joint` y `Dave_joint`.
- Las tres tenían **`effort="0"`**. En Gazebo ese es el esfuerzo máximo, así que
  **el robot no se podía mover**: el controlador calculaba la fuerza y se recortaba a
  cero. En RViz no se nota. Se puso 50 N·m en las de giro y 200 N en la prismática.
- Las dos continuas tenían el límite inferior por encima del superior (`lower=3 upper=0`
  y `lower=6 upper=-3`); en una continua no significan nada y se quitaron.
- No estaba fijado al suelo; se le añadió `world`.

**Transportador 2 (`urdf_2`):** **todo el robot está desplazado 1.11 m** respecto al
origen de su modelo: la malla de la base va de y = −1.22 a y = −1.00 y la primera
articulación está en y = −1.1147. Es típico del exportador de SolidWorks con el sistema
de referencia mal puesto. No se tocó su modelo; se compensa al colocarlo, para que su
eje quede en su sitio.

## Verificado

- Los cinco robots aparecen sin errores en Gazebo.
- El eje de cada uno cae en su punto del plano (0.0 mm de error).
- Todos se sostienen quietos, a menos de 0.14° de su consigna.
- Todos responden a una consigna: giros a 34.4° llegan a 34.5–34.8°. La prismática del
  SCARA es lenta en el último tramo (97 mm de 100 a los 9 s), porque carga su propio peso.
- Con `maestro:=true`, el maestro hace el ciclo completo de 9 pasos y la caja sale por la
  posición 4.

Las velocidades articulares muestran una vibración numérica de amplitud diminuta
(±0.05°, a cientos de hercios). No se ve, y nuestro robot la tiene igual.
