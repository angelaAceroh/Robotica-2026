# Trayectorias reales por Jacobiano (`ruta`)

Añadido del **29 de septiembre de 2026**. Es el código guía (la perforación
P1…P6 con control por Jacobiano) llevado al SCARA de la mesa: sus medidas, sus
límites, sus velocidades reales y unos puntos de trabajo que caben en él.

**Qué se ha tocado:** nada de lo que funcionaba. `tray`, los seis nodos, el
firmware, los URDF y los YAML están igual. Lo añadido:

| archivo | qué es |
|---|---|
| `scara_kinematics/ruta.py` | la guía, sección a sección: parámetros, puntos, rutas, FK/J/IK, seguimiento, informe, CSV y gráficas |
| `scara_kinematics/ruta_nodo.py` | el nodo que manda esa ruta a Gazebo (o al brazo) y mide lo que hizo |
| `launch/ruta.launch.py` | gz + el nodo, igual que `trayectoria.launch.py` |
| `test/test_ruta.py` | 22 tests: las medidas son las del YAML, FK/J/IK iguales a `model.py`, las rutas caben |
| `RUTA.md` | este archivo |
| `setup.py` | +2 líneas en `entry_points`: `ruta` y `ruta_nodo` |
| `package.xml` | +2 `exec_depend`: `python3-numpy` y `python3-matplotlib` |

---

## 1. Arranque

**Solo calcular**, como la guía: informe por consola, CSV y las tres gráficas.
No necesita Gazebo ni la placa.

```bash
cd ~/ros2_ws && colcon build --packages-select scara_kinematics && source install/setup.bash

ros2 run scara_kinematics ruta                          # perforado
ros2 run scara_kinematics ruta --ruta contorno
ros2 run scara_kinematics ruta --perfil lineal          # la ley horaria de la guía, tal cual
ros2 run scara_kinematics ruta --png ~/graficas         # además guarda las gráficas
```

El CSV va a `~/trayectoria_scara_<ruta>.csv` (o `--csv otro.csv`). Empieza con
las cuatro columnas de la guía (`tiempo, q1_rad, q2_rad, d3_mm`) y detrás lleva
tramo, tipo, velocidades articulares, la punta real y la deseada, y el error.

**Verlo en Gazebo** (arranca solo a los 6 s):

```bash
ros2 launch scara_kinematics ruta.launch.py
ros2 launch scara_kinematics ruta.launch.py ruta:=contorno
ros2 launch scara_kinematics ruta.launch.py registro:=~/ruta_gz.csv   # lo medido
ros2 launch scara_kinematics ruta.launch.py rviz:=true                # + los caminos
```

Y ya en marcha, por el topic de órdenes:

```bash
ros2 topic pub --once /scara/ruta/orden std_msgs/String "data: 'contorno perfil=quintico'"
ros2 topic pub --once /scara/ruta/orden std_msgs/String "data: 'parar'"
```

No se lanza a la vez que `tray`: los dos escriben en los mismos mandos de gz.

---

## 2. Las medidas del robot

| | la guía | el robot de la mesa | de dónde sale |
|---|---|---|---|
| eslabón 1 | a1 = 350 mm | **a1 = 250.0032 mm**, giro fijo b1 = +13.0° | `scara_urdf3.yaml` (`l1`, `b1`) |
| eslabón 2 | a2 = 250 mm | **a2 = 249.9997 mm**, giro fijo b2 = −11.6° | `l2`, `b2` |
| alcance | 600 mm | **23 a 500 mm** (el codo no pasa de ±150°) | `model.reach()` |
| eje z | z = d1 − d3 (baja) | **z = 65.51 + d3 (sube)** | `z_ref`, `z_dir` |
| recorrido de z | — | d3 de 20 a 170 mm → **z de 85.5 a 235.5 mm** | `d3_min`, `d3_max` |
| hombro / codo | sin límite | **±120° / ±150°** | `q1_*`, `q2_*` |
| vel. máx. juntas | 90°/s, 90°/s, 150 mm/s | **1.5 rad/s, 1.5 rad/s, 8 mm/s** | `max_vel_hw` de `esp.py` |

Las medidas están copiadas arriba del todo de `ruta.py`, en la sección 1, como en
la guía. `test_ruta.py` comprueba que son las del YAML; el nodo, de todas formas,
lee el YAML como los demás.

---

## 3. Lo que cambia respecto de la guía, y por qué

1. **Las medidas**, las de la tabla de arriba.
2. **Los giros fijos b1 y b2.** Al salir de SolidWorks los eslabones no arrancan
   alineados con X: `x = a1·cos(q1 + b1) + a2·cos(q1 + q2 + b2)`. Entran en la
   FK, el Jacobiano y la IK. Sin ellos la punta se va unos 13° de donde dice la
   fórmula.
3. **El husillo sube con d3**: la tercera fila del Jacobiano es `[0, 0, +1]`, no `−1`.
4. **Los ángulos no se normalizan.** La guía los mete en [−π, π] en cada paso. Este
   brazo no da la vuelta (topes a ±120°/±150°, cables), así que no hay que
   normalizar nada. Si una junta se sale de su recorrido, la ruta se rechaza
   **antes de mover nada**, diciendo en qué tramo y en qué segundo. Lo mismo si el
   brazo pasa casi estirado (singularidad).
5. **Velocidades reales.** Bajada 5 mm/s y subida 7 mm/s (la guía: 15 y 40); el
   husillo de la mesa no da más de 8. En el plano, 40 mm/s en el aire (la guía:
   120) y 15 en contacto. Así las juntas no pasan de 0.16 rad/s, y Gazebo las
   sigue bien por debajo de ~0.5.
6. **La ley horaria.** La guía arranca cada tramo ya a velocidad de crucero
   (`s = k/N`), o sea, con aceleración infinita en cada punto. Un motor no puede
   hacer eso. Por defecto el perfil es `trapecio`: rampa, crucero y rampa. Mientras
   taladra el avance es constante, como en la guía, pero arranca y para. Con
   `--perfil lineal` sale exactamente la ley de la guía. También valen los otros
   cuatro de `tray` (`cubico`, `quintico`, `septico`, `scurve`). En todos,
   `velocidad` es el **pico**.
7. **El codo `down`** (el `c2_sign = −1` de la guía). El brazo en reposo (q = 0) ya
   está en esa rama; la otra obliga a pasar por el brazo estirado. `--codo up`
   también sale (θ2 llega a 132°, cerca del tope de 150°).
8. **HOME al principio y al final**: (300, 0, 150) mm, la misma pose de reposo que `tray`.
9. **Un fallo de la guía: el robot va un paso por delante.** La guía mide el error
   contra `Pd`, que es el punto siguiente, y el feed-forward ya da ese paso. Al
   final el robot va por delante de la consigna, con un error fijo de `v·dt`.
   Medido con la propia guía: 0.54 mm al acabar un tramo a 120 mm/s (tiende a
   0.6), y 0.075 mm a 15 mm/s. Aquí el error se mide contra donde se debería
   estar **ahora**. Con eso el error del modelo baja de 200 a ~5 µm, y la
   autocorrección del final de cada tramo ya no tiene nada que corregir (0
   iteraciones). Se deja en el código por si se cambia `KP_POS` o `DT`.

Lo que **no** cambia: el método. Es el mismo feed-forward + `Kp·error`, el
Jacobiano inverso con `pinv`, la saturación de velocidad articular, Euler y la
autocorrección al final de cada tramo, y en el mismo orden.

---

## 4. Las rutas

Todos los puntos caen entre 280 y 380 mm de radio: lejos del brazo estirado
(500 mm) y con 25 mm de margen en z por arriba y por abajo.

| altura | z | para qué |
|---|---|---|
| `Z_SEGURO` | 150 mm | de paso, en el aire |
| `Z_TRABAJO` | 110 mm | fondo del agujero / contacto (40 mm de carrera) |

**`perforado`**: tres agujeros, igual que la guía.
HOME → P1 → **P2** → P1 → P3 → **P4** → P3 → P5 → **P6** → P5 → HOME.

| punto | x | y | z |
|---|---|---|---|
| P1 / P2 | 280 | 120 | 150 / 110 |
| P3 / P4 | 360 | 50 | 150 / 110 |
| P5 / P6 | 330 | −100 | 150 / 110 |

**`contorno`**: un cuadrado de 80 mm recorrido en contacto (marcar, desbarbar).
HOME → C0 → C1 → C2 → C3 → C4 → C1 → C0 → HOME, con C1…C4 las esquinas de
(290, −20) a (370, 60) a `Z_TRABAJO` y C0 encima de C1.

La **velocidad de cada tramo** la decide su forma, como en la guía, pero con un
caso más:

| tramo | tipo | velocidad |
|---|---|---|
| baja | `bajada` (la «perforación») | 5 mm/s |
| sube | `subida` (la «salida») | 7 mm/s |
| horizontal a `Z_SEGURO` | `desplazamiento` | 40 mm/s |
| horizontal por debajo | `trazado` (en contacto) | 15 mm/s |

**Para poner tus puntos:** se cambian en la sección 2 de `ruta.py`, y la ruta en
el diccionario `RUTAS` de la sección 3, como en la guía. Antes de mandarla a
ningún sitio, `ros2 run scara_kinematics ruta --ruta <nombre>`: si algo no
cabe, lo dice ahí, sin mover nada.

---

## 5. Resultados

**En el modelo** (`ros2 run scara_kinematics ruta`), error de la punta frente a
la deseada:

| ruta | perfil | duración | error máx. | error RMS | husillo pico |
|---|---|---|---|---|---|
| perforado | `lineal` (la guía) | 53.3 s | 5.3 µm | 2.1 µm | 7.0 mm/s |
| perforado | **`trapecio`** | 71.0 s | 5.3 µm | 1.7 µm | 7.0 mm/s |
| perforado | `quintico` | 99.9 s | 5.0 µm | 1.2 µm | 7.0 mm/s |
| contorno | `lineal` | 36.2 s | 4.4 µm | 0.7 µm | 7.0 mm/s |
| contorno | **`trapecio`** | 48.2 s | 4.2 µm | 0.6 µm | 7.0 mm/s |

Estos micrómetros son solo el error de integrar: el modelo es exacto por
definición. El de verdad es el de abajo.

**En Gazebo** (`ruta.launch.py gui:=false`), punta medida frente a la deseada:

| ruta | muestras | error medio | error máx. | RMS | en z (bajada / subida), medio | en el plano, medio |
|---|---|---|---|---|---|---|
| perforado | 3.553 | **0.69 mm** | 4.60 mm | 1.19 mm | 0.29 / 0.22 mm | 2.16 mm (desplazamiento) |
| contorno | 2.413 | **0.79 mm** | 2.83 mm | 0.96 mm | 0.18 / 0.23 mm | 1.11 mm (trazado), 1.88 mm (desplaz.) |

Lo que queda viene del control de posición de gz, no de la ruta. En el plano, el
hombro va 0.5–1° por detrás de la consigna, y a 300 mm de radio eso son 2–5 mm.
En z, el husillo sigue los 5–7 mm/s con 0.2–0.3 mm de error medio (1.2 mm de pico). Como
referencia, el demo de `tray` da 7.7 mm de media, pero va más deprisa
([TRAYECTORIAS.md §7](TRAYECTORIAS.md#7-lo-medido-en-gazebo)).

**Una trampa encontrada así:** la primera pasada dio **19.7 mm** de máximo, y no
era de la ruta. Al acabar la aproximación, el husillo de gz se había quedado
20 mm por detrás (se le pedían unos 30 mm/s) y lo fue arrastrando durante dos
tramos. Ahora la aproximación también respeta los 8 mm/s del husillo real, y
la ruta no empieza hasta que el brazo **ha llegado** al primer punto (fase
`llegada`, 0.005 rad / 0.5 mm, 15 s como mucho). En gz eso son unos 11 s de
espera.

El nodo saca estas cuentas solo al acabar cada ruta (en el log), y con
`registro:=` las deja muestra a muestra en un CSV.

---

## 6. Cuando conectes el brazo

Es el mismo camino que `tray` ([TRAYECTORIAS.md §10](TRAYECTORIAS.md#10-cuando-conectes-el-brazo)),
y tampoco se ha probado con la placa. Las diferencias:

```bash
ros2 launch scara_kinematics ruta.launch.py destino:=placa
```

- **No arranca sola.** Con `destino:=placa` el nodo espera a que la mandes, aunque
  el launch diga `ruta:=perforado`. Antes, `scara/hw/ok` tiene que ser `true`
  (placa viva, habilitada y con el cero hecho, desde `brazo_hw`):

  ```bash
  ros2 topic echo /scara/hw/ok                       # data: true
  ros2 topic pub --once /scara/ruta/orden std_msgs/String "data: 'perforado'"
  ```

- **Primero la aproximación.** El brazo está en reposo y la ruta empieza en HOME.
  Primero va a HOME en articular: sin IK, así que no puede cambiar de rama, y
  con el husillo a 8 mm/s como mucho (unos 15 s). Después espera a que el brazo
  llegue de verdad, y solo entonces empieza la ruta.
- **Una orden mandada con `scara/hw/ok` en false se tira**, no se guarda. Así
  no puede ponerse a mover el brazo sola cuando la placa esté lista.
- **La ruta dura lo mismo que en Gazebo.** El husillo ya está planificado a 5–7
  mm/s, por debajo de los 8 de la placa. Aquí no aparece la sorpresa de `tray`,
  que en la mesa iba 12 veces más lento.
- `parar` corta en cualquier momento, y si `scara/hw/ok` se cae a mitad, se para solo.

**Probado contra una placa falsa** (un nodo que hace de aduana y de `cc`: da
`scara/hw/ok` y un `/joint_states` que sigue la consigna con 0.1 s de retraso):

- con `ok` en false rechaza la orden;
- el contorno entero salen 3.171 consignas a 50 Hz, con d3 en metros (0.020 a
  0.0845) y un salto máximo entre muestras de 0.002 rad;
- el husillo no pasa de 8 mm/s;
- si `ok` cae a mitad del tramo 2, sale **0** consignas más;
- el Ctrl+C sale limpio.

Lo que no ha visto nunca es la ESP32.

---

## 7. Límites conocidos

- **El lazo del Jacobiano se cierra sobre el modelo, no sobre los sensores.**
  Como en la guía: la trayectoria se calcula entera y luego se reproduce. El lazo
  con lo que mide el brazo lo cierra ya el control de cada junta (el de gz o el
  PID de la ESP32). Un segundo lazo encima, con `/joint_states` llegando tarde,
  se pelearía con él. Lo medido se usa para el informe del final.
- **Sin orientación de la herramienta**: son 3 GDL, así que el giro de la pinza
  lo decide el brazo (q1 + q2).
- **Sin obstáculos.** Solo se comprueban alcance, límites de junta y singularidad.
- **Al cerrar Gazebo con Ctrl+C, a veces el `parameter_bridge` muere con
  segfault** (exit −11). Con `trayectoria.launch.py demo:=true` pasa igual: es
  cosa del puente de `gazebo.launch.py`, no de la ruta, y no afecta a nada.
- **La altura de la mesa no la sé.** `Z_TRABAJO = 110 mm` cabe en el recorrido
  del husillo, pero la pieza tiene que estar a esa altura en el mundo del robot.
  La punta no baja de 85.5 mm. Si la mesa está más baja, hay que calzar la
  pieza o cambiar `Z_TRABAJO`/`Z_SEGURO`.

---

Ver también: [TRAYECTORIAS.md](TRAYECTORIAS.md) (`tray`, los 25 métodos por IK) y
[NODOS.md](NODOS.md) (los seis nodos).
