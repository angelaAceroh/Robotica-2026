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

## 8. Con los motores en MicroPython: `main.py` + el nodo `vector`

Añadido del **1 de octubre de 2026**. Sirve para cuando la ESP32 lleva tus
programas de MicroPython (el control de velocidad de `Alvin.py`, `Simon.py` y
`Teodoro.py`) en vez de `scara_esp32`.

**Corregido el 21 de octubre** con los `Alvin.py`, `Simon.py` y `Teodoro.py` que
funcionan: sus pines, sus 5500 pulsos por vuelta (22000/4), la transmisión
21/18 de Simon, Teodoro en mm, el error en porcentaje y sus recorridos. Los
números de abajo ya son esos. `main.py` y `main_control.py` (el del control
cinemático, [CONTROL_PLACA.md](CONTROL_PLACA.md)) cuentan igual y comparten
`config/micropython.yaml`: se puede pasar de uno a otro sin perder el cero.

**Las unidades son las de tus programas**: Alvin y Simon en grados de su junta y
RPM; Teodoro en **mm** y **mm/min** (lo que tu `Teodoro.py` llama «RPM»).

| archivo | dónde corre | qué es |
|---|---|---|
| `firmware/micropython/main.py` | la ESP32 | los tres motores a la vez, con tu mismo control |
| `firmware/micropython/Teodoro.py` | la ESP32 | Teodoro solo, leyendo el vector (con tus pines y en mm) |
| `scara_kinematics/vector.py` | el PC (nodo de ROS) | de la trayectoria al vector, tramo a tramo |
| `setup.py` | | +1 línea: `vector` |

**Por qué un `main.py` y no tres archivos:** en una ESP32 solo corre un
programa a la vez. Con `Alvin.py` cargado, Simon y Teodoro no existen. Dentro
de `main.py` cada junta sigue siendo su propio nodo (la clase `Motor`, con su
`INDICE` en la tabla `MOTORES`): **a los tres les llega el vector entero, y cada
uno toma solo su pareja**, `vector[2*INDICE]` y `vector[2*INDICE + 1]`. Y **por
qué el nodo está en el PC:** MicroPython no puede ejecutar un nodo de ROS 2.
Así que `main.py` hace de placa y `vector` es su cara en ROS.

```
 ruta.py (Jacobiano) ──► [vector] ──USB──► main.py ──► Alvin, Simon, Teodoro
                            ▲   [pA, vA, pS, vS, pT, vT]      │
                            └──────── LISTO pA pS pT ◄────────┘
```

### Cargarlo y arrancarlo

1. En Thonny, abre `firmware/micropython/main.py`. La tabla `MOTORES`, arriba
   del todo, lleva ya tus pines y tus cuentas.
2. *Archivo → Guardar como → Dispositivo MicroPython → `main.py`*. Arranca solo
   cada vez que se enciende la placa.
3. **La primera vez no sabe dónde está cada motor**: tus programas guardan en
   `posicion_guardada.txt` (uno para los tres), y `main.py` en
   `posicion_Alvin.txt`, `posicion_Simon.txt` y `posicion_Teodoro.txt`. Díselo
   con lo último que marcó cada uno: `fijar alvin 37.5`, `fijar simon -120`,
   `fijar teodoro -40`. **A Alvin no se le da la `h`** («NO DARLE A LA H»): la
   rechaza. A Simon y a Teodoro sí, `h simon`, cuando estén en su cero. `h` y
   `fijar` solo valen con los tres parados.
4. Cierra Thonny (el puerto solo puede tenerlo uno) y en el PC:

```bash
ros2 run scara_kinematics vector --ros-args \
    --params-file install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
ros2 topic pub --once /scara/vector/orden std_msgs/String "data: 'perforado'"
```

Sin placa: `-p simular:=true` hace todo menos abrir el puerto.

### La ventana `efector`: escribes (x, y, z) y te dice si llega

Con los recorridos de tus programas: **Alvin de 0° a 100°, Simon de −200° a 0°**
(llega a −280, pero «de preferencia no pasarse de 200») **y Teodoro de −100 a
0 mm**.

```bash
ros2 launch scara_kinematics efector.launch.py                         # ventana + vector, con la placa
ros2 launch scara_kinematics efector.launch.py simular:=true           # sin placa
ros2 launch scara_kinematics efector.launch.py simular:=true gz:=true  # sin placa, viéndolo en Gazebo
ros2 launch scara_kinematics efector.launch.py gz:=true                # la placa, con Gazebo de espejo
```

Con **`gz:=true`**, `vector` manda a los mandos de Gazebo la posición de los
motores pasada a junta: la de la placa de verdad (su telemetría), o la de la de
mentira con `simular:=true`, que mueve cada motor a sus 14 RPM, 30 RPM y
60 mm/min. El husillo de gz tarda unos segundos en llegar, como siempre.

- **La planta**: en verde, la zona donde llega la punta con esos límites; en gris,
  el brazo donde está; en azul, donde iría. Si el punto no llega, sale una X
  roja. Con un clic pones x e y.
- **El veredicto**: ALCANZABLE / NO ALCANZABLE, y qué motor no llega y hasta
  dónde tendría que ir (prueba las dos ramas del codo).
- **Los ángulos**: en la junta (lo que sale de la cinemática) y en el motor (lo
  que se le manda a `main.py`).
- **Ir** (o Enter) le manda `ir x y z` a `vector`, que lo vuelve a comprobar
  antes de mandar nada. **Esc** = parar.

Sin la ventana: `ros2 topic pub --once /scara/vector/orden std_msgs/String "data: 'ir 430 -160 125'"`.

**Lo que sale en la consola** del `ros2 launch` con cada Ir: el resultado de la
trayectoria, el vector que se manda, lo que le llega a la placa y lo que toma
cada nodo de junta. Las líneas `la placa recibe` y `nodo … toma` las imprime
`main.py` en la ESP32 (también se ven en Thonny) y `vector` las repite:

```
TRAYECTORIA  ir (430.0, -160.0, 125.0) mm -> ALCANZABLE, codo down
   junta:  theta1 -9.99°   theta2 -22.22°   d3 59.49 mm
   motor:  Alvin +9.99°   Simon -22.22°   Teodoro -39.49 mm
VECTOR -> placa (ir 430 -160 125, ~39.5 s): [9.986, 14, -22.221, 30, -39.49, 60] *F8
   la placa recibe [9.986, 14, -22.221, 30, -39.49, 60] checksum ok   ✓ igual que lo mandado
   nodo Alvin    toma  pos    +9.99°   vel 14 RPM   (estaba en +0.00°)   ✓
   nodo Simon    toma  pos   -22.22°   vel 30 RPM   (estaba en +0.00°)   ✓
   nodo Teodoro  toma  pos   -39.49 mm   vel 60 mm/min   (estaba en +0.00 mm)   ✓
   CONFIRMADO en 22 ms: la placa leyo el vector entero y cada nodo tomo su pareja
   placa: [!] Alvin: OBJETIVO ALCANZADO en 9.99° (meta 9.99°, error +0.00°)
   placa: [!] Simon: OBJETIVO ALCANZADO en -22.22° (meta -22.22°, error +0.00°)
   placa: [!] Teodoro: OBJETIVO ALCANZADO en -39.49mm (meta -39.49mm, error +0.00mm)
LISTO (ir 430 -160 125): Alvin +9.99°   Simon -22.22°   Teodoro -39.49 mm
```

(Es la placa de mentira de `simular:=true`, que llega justa; la de verdad se
pasa un poco por inercia.) **Teodoro tarda 39.5 s** en esos 39.5 mm: a 60 mm/min
va a 1 mm/s. Es el máximo de tus programas («usa velocidades de 10 a 60»).

Un punto que no llega sale en rojo con el motivo. Pasa al pulsar Enter en la
ventana, aunque el botón Ir esté apagado.

**No va en `brazo_hw`** porque esa ventana habla con la placa a través de
`esp`, en el protocolo de `scara_esp32`, y con `main.py` en la placa no movería
nada.

**`vector` ya no manda nada fuera de recorrido**: ni un `ir`, ni una ruta (la
comprueba entera antes de empezar), ni un vector tecleado a mano.

**Lo que dicen esos límites, con los supuestos de ahora** (0 de Alvin y Simon =
brazo estirado; Alvin positivo en horario, Simon en antihorario; 0 de Teodoro =
abajo del todo, d3 = 20 mm; ver [CONTROL_PLACA.md](CONTROL_PLACA.md) §4):

- La punta llega a casi media vuelta por el lado de y negativa: x entre −237 y
  488 mm, y entre −488 y 6 mm. El codo, como mucho a 150° (el CAD), aunque Simon
  pueda ir a −200.
- La z de la punta va de 85.5 a 185.5 mm (Teodoro de 0 a −100 mm).
- **Ningún punto del perforado ni del contorno cae dentro**: en todos, Alvin
  tendría que ir a negativo. `vector` los rechaza.
- Si `signo` no es el supuesto, la zona es otra muy distinta: con Alvin positivo
  antihorario, `ir 430 -160 125` ya no llega. Por eso hay que confirmarlos antes
  de fiarse del verde.

Todos estos números están en **`config/micropython.yaml`**, el único sitio donde
se tocan. Lo leen `vector` y `efector`.

### Los límites: dónde están y cómo se cambian

Están en **dos sitios**, a propósito:

| dónde | qué | para qué |
|---|---|---|
| `config/micropython.yaml` → `limites_motor` | `[-60.0, 60.0, -280.0, 0.0, -135.0, 0.0]` (min y max de Alvin y Simon en grados, Teodoro en mm; los que dio Sergio el 21/10) | el PC: decide si un punto es alcanzable y no manda nada fuera |
| `firmware/micropython/main.py` y `main_control.py` → tabla `MOTORES`, columnas `MIN` y `MAX` | los mismos números | la placa: rechaza un vector que saque a algún nodo de su recorrido (por ejemplo, uno tecleado en Thonny), y entonces **no se mueve ninguno** |

**Para cambiarlos:**

1. Cambia los números en los dos sitios.
2. Recompila: `colcon build --packages-select scara_kinematics`.
3. Sube otra vez `main.py` (o `main_control.py`) a la placa.

Si se te olvida uno, se nota:

- Al conectar, `vector` le pide a la placa sus límites (`l`) y los compara: dice
  «coinciden», o «LIMITES DISTINTOS» con los dos juegos de números.
- `test_main_micropython.py` falla si `main.py` y el YAML no tienen los mismos,
  o si `main.py` y `main_control.py` no tienen la misma tabla.

Además de estos, siguen valiendo los topes del CAD de `scara_urdf3.yaml` (±120°,
±150°, d3 de 20 a 170 mm), que son más anchos.

### El error de posición

Se ve en tres sitios:

- **En la placa**, cuando cada nodo llega:
  `[!] Alvin: OBJETIVO ALCANZADO en 10.28° (meta 10.00°, error +0.28°)`. Es el
  error en el instante en que corta el motor.
- **En la consola de `vector`**, al recibir `LISTO`, con los motores ya quietos
  (incluye lo que se pasan por inercia):
  `ERROR DE POSICION  Alvin +3.03°   Simon -4.59°   Teodoro -0.03 mm   |   punta: ...`.
  También sale por los topics `scara/vector/error` (Vector3: grados, grados y
  mm) y `scara/vector/error_punta` (Float64, mm), para rqt.
- **En la ventana `efector`**, en directo: la fila «error (ahora − meta)» de
  cada motor y el error de la punta frente al último punto mandado con Ir.

Los números de arriba son de `main.py` con el vector `[10, 14, -22, 30, -2, 60]`
y los motores de mentira de
`test/micropython_falso/`, que no tienen rozamiento y se pasan más que los de
verdad (unos 3° a 14 RPM). La primera medida real saldrá en esa misma línea.

### Cómo se asegura que llega a los nodos y que lo entienden bien

En cada vector hay cuatro comprobaciones, en este orden:

1. **Checksum.** `vector` manda `[...] *21`. `main.py` rehace la suma y, si no
   cuadra (ruido del USB), contesta `ERROR vector checksum` y **no se mueve
   nada**. `vector` lo reintenta una vez y, si vuelve a fallar, corta.
2. **Eco.** `main.py` devuelve lo que ha leído (`VECTOR [...] checksum ok`) y
   `vector` lo compara número a número con lo que mandó.
3. **Cada nodo.** Cada nodo dice qué ha tomado del vector
   (`RECIBE Simon pos -22.221 vel 30`), y `vector` comprueba que es justo
   `vector[2·índice]` y `vector[2·índice+1]`. En la consola sale un ✓ por nodo y
   `CONFIRMADO en 25 ms`.
4. **Plazo.** Si en 1.5 s no han llegado el eco y los tres `RECIBE`, o algo no
   cuadra, sale `VECTOR NO CONFIRMADO: <qué>` en rojo, `vector` manda `q` y corta.

Probado metiéndole a propósito un fallo a `main.py`: Simon leía la pareja de
Teodoro. `vector` dijo «Simon ha tomado pos 0 vel 60 y le tocaba pos -22.221 vel
30», mandó `q` y la placa quedó `PARADO` sin haberse movido.

Lo que no cubre: entre que la placa recibe el vector y llega la `q` del PC pasan
unos 25 ms. Si un nodo lo entendiera mal, en ese tiempo ya habría empezado a
moverse. Los límites de la placa siguen protegiendo.

**Y el test que lo comprueba sin placa:** `test/test_main_micropython.py` ejecuta
el `main.py` de verdad en el PC (con el `machine` de mentira de
`test/micropython_falso/`) y comprueba que:

- cada nodo toma su pareja y llega;
- con velocidad 0 ese motor no se mueve;
- un checksum malo no mueve nada;
- si uno se sale del recorrido, no se mueve ninguno;
- un vector a mano sin checksum vale;
- los límites coinciden con el YAML;
- un motor al revés para a los tres;
- a Alvin no se le da la `h`, y `fijar` deja cada motor donde se le dice;
- `main.py` y `main_control.py` tienen la misma tabla de motores.

```bash
cd ~/ros2_ws && python3 -m pytest src/scara_kinematics/test/test_main_micropython.py
```

**Probar la cadena entera sin la ESP32:**
`test/micropython_falso/placa_falsa.py` pone `main.py` detrás de un puerto serie
falso y dice cuál es (`puerto: /dev/pts/5`):

```bash
python3 src/scara_kinematics/test/micropython_falso/placa_falsa.py
ros2 launch scara_kinematics efector.launch.py puerto:=/dev/pts/5 gz:=true
```

### Lo que queda por comprobar con el brazo

Tus programas ya dan grados de junta y mm, así que `reduccion` es `[1, 1]` y
`paso_husillo_mm` es `360` (un «grado» de Teodoro es un mm). Lo que sigue siendo
**supuesto**:

| parámetro | supuesto | cómo comprobarlo, con `main.py` cargado y desde Thonny |
|---|---|---|
| `signo` | `[-1, -1, 1]` (**comprobado** el 21/10, CONTROL_PLACA.md §9) | Alvin y Simon: «los negativos son en sentido contrario a las agujas del reloj» (Sergio), o sea, su positivo es horario. Teodoro con −5.4 mm **bajó**: su positivo sube |
| `cero` | `[-45.68, -90.01, 155]` | dónde está cada junta con su motor en 0 (grados, grados, d3 en mm). Alvin y Simon en 0 son **tu HOME**, (0, −270) en el plano de la ventana desde el 23/10 (antes (0, −250), `[-43.0, -95.38, 155]`). **Supuesto**: Teodoro en 0 arriba del todo, con abajo del todo (−135) en el tope bajo del CAD, d3 = 20. Ver CONTROL_PLACA.md §9 |

Se cambian en `config/micropython.yaml`, y después hay que recompilar:
`colcon build --packages-select scara_kinematics`.

### Tres cosas que no son obvias

- **Las velocidades son fijas: Alvin 14 RPM, Simon 30 RPM y Teodoro 60 mm/min**
  (las pidió Sergio el 1 de octubre; `velocidades` en `config/micropython.yaml`).
  Cada vector sale como `[pos_alvin, 14, pos_simon, 30, pos_teodoro, 60]`, y los
  tres motores no llegan a la vez: Teodoro va a 1 mm/s y es el que más tarda. Si
  se pone alguna a `0.0`, `vector` vuelve a calcularlas por tramo para que
  lleguen juntos. Pero la trayectoria pide al hombro y al codo 0.1–1.1 RPM, y la
  placa no baja de 10.
- **Teodoro por debajo de unos 45 mm/min va a saltos**: a esa velocidad el
  encoder da menos de un pulso cada 10 ms, y la velocidad medida salta entre 0 y
  algo (en tu `Teodoro.py` pasa igual, es la misma cuenta). En la simulación, a
  10 mm/min hizo 1 mm en 2.3 s en vez de 6 s. A 60 mm/min va justo.
- **El error del control es en porcentaje**, como en tus programas: 100 al
  arrancar sea cual sea la velocidad. Tus programas hacen
  `(|ref| − |rpm|)/|ref|·100·signo`; `main.py` hace `(ref − rpm)/|ref|·100`, que
  da lo mismo mientras el motor gira hacia donde se le pide. Si gira al revés,
  la tuya cree que va bien y esta empuja más fuerte (y salta el vigilante).
- **Tu control no frena antes de llegar:** corta el PWM al pasar el destino, y el
  motor sigue un poco por inercia. Por eso la placa solo dice `LISTO` cuando los
  tres están quietos, y el nodo calcula cada vector desde donde dice la placa que
  está cada uno. Si un motor se pasó, el tramo siguiente lo corrige.
- **Vigilantes en `main.py`.** Si un motor se mueve al revés más de 5° (de su
  eje: 0.6 mm en Teodoro), o lleva 2 s sin girar ni 1°, o tarda el triple de lo
  previsto, la placa **para los tres** y dice `ERROR`, y el nodo corta la ruta.
  Sin el primero, en la simulación Alvin al revés llegaba a −278°. Ojo con
  **Simon**: el 11 de septiembre su canal B (GPIO 13) no contaba. Con un solo
  canal, la cuadratura suma +1 y −1 y no avanza: lo para el vigilante de los 2 s.

### Probado

Sin placa:

- **`main.py` en el PC**, con un `machine` falso y tres motores simulados
  (inercia y encoder en cuadratura, tus pines y 5500 pulsos): los tres se mueven
  a la vez, saltan los vigilantes, y funcionan `q`, `h simon`, `fijar` y `?`.
  `h alvin` se rechaza.
- **El nodo contra ese `main.py`**, por un puerto serie falso (pty): el
  perforado entero, 11 vectores y 11 `LISTO`. Un `ERROR` corta la ruta tras el
  primer vector.

Con la ESP32, todavía nada.

---

Ver también: [TRAYECTORIAS.md](TRAYECTORIAS.md) (`tray`, los 25 métodos por IK) y
[NODOS.md](NODOS.md) (los seis nodos).
