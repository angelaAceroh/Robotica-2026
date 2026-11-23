# Control cinemático en Gazebo (`control_nodo`)

Añadido del **13 de octubre de 2026**; la ventana y las correcciones del §6
(puntos 5 a 7), del **15 de octubre**. Es el control cinemático del SCARA de la
mesa, cerrado con lo que **mide** Gazebo, y su Gazebo propio con las juntas
movidas por **velocidad**. Por ahora solo en simulación.

**Qué se ha tocado:** nada de lo que funcionaba. Los seis nodos, `ruta`, `tray`,
los URDF, los YAML y `gazebo.launch.py` están igual. Lo añadido:

| archivo | qué es |
|---|---|
| `scara_kinematics/control_cinematico.py` | la ley, las referencias, las comprobaciones de antes de mover y un simulador ideal (sin ROS) |
| `scara_kinematics/control_nodo.py` | el nodo: cierra el lazo con lo que mide gz, a 100 Hz |
| `launch/control.launch.py` | su Gazebo (el mismo robot, con las juntas por velocidad), el puente y el nodo |
| `scara_kinematics/control_gui.py` | la ventana: manda las órdenes y enseña el lazo (§4, «La ventana») |
| `launch/control_gui.launch.py` | `control.launch.py` + la ventana; cerrarla cierra todo |
| `rviz/control.rviz` | el camino pedido, el hecho y la consigna |
| `rqt/control.perspective` | cuatro gráficas: x e y, z, el error y qd |
| `test/test_control.py` | 21 tests: el control contra la teoría, y el alcance que sombrea la ventana |
| `CONTROL.md` | este archivo |
| `setup.py` | +2 líneas en `entry_points`: `control_nodo` y `control_gui` |

---

## 1. Arranque

```bash
cd ~/ros2_ws && colcon build --packages-select scara_kinematics && source install/setup.bash

ros2 launch scara_kinematics control_gui.launch.py                   # Gazebo + la ventana: lo normal
ros2 launch scara_kinematics control_gui.launch.py tarea:=nada k:=10  # sin tarea, otra K

ros2 launch scara_kinematics control.launch.py                       # sin ventana: un círculo
ros2 launch scara_kinematics control.launch.py tarea:=perforado      # las rutas de ruta.py
ros2 launch scara_kinematics control.launch.py tarea:=contorno
ros2 launch scara_kinematics control.launch.py tarea:=home k:=10     # otra ganancia
ros2 launch scara_kinematics control.launch.py rqt:=true rviz:=true  # + gráficas y RViz
ros2 launch scara_kinematics control.launch.py gui:=false registro:=~/control.csv
```

La primera orden (`tarea:=`, por defecto `circulo`) sale a los 6 s, cuando gz
ya ha cargado el robot. Con `tarea:=nada` no hace nada y espera órdenes (`tarea:=`
vacío no vale: `ros2 launch` lo rechaza). Lo demás se manda en marcha, desde la
ventana o a mano:

```bash
ros2 topic pub --once /scara/control/orden std_msgs/String "data: 'salto 320 20 150'"
ros2 topic pub --once /scara/control/orden std_msgs/String "data: 'k=10'"
ros2 topic pub --once /scara/control/orden std_msgs/String "data: 'empujon 0 15 0'"
```

No se lanza a la vez que `gazebo.launch.py`, `ruta.launch.py`,
`trayectoria.launch.py` ni `efector.launch.py gz:=true`, porque serían dos Gazebo con
el mismo robot y los topics se cruzarían.

---

## 2. La ley

```
 xd(t), ẋd(t) ──►(+)──► u = ẋd + K·e ──► q̇ = J(q)⁻¹·u ──► juntas de gz (por velocidad)
                  ▲ −                                              │
                  └──────── x = FK(q medida) ◄── /joint_states ◄───┘
```

En cada tic, a 100 Hz con el reloj de gz:

1. `q` es lo que mide gz y `x = FK(q)` es la punta de verdad.
2. `e = xd − x` es el error, en mm.
3. `q̇ = J⁻¹·(ẋd + K·e)` es la velocidad que se manda a las juntas. `J` es el
   Jacobiano 3×3 de x, y, z, el mismo de `ruta.py`. El giro de la pinza no se
   controla: con 3 GDL lo decide el brazo.

Si las juntas hacen la velocidad que se les pide, el error cumple `ė = −K·e`, es
decir:

- cae como **`e(t) = e₀·e^(−K·t)`**;
- al 37 % en **τ = 1/K** y al 5 % en **3/K**;
- **en línea recta** hacia la consigna.

Eso es lo que se mide en el §5.

**La diferencia con `ruta`:** `ruta` hace la misma cuenta, pero sobre el modelo.
Integra la trayectoria entera antes de mover nada y luego la manda como
posiciones; el lazo con el brazo lo cierra el PID de posición de cada junta
(`RUTA.md` §7). Aquí no hay PID de junta: el único lazo es este, y se cierra
con la punta medida.

**Las tres protecciones, antes de mandar:**

- **Brazo casi estirado** (|sin codo| < 0.10, unos 5.7°). Se usan mínimos
  cuadrados amortiguados: el brazo deja de seguir la consigna exacta en vez de
  pedir velocidades enormes.
- **Topes.** Una junta en su tope no se deja seguir hacia él, pero sí salir.
- **Saturación.** Si alguna junta pasa de su techo (1.5 rad/s, 1.5 rad/s y
  8 mm/s, los del brazo real), las **tres** se escalan por el mismo factor. La
  punta va más despacio, pero en la misma dirección. Si se recortara cada junta
  por su lado, la punta se torcería.

**Antes de mover**, cada tarea se recorre entera con la IK, en la rama del codo
en la que esté el brazo, y se rechaza si se sale de alcance, de un tope o pasa
casi estirada. Si a esa velocidad alguna junta pasaría del 80 % de su techo, se
hace más lenta y se avisa. El 20 % que queda es para que `K·e` pueda corregir.

---

## 3. Su Gazebo: las juntas por velocidad

El Gazebo de siempre (`gazebo.launch.py`) mueve cada junta con un **PID de
posición**. El control cinemático manda **velocidades**, y ponerlo encima de un
PID de posición serían dos lazos peleándose. Así que `control.launch.py` monta
su propio Gazebo, con un único cambio:

- Lee `scara_urdf_gz.urdf` y escribe una copia en `/tmp/scara_urdf_control_vel.urdf`.
- En esa copia, hombro, codo y husillo llevan `gz-sim-joint-controller-system` en
  modo velocidad (`use_force_commands=false`): gz impone la velocidad pedida,
  hasta el esfuerzo del `<limit>` de la junta.
- `manita` sigue con su PID de posición, quieta en su cero.
- El URDF original no se toca.

```
control_nodo --Float64--> /scara_urdf/control/cmd_vel/<junta>  --puente-->  /model/scara_urdf/joint/<junta>/cmd_vel
gz --> /world/empty/model/scara_urdf/joint_state  --puente-->  /joint_states  --> control_nodo
```

Comprobado a mano antes de montar nada encima:

- la junta sigue la velocidad al instante;
- el husillo se sostiene contra la gravedad con velocidad 0;
- los topes del URDF se respetan (el husillo se para en 20 mm);
- **gz se queda con la última velocidad que recibió.** Por eso hay vigilante
  (§6.4).

**Lo que se ve en la ventana de Gazebo:**

- La cámara se coloca sola, cerca y en diagonal. Se gira con el ratón.
- El camino pedido es una cinta **verde** plana.
- Lo que va haciendo la punta es una cinta **naranja** de canto, que se
  repinta cada 2 s. Si van juntas, la naranja tapa a la verde según avanza.
- La bola verde marca el final de la tarea.

Con `gui:=false` no se pinta nada.

**En RViz** (`rviz:=true`) se ve lo mismo y además la consigna de ahora (la
bola). **En rqt** (`rqt:=true`) hay cuatro gráficas:

- x e y, pedidas y medidas;
- z;
- el error (x, y, z y la norma), en mm;
- q̇ mandado (°/s, °/s, mm/s).

---

## 4. Las órdenes

Por `/scara/control/orden` (`std_msgs/String`). Una orden nueva sustituye a la
que esté en marcha. Sin tarea, el nodo **regula**: mantiene la punta en el
último punto pedido.

| orden | qué hace |
|---|---|
| `home` | a HOME (300, 0, 150) mm, en articular |
| `ir X Y Z [v=40] [perfil=trapecio]` | recta hasta (X, Y, Z) mm. **Seguimiento** |
| `salto X Y Z` | la consigna salta de golpe. **Regulación**: mide τ y lo compara con 1/K |
| `circulo [CX CY R] [z=150] [v=20] [vueltas=1] [perfil=trapecio]` | círculo horizontal; sin números, el de (330, 0) con R = 40 |
| `perforado` \| `contorno` `[perfil=…]` | las rutas de `ruta.py`, con sus velocidades por tramo |
| `empujon DX DY DZ` | durante 0.2 s saca la punta DX, DY, DZ mm **sin control**; luego vuelve el lazo y mide cómo recupera |
| `k=5` \| `k=5,5,2` | cambia K en marcha (una para todo, o una por eje x, y, z) |
| `parar` | se queda donde está, regulando ahí |

Los círculos y las rutas empiezan con una **aproximación en articular** desde
donde esté el brazo. Es la misma que hacen `tray` y `ruta_nodo`, pero dada como
referencia de la punta para que la siga el mismo control. Desde el reposo tarda
unos 16 s, porque el husillo sube a 8 mm/s como el de verdad.

Los perfiles son los seis de `ruta`: `lineal`, `trapecio`, `cubico`, `quintico`,
`septico` y `scurve`. Al acabar cada tarea el nodo escribe en el log el error
medio, el máximo y el RMS, por tipo de tramo. Al acabar un salto o un empujón
escribe la τ medida junto a la teórica.

**El CSV** (`registro:=`) lleva una fila por tic con:

- `t_medida`, el instante en que midió gz;
- la tarea y el tramo;
- la punta pedida, la medida y el error (x, y, z y la norma);
- q y q̇;
- la saturación (> 1: se recortó), el amortiguamiento y K.

### La ventana (`control_gui`)

`control_gui.launch.py` es `control.launch.py` incluido tal cual, más una
ventana Tk hecha para la pantalla de 1366×768. No calcula nada: compone las
mismas órdenes de la tabla y las publica en `/scara/control/orden`. Cerrarla
cierra Gazebo, el nodo y el puente.

- **Cabecera:** si el control está regulando, moviendo, sin medidas o no está;
  *Pausar Gazebo* (el control se para con él, corre con su reloj) y **Parar**
  (también con `Esc`).
- **Planta:** el camino pedido (verde), lo que hace la punta en Gazebo
  (naranja), la consigna de ahora (anillo) y, en azul discontinuo, lo que
  mandarían los campos. Sombreado, hasta dónde llega el brazo con el codo
  `down`; más claro, solo con el `up`. Clic: X, Y de *Ir a un punto*; clic
  derecho: centro del círculo. A la derecha, la regla de z.
- **|error|** de los últimos 15 s en **escala logarítmica**: una caída
  `e₀·e^(−K·t)` sale recta, y se ven a la vez un salto de 30 mm y un
  seguimiento de 0.01 mm. En rojo, donde la ley saturó.
- **Órdenes:** Home, Perforado, Contorno y el perfil; *Ir en recta* y *Salto*
  a (X, Y, Z); *Círculo*; *Empujar*; K con su τ. Un campo que no es un número
  se pone en rojo y no se manda nada.
- **Ahora:** tarea, tramo y avance; punta, consigna, error (y el máximo desde
  la última orden), saturación y la K que lleva el nodo.
- **Mensajes:** lo que dice `control_nodo` (en verde, la τ medida y el error al
  acabar) y una línea para órdenes a mano.

Mandar desde la ventana es mejor que `ros2 topic pub --once`. Cada `pub` es un
participante de DDS que entra y sale, el nodo se para un momento cada vez
(§6.3) y alguna orden se perdía (§7). La ventana entra una vez, y no deja mandar
nada hasta que `control_nodo` la escucha.

Probada pulsando sus botones contra Gazebo, el 15 de octubre:

| orden | resultado |
|---|---|
| Home | error medio 0.005 mm |
| Salto de 36 mm | τ medida **0.209 s** (teoría 0.200) |
| Empujón de 15 mm | τ medida **0.209 s** |
| Perforado | error medio **0.018 mm** |
| `abc` en un campo | no sale nada |

---

## 5. Lo medido en Gazebo

Todo con `control.launch.py` en este portátil. Gazebo iba entre el 40 y el 90 %
del tiempo real; no importa, porque todo corre con el reloj de gz.

### Regulación: un salto de 14.1 mm en el plano

| K [1/s] | τ teoría | **τ medida** | 5 % teoría | **5 % medido** |
|---|---|---|---|---|
| 2 | 0.500 s | **0.505 s** | 1.500 s | **1.488 s** |
| 5 | 0.200 s | **0.207 s** | 0.600 s | **0.591 s** |
| 10 | 0.100 s | **0.108 s** | 0.300 s | **0.291 s** |
| 20 | 0.050 s | **0.063 s** | 0.150 s | **0.149 s** |
| 40 | 0.025 s | 0.048 s | 0.075 s | 0.072 s — **saturó** (1.3 veces el techo) |

Hasta K = 20 sale la exponencial de la teoría. Los 8–13 ms que sobran son el
retraso del lazo, menos de un tic. Con K = 40 el error inicial pide más
velocidad de la que dan las juntas, y mientras satura el error cae en rampa, no
en exponencial.

### Hasta dónde se puede subir K

Con un salto de 2 mm, para que no sature, lo que se pasa de largo:

| K | 20 | 30 | 60 | 90 |
|---|---|---|---|---|
| **Gazebo** | 0 % | 0 % | 6 % | 32 % |
| simulador ideal, 1 tic de retraso | 0 % | 1 % | 44 % | 88 % (inestable desde K = 100) |

Es lo que pasa en todo lazo discreto con retraso. Con un tic de retraso a
100 Hz, el error no se pasa de largo hasta K = 25 y oscila más cuanto más se
acerca a K = 100, donde se vuelve inestable. Gazebo se porta como algo menos de
un tic. **K entre 5 y 20 es la zona buena.** Por encima solo se gana oscilación,
y en el brazo real, con más retraso, antes.

### Empujones

| empujón | τ de la vuelta | |
|---|---|---|
| 15 mm en y | **0.211 s** (teoría 0.200) | K = 5 |
| 10 mm en x y 5 mm en z | 0.407 s | saturó: 5 mm en z piden 25 mm/s y el husillo da 8 |

El segundo enseña lo que hace la saturación común. Mientras el husillo no
alcanza, el plano también va despacio, pero la punta vuelve en línea recta. Lo
comprueba `test_el_husillo_satura_y_frena_a_las_tres`.

### Seguimiento: error de la punta medida frente a la pedida

| tarea | K | error medio | error máx. |
|---|---|---|---|
| círculo R 40 mm a 20 mm/s | 2 | 0.046 mm | 0.586 mm |
| círculo R 40 mm a 20 mm/s | **5** | **0.012 mm** | **0.019 mm** |
| círculo R 40 mm a 20 mm/s | 20 | 0.003 mm | 0.005 mm |
| círculo R 40 mm a 60 mm/s | 5 | 0.105 mm | 0.158 mm |
| **perforado** (con aproximación) | 5 | **0.009 mm** | 0.112 mm |
| **contorno** (con aproximación) | 5 | **0.008 mm** | 0.251 mm |
| *`ruta_nodo`, perforado (RUTA.md §5)* | — | *0.69 mm* | *4.60 mm* |
| *`ruta_nodo`, contorno* | — | *0.79 mm* | *2.83 mm* |

**Por qué tanto mejor que `ruta_nodo`:** allí cada junta va 0.5–1° por detrás de
su consigna por culpa de su PID de posición, y no hay nada por encima que lo
corrija. Aquí la junta hace la velocidad pedida y el lazo de la punta corrige lo
que quede.

**Pero ojo:** Gazebo con juntas por velocidad es casi la planta ideal de la
teoría. El brazo real tiene inercia, rozamiento y un motor que tarda en
responder. Estos números son el **techo**, no lo que va a dar la mesa.

---

## 6. Siete cosas que se encontraron al medirlo

1. **La cola de `/joint_states`, de uno.** gz lo manda cada milisegundo. Con una
   cola larga, el lazo leía medidas atrasadas, y el retraso dentro del lazo es
   lo que limita K. Con cola de 1, el error de la aproximación bajó de 0.023 a
   0.007 mm.

2. **El error, contra la referencia del instante en que se midió.** Se usa el
   sello de tiempo de gz, no la hora de llegada. Comparar una medida de hace uno
   o dos tics con la consigna de *ahora* hace creer que la punta va v·retraso
   por detrás, y el lazo la adelanta justo eso. Es el mismo fallo de la guía que
   se corrigió en `ruta.py` (RUTA.md §3, punto 9). Corregido, desaparecieron
   los picos de 0.9 mm de un solo tic. Entre este arreglo y el anterior, el
   error medio del círculo bajó de 0.060 a 0.008 mm.

3. **El nodo a veces se queda parado 0.3–0.4 s y gz no espera.** Pasa en este
   portátil al entrar o salir otro participante de DDS, por ejemplo cada
   `ros2 topic pub --once`. El puente sigue entregando `/joint_states` sin
   huecos; es el proceso de Python el que se para. Durante el parón las juntas
   siguen con la última velocidad mandada, y la referencia, con el reloj.
   - **Antes:** al arrancar el perforado el brazo estaba quieto, la referencia
     se fue 56 mm por delante y luego saturó para alcanzarla.
   - **Ahora:** al volver, el nodo pone la referencia en el punto del camino más
     cercano a la punta medida. Si el parón lo pilló siguiendo el camino, no
     cambia nada; si lo pilló quieto, la referencia le espera. Queda en el log:
     `el nodo estuvo parado 0.41 s (gz no espera): la referencia se recoloca…`.
     El máximo del perforado lanzado justo después del círculo bajó de 56.8 a
     0.97 mm.

4. **El vigilante.** gz se queda con la última velocidad que recibió, así que si
   `/joint_states` deja de llegar 0.25 s (de gz) **seguidos**, el nodo manda
   ceros. Un tic suelto con medida vieja no cuenta: es el primer tic después de
   un parón, que llega antes que la medida nueva, y ahí el nodo no manda nada.
   Al cerrar con Ctrl+C también manda ceros (punto 5).

5. **(15 oct.) Ctrl+C no paraba las juntas.** Con los manejadores de señales
   de rclpy, Ctrl+C apaga ROS *antes* de llegar al `finally` del nodo. Los
   ceros ya no salían y gz seguía moviendo el brazo con la última velocidad:
   q1 avanzó 0.09 rad en los 2 s siguientes a cerrar solo el nodo. Ahora el nodo
   pone sus propios manejadores de SIGINT y SIGTERM, que solo levantan una
   bandera. El bucle sale entre dos callbacks y manda los ceros con ROS aún
   vivo, esperando a que el puente los confirme. Medido: 0.0000 rad después de
   cerrar, con SIGINT y con SIGTERM.

6. **(15 oct.) El «Gazebo colgado al relanzar» eran servidores huérfanos.**
   `ros_gz_sim/gz_sim.launch.py` arranca gz como `sh -c "ruby gz sim …"`. Al
   cerrar, el SIGINT del launch le llega al `sh` y no al servidor. A los 5 s el
   launch mata el `sh` y **`gz-sim-main` se queda vivo**, comiendo CPU. Se
   contaron tres tras tres cierres. El siguiente arranque choca con él
   («Another world of the same name is running») y `create` se puede quedar
   pidiendo la lista de mundos. Además, roban CPU a la simulación.
   `control.launch.py` arranca ahora gz **sin shell**: ruby hace `exec` de
   `gz-sim-main`, el SIGINT le llega a él y se cierra en una décima. Probado:
   cuatro arranques y cierres seguidos, sin huérfanos, y el robot aparece en
   4–5 s cada vez. El cierre pasa de 5.3 s a 1 s. `gazebo.launch.py` (el de
   siempre) sigue como estaba y tiene el mismo problema; no se ha tocado.

7. **(15 oct.) El parón falso después de cada orden.** El reloj de gz le llega
   al nodo por el mismo ejecutor que el tic. Mientras se prepara una tarea
   (~0.1 s: recorrerla con la IK, ajustar velocidades) el reloj no se lee, y
   pedir la hora después devuelve la de antes. Ese tiempo aparecía de golpe en
   el tic siguiente y contaba como parón: «el nodo estuvo parado 0.09 s» en
   cada orden, aunque no entrara nadie en DDS. Ahora la tarea empieza a contar
   en el tic siguiente al que la preparó, y ese tic no cuenta como parón. Los
   parones de verdad (punto 3) se siguen recolocando.

---

## 7. Límites conocidos

- **Solo Gazebo.** Para llevarlo al brazo, la placa tendría que aceptar
  velocidades de junta. El MicroPython de Sergio ya es un lazo de velocidad en
  RPM, así que encajaría. Pero no baja de 10 RPM y estas tareas piden de 0.1 a
  1 RPM en hombro y codo (RUTA.md §8). Sin probar.
- **Si el nodo muere de golpe** (`kill -9`, un fallo de Python), gz sigue con
  la última velocidad hasta el tope de la junta. Con Ctrl+C o al cerrar la
  ventana sí se paran (§6.5). En simulación no rompe nada.
- **La rama del codo no cambia.** Cada tarea se hace en la rama en que esté el
  brazo, la `down` desde el reposo. Cambiarla obliga a pasar por el brazo
  estirado.
- **Sin orientación de la herramienta** (3 GDL) **y sin obstáculos.** Solo se
  comprueban alcance, topes y singularidad.
- **Un salto grande no es la teoría.** Desde el reposo, `salto 330 20 140` son
  165 mm, 44 de ellos en z: satura 28 veces y tarda 6 s, casi todo en rampa. Para
  ver la exponencial, saltos de unos 15 mm en el plano.
- **Una orden que no llega**, solo con `ros2 topic pub --once`: una vez, de
  unas cuarenta, se perdió por el camino. Si no pasa nada, se vuelve a mandar.
  Desde la ventana no pasa (§4, «La ventana»).
- **El puente se cierra mal.** Al cerrar, `parameter_bridge` tarda 5 s y muere
  con *segfault* (exit −11). Pasa igual con `gazebo.launch.py`; no es de
  ningún nodo nuestro y no deja nada abierto.
- ~~Gazebo colgado al arrancar~~: era un servidor huérfano del cierre anterior.
  Corregido (§6.6).

---

Ver también: [RUTA.md](RUTA.md) (la misma ley, integrada sobre el modelo),
[TRAYECTORIAS.md](TRAYECTORIAS.md) (`tray`) y [NODOS.md](NODOS.md) (los seis nodos).
