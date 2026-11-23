# Control cinemático en el brazo de verdad (`control_placa`)

Añadido del **21 de octubre de 2026**. Es el control cinemático de
[CONTROL.md](CONTROL.md) (q̇ = J⁻¹(ẋd + K·e), cerrado con lo que se **mide**)
llevado del Gazebo por velocidad a la ESP32 y sus tres motores. Primero se probó
contra una placa simulada que ejecuta el firmware de verdad (el §4 es la lista
de lo que hay que mirar la primera vez). **El 21/10 por la tarde se probó en el
brazo**: recorridos y sentidos que dio Sergio, el PWM de arranque de cada motor
medido y la fase 1 de `control_puntos` entera. Ver el §9.

**Revisado esa misma tarde con los programas que mueven el brazo** (`Alvin.py`,
`Simon.py` y `Teodoro.py`, de Sergio): los pines, los pulsos por vuelta, la
transmisión de Simon, el piñón de Teodoro y los recorridos son ahora los suyos.
Lo que cambió, y por qué, está en el §8.

**Qué se ha tocado:** nada de lo que funcionaba. `control_nodo` y su ventana, el
`main.py` de `vector`, los seis nodos, los URDF y los YAML que había están
igual. Lo añadido:

| archivo | dónde corre | qué es |
|---|---|---|
| `firmware/micropython/main_control.py` | la ESP32 | el otro programa de la placa: recibe velocidades y sigue cada motor con un PI de posición |
| `scara_kinematics/puente_placa.py` | el PC (nodo) | hace de Gazebo: velocidades de `control_nodo` → placa, encoders → `/joint_states` |
| `scara_kinematics/control_placa_gui.py` | el PC (nodo) | la ventana de `control_gui`, heredada, puesta sobre el brazo |
| `scara_kinematics/placa_vel.py` | el PC | el protocolo con la placa y las cuentas de junta a motor (sin ROS) |
| `scara_kinematics/placa_simulada.py` | el PC | la placa sin brazo: `main_control.py` de verdad con motores simulados |
| `launch/control_placa.launch.py` | | todo junto |
| `config/micropython.yaml` | | de junta a motor y los recorridos, en las unidades de la placa (el mismo de `main.py` y `vector`) |
| `test/test_control_placa.py` | | 16 tests |
| `setup.py` | | +3 líneas en `entry_points` (`puente_placa`, `control_placa_gui`, `placa_simulada`) y +1 en `data_files` (instala `firmware/micropython/*.py`) |

---

## 1. Cómo va

```
 [ventana] --scara/control/orden--> [control_nodo] --q̇ (rad/s, m/s)--> [puente_placa]
                                          ▲                                 │  junta → °/s, mm/s
                                          │                                 ▼
                                    /joint_states              'V va vs vt *HH'  (50 por segundo)
                                          │                                 │ USB
                                    [puente_placa] ◄── 'P ms pa ps pt AAL *HH' ── main_control.py
                                     motor → junta          (50 por segundo)       (la ESP32)
```

- **`control_nodo` es el mismo de Gazebo.** El launch solo le cambia los
  parámetros: manda a `/scara_urdf/placa/cmd_vel/<junta>` en vez de a gz, sus
  topes son los recorridos **medidos** de los motores, el techo de velocidad es
  más bajo y va a 50 Hz con el reloj del PC.
- **`puente_placa`** pasa cada velocidad de junta a lo que entiende la placa
  (grados/s para Alvin y Simon, mm/s para Teodoro) con la calibración de
  `config/micropython.yaml` (el mismo archivo y la misma cuenta que usan `vector`
  y `efector`).
  La manda a la placa en cuanto le llegan las tres juntas del mismo tic. Al revés, convierte lo que miden los encoders a junta y lo publica en
  `/joint_states` **con la hora a la que midió la placa**, no con la de llegada
  (el reloj de la placa se pasa al del PC con el mínimo del desfase). Es lo que
  pide el §6.2 de CONTROL.md.
- **`main_control.py`**, en la placa. Cada 10 ms suma a la consigna de cada
  motor la velocidad que le llega (`ref += v·dt`). La sigue con un PI más
  anticipativa: `pwm = KP·(ref − pos) + integral + KV·v`, con un suelo de
  arranque. Teodoro va sin integral (§9, puntos 11 y 12).

**Las unidades son las de tus programas.** Lo que se teclea y se lee en la placa
va en grados de junta para Alvin y Simon, y en mm para Teodoro: los mismos
números que dan `Alvin.py`, `Simon.py` y `Teodoro.py`. Por dentro, el lazo de
cada motor cuenta en grados del eje del motor (5500 pulsos por vuelta en los
tres), y así valen las mismas ganancias para los tres.

**Por qué no con el lazo de RPM de tus `Alvin.py`, `Simon.py` y `Teodoro.py`.**
El control pide menos de 3 RPM en el hombro y el codo, y ese lazo está probado
entre 10 y 60. Tan
despacio el motor va a tirones por la fricción, y la velocidad que se mide con
el encoder es ruido. En cambio, una consigna de posición que avanza despacio sí
se puede seguir: el motor da un tirón cada vez que la consigna se le adelanta
una décima de grado, y el lazo de la punta, en el PC, corrige lo que quede. Las
ganancias de partida son las del PID de `scara_esp32` que movió el brazo el 11
de septiembre (con 0.01° de error), pasadas a grados y a PWM de 0 a 1023. Aquel
firmware contaba 5440 pulsos por vuelta, casi los 5500 de ahora, así que las
ganancias valen tal cual.

**El husillo es de verdad**: Teodoro se activa con los otros dos y la z sale de
su encoder (41.4 mm por vuelta, los de `Teodoro.py`). Con `husillo:=virtual`,
Teodoro se queda suelto y la d3 que publica `puente_placa` es la que se integra
con la velocidad que pide `control_nodo`: **la z es de mentira** (sale
perfecta) **y el plano, de verdad**. Sirve para probar el plano antes de fiarse
de la z (§4).

---

## 2. Cargar `main_control.py` en la placa

Es otro programa, no un añadido al `main.py` de `vector`: en la ESP32 corre uno
solo.

1. En Thonny, abre `firmware/micropython/main_control.py`.
2. **Mira la tabla `MOTORES`**: los pines, los pulsos y los recorridos son los
   de tus `Alvin.py`, `Simon.py` y `Teodoro.py` del 21/10 (el test
   `test_la_tabla_de_motores_es_la_de_los_programas_de_sergio` lo comprueba). Si
   cambias algo en tus programas, cámbialo aquí también.
3. *Archivo → Guardar como → Dispositivo MicroPython → `main.py`*. Sustituye al
   de `vector`. Para volver a `vector`, guarda otra vez aquel como `main.py`.
4. Reinicia la placa (Ctrl+D en Thonny). Tiene que decir:

```
CONTROL listo: Alvin, Simon, Teodoro (control cinematico: velocidades por el USB)
 Alvin: ultima posicion en flash 8.90 grados, recorrido [-60, 60]
 Simon: ultima posicion en flash -42.98 grados, recorrido [-280, 0]
 Teodoro: ultima posicion en flash -30.38 mm, recorrido [-135, 0]
```

y después una línea `P ...` cincuenta veces por segundo.

**Dónde está cada motor.** `main_control.py` guarda la posición en
`posicion_Alvin.txt`, `posicion_Simon.txt` y `posicion_Teodoro.txt` (en pulsos),
y solo al **soltar** los motores (`q`, Ctrl+C en el PC, un vigilante). Tus
programas la guardan en `posicion_guardada.txt`, así que **la primera vez la
flash no dice dónde está el brazo**: díselo con `fijar` (§4, paso 2). Ojo:
`posicion_guardada.txt` es el mismo archivo para los tres programas, así que
cada uno arranca con lo que guardó el último que se ejecutó.

### Las órdenes, también a mano desde Thonny

| orden | qué hace |
|---|---|
| `a` / `a alvin simon` | activa los tres / esos. Cada uno se engancha **donde está**, sin moverse |
| `V va vs vt` | velocidades: grados/s de Alvin y Simon, mm/s de Teodoro (en Thonny, sin checksum) |
| `q` | suelta los tres (PWM 0) y guarda las posiciones |
| `fijar simon -35.2` | «Simon está en −35.2». Solo con los motores sueltos |
| `h simon` | home: `fijar simon 0`. **A Alvin no** («NO DARLE A LA H»): si de verdad hace falta, `fijar alvin 0` |
| `?`, `l`, `g` | posiciones, límites y ganancias |
| `g alvin kp 60` | cambia una ganancia en caliente (`kp`, `ki`, `kv`, `arranque`, `arranque_menos`, `tope`, `banda`, `freno`, `anticipo`, `pulso`; las cuatro nuevas, del 24/10, en CONTROL_SIN_TEMBLOR.md). No se guarda |

**Una `V` tecleada a mano es un paso corto.** Como no llega otra en 0.3 s, la
placa para el motor y lo sujeta. Por ejemplo, `V 5 0 0` mueve a Alvin unos
+1.5° y lo deja quieto, y `V 0 0 -2` sube a Teodoro unos 0.6 mm. A los 3 s sin órdenes suelta los motores. Es la forma
segura de probar cada motor sin el PC (§4).

---

## 3. Arranque

Cierra Thonny antes: el puerto solo lo puede tener uno.

```bash
cd ~/ros2_ws && colcon build --packages-select scara_kinematics && source install/setup.bash

ros2 launch scara_kinematics control_placa.launch.py                    # el brazo + la ventana
ros2 launch scara_kinematics control_placa.launch.py activar:=false     # los motores sueltos al empezar
ros2 launch scara_kinematics control_placa.launch.py k:=2 vel_max:=0.1  # más suave
ros2 launch scara_kinematics control_placa.launch.py simular:=true      # SIN el brazo
ros2 launch scara_kinematics control_placa.launch.py rviz:=true rqt:=true registro:=~/placa.csv
```

| argumento | por defecto | |
|---|---|---|
| `puerto` | vacío = la busca | |
| `simular` | `false` | `true` = `placa_simulada` en vez de la ESP32 |
| `fallo` | vacío | con `simular`: `alvin_inv` o `simon_sin_encoder` |
| `k` | `3.0` | K [1/s]. En Gazebo era 5; aquí se empieza más suave |
| `vel_max` | `0.2` | techo de hombro y codo [rad/s]: unos 90 mm/s en la punta |
| `vel_z` | `1.0` | techo de Teodoro [mm/s]. Hace unos 2 mm/s; con los 8 de antes se quedaba atrás y la placa soltaba los motores (§9) |
| `activar` | `true` | engancha los motores en cuanto la placa contesta |
| `husillo` | `auto` | `virtual`: Teodoro suelto y la z de mentira (para probar el plano) |
| `margen` | `0.5` | grados (Alvin, Simon) o mm (Teodoro) que `control_nodo` deja antes de cada tope |
| `tarea` | `nada` | la primera orden de `control_nodo` |
| `ventana`, `rviz`, `rqt` | `true`, `false`, `false` | |

**Al arrancar**, en cuanto la placa contesta, `puente_placa` hace tres cosas:

1. Compara los límites de la placa con los del YAML («coinciden» o «LIMITES
   DISTINTOS»).
2. Le dice `parar` a `control_nodo`, para que regule donde está el brazo ahora.
3. Medio segundo después manda `a` (o `a alvin simon` con el husillo virtual).

Los motores se enganchan **donde están**: el brazo no se mueve hasta la primera
orden.

**Las órdenes** son las de CONTROL.md §4 (`ir`, `salto`, `circulo`, `empujon`,
`k=`, `parar`), desde la ventana o a mano, y además las de la placa:

```bash
ros2 topic pub --once /scara/placa/orden std_msgs/String "data: 'soltar'"    # q: motores sueltos
ros2 topic pub --once /scara/placa/orden std_msgs/String "data: 'activar'"
ros2 topic pub --once /scara/placa/orden std_msgs/String "data: 'g simon kp 60'"
```

**No arranques con un motor justo en su tope.** Pasa siempre que sales de tu
HOME, porque allí Simon está en 0, que es su tope (y Teodoro en 0 después de una
`h`). `control_nodo` deja `margen` antes de cada tope, así que ve el brazo fuera
de sus topes y rechaza cualquier `ir`, incluso hacia dentro: «theta2 = -95.4 deg
fuera de [-94.9, 150] deg». Antes de lanzar, mételo un poco hacia dentro desde
Thonny: `a simon`, dos o tres `V 0 -3 0` (−1° cada una) y `q`.

**Lo que cabe**, con los recorridos del 21/10 y el cero en tu HOME (§9): la zona
sombreada de la ventana. Es la rama del codo en la que está el brazo: x de −248
a 498, y de −498 a 207, z de 86 a 220 mm. El círculo por defecto (`circulo` sin
números) está calculado para caber: R 15 en (20, −371). Si el cero de algún
motor no es el de verdad, la zona gira (§4, paso 4).

### La ventana

Es la de Gazebo (`control_gui`), heredada, con estos cambios:

- **La planta** enseña la franja que alcanza el brazo de verdad, y los campos
  empiezan dentro de ella.
- **En la cabecera**, «Pausar Gazebo» pasa a ser **Soltar motores / Activar
  motores**. **Parar (Esc)** sigue siendo el de `control_nodo`: se queda
  regulando donde está y los motores siguen sujetando.
- **En «Ahora»** hay dos filas más:
  - `placa`: ACTIVA o SUELTA, los motores activos (`AAA`), a cuántos Hz llega la
    telemetría y si el husillo es real o virtual;
  - `motores`: dónde está cada uno (grados, grados, mm) y, entre paréntesis,
    cuánto va por detrás de su consigna.
- **En los mensajes** salen también los de `puente_placa`, por ejemplo los
  vigilantes de la placa.

Pide 1273×695 píxeles, así que cabe en la pantalla de 1366×768.

---

## 4. La primera vez con el brazo

En este orden. Cada paso se puede parar con **Ctrl+C** o con **Soltar
motores**. Las dos sueltan los motores y guardan la posición.

1. **Los 12 V puestos.** El LED del L298N no lo demuestra; hay que medir el
   borne.
2. **En Thonny, con `main_control.py` cargado y los motores sueltos:**
   - `l` tiene que dar `-60 60`, `-280 0` y `-135 0`.
   - `?` dice dónde cree que está cada motor. **La primera vez no lo sabe** (tus
     programas guardan en otro archivo). Díselo con lo último que marcó cada
     uno de tus programas: `fijar alvin 37.5`, `fijar simon -120`,
     `fijar teodoro -40`.
3. **Cada motor, solo, con pasos cortos.** `a alvin` y luego `V 5 0 0` varias
   veces: Alvin tiene que avanzar +1.5° en cada una y quedarse quieto. Si sale
   `ERROR Alvin va al reves`, cambia IN1 por IN2 en la tabla `MOTORES`. Después
   `q`, y lo mismo con `a simon` / `V 0 5 0` y con `a teodoro` / `V 0 0 -2`
   (−0.6 mm cada vez).
4. **La calibración**, en `config/micropython.yaml`. Son **supuestos**, sacados
   de comparar tus pines con las pruebas del 11 de septiembre:
   - **`signo`**: con `V 5 0 0`, el hombro tiene que girar en **horario** visto
     desde arriba (`signo` −1); con `V 0 5 0`, el codo en **antihorario** (+1);
     con `V 0 0 2`, Teodoro tiene que **bajar** (−1). Si alguno va al revés,
     cambia su signo.
   - **`cero`**: dónde está cada junta con su motor en 0. Se supone que con
     Alvin y Simon en 0 el brazo está **estirado** sobre el eje x del robot, y
     que con Teodoro en 0 la punta está **abajo del todo**, a d3 = 20 mm (z =
     85.5 mm). Si no, pon el ángulo de verdad (grados, antihorario positivo) o
     la d3 de verdad (z de la punta − 65.5 mm).
   - Después, `colcon build --packages-select scara_kinematics`.
   - **Si el signo o el cero están mal, el lazo no se vuelve inestable**: el PC
     calcula con la misma calibración en los dos sentidos. Lo que pasa es que la
     punta del modelo y la de verdad no coinciden (la figura sale girada o en
     espejo).
5. **Cierra Thonny** y lanza con los motores sueltos y la z de mentira:
   `ros2 launch scara_kinematics control_placa.launch.py activar:=false k:=2 vel_max:=0.1 husillo:=virtual`.
   Mueve el brazo a mano y comprueba que la planta y la fila `motores` le siguen.
6. **Activar motores.** El brazo tiene que quedarse quieto, sujetando. Si tiembla
   o zumba, ve a «Sintonía», abajo.
7. **Un salto pequeño**, de unos 5 mm en el plano, dentro de la zona sombreada.
   En los mensajes sale la τ medida; con K = 2, la teoría es 0.5 s.
8. Luego **Ir en recta** a 20 mm/s y un **Círculo**.
9. **La z de verdad**: otra vez sin `husillo:=virtual`, y un salto de 2 mm solo
   en z. Después, subir K a 3 o 5 y `vel_max` a 0.2.

### Sintonía

Las ganancias se cambian en caliente con `g`, desde Thonny o desde el PC
(`/scara/placa/orden`). La fila `motores` de la ventana dice cuánto va cada
motor por detrás de su consigna, y `scara/placa/seguimiento` y
`scara/placa/pwm` lo mismo en rqt.

| lo que pasa | qué tocar |
|---|---|
| quieto, tiembla o zumba | `banda` más grande (0.15–0.2) o `arranque` más pequeño |
| `ERROR … no se ha movido` sin que nada lo bloquee | ni subiendo hasta `tope` se ha movido: mira los 12 V, o `tope` más grande |
| va a tirones, cada arranque es un salto | `arranque` más pequeño: si no basta, la placa lo sube sola hasta `tope` (§9) |
| al seguir, se queda más de 0.5° por detrás | `kv` o `kp` más grandes |
| se pasa de la consigna, oscila | `kp` o `ki` más pequeños |
| quieto, se queda siempre a un poco de la consigna | `ki` más grande |
| `ERROR … va al reves` justo al frenar, y el motor **no** está al revés (rueda por inercia más de lo que se le perdona) | `GRADOS_AL_REVES` o `INERCIA_S` más grandes, en `main_control.py` |

Lo que se cambia con `g` **no se guarda**: cuando funcione, cópialo a la tabla
`GANANCIAS` de `main_control.py` y vuelve a cargarlo.

---

## 5. Los vigilantes

La placa cierra el lazo de cada motor, así que un encoder que no cuenta o un
motor al revés no se quedan en «no llega»: el motor se embala. Hay vigilantes
en la placa y en el PC. Los de la placa van en grados del eje del motor: en
Alvin son los de la junta, en Simon 7/6 de los de la junta, y en Teodoro 1 mm
son 8.7°.

| dónde | qué | qué hace |
|---|---|---|
| placa | empujando hacia un lado, va hacia el otro 0.5° más lo que lleve de inercia (velocidad × 0.05 s): motor al revés de su encoder | suelta los tres: `ERROR <motor> va al reves` |
| placa | empujando y sin moverse ni 0.1°: el PWM sube de ARRANQUE a TOPE en 0.2 s | si a los 0.25 s sigue sin moverse (encoder muerto, sin 12 V, bloqueado), suelta los tres: `ERROR <motor> … no se ha movido` |
| placa | la posición se separa más de 3° de la consigna | suelta los tres |
| placa | la consigna llega a MIN o MAX | no sigue hacia fuera, y vacía el integral para no pasarse |
| placa | la consigna se adelanta más de 1.5° a la posición | la consigna espera al motor |
| placa | 0.3 s sin una `V` buena | velocidad 0: quietos, sujetando (`VIGILANTE`) |
| placa | 3 s sin una `V` buena | suelta los motores y guarda (`PARADO`) |
| placa | una `V` con checksum malo | se tira (llega otra en 20 ms) |
| placa | `a` con un motor a más de 2° fuera de su recorrido | no activa nada (`RECHAZADO`) |
| PC, `puente_placa` | `control_nodo` no manda velocidades en 0.2 s | manda ceros: quietos, sujetando |
| PC, `puente_placa` | la placa suelta los motores (`PARADO`) | le dice `parar` a `control_nodo`. Para seguir hay que pulsar **Activar motores**: no se reactivan solos |
| PC, `control_nodo` | `/joint_states` no llega en 0.25 s | manda ceros (CONTROL.md §6.4) |
| PC, `control_nodo` | una tarea que se sale de los recorridos medidos | la rechaza antes de mover |

**Probados en la placa simulada:**

- **Alvin al revés**: se para tras 0.65° en el sentido contrario, unos 3 mm en
  la punta. Con el vigilante de 3° solo, eran 24 mm.
- **El encoder de Simon muerto**: se para a los 0.25 s.
- **Contra el tope a toda velocidad**: Alvin (15 °/s) se pasa 0.20° y Teodoro
  (10 mm/s) 0.06 mm. Antes de perdonar la inercia, Teodoro disparaba el
  vigilante de «va al revés» al rebotar en el tope.
- **Ctrl+C en el PC**: la placa queda `LLL`, con los tres sueltos.
- **El PC se calla**: la placa sujeta 3 s y luego suelta.

---

## 6. Lo medido en la placa simulada

`control_placa.launch.py simular:=true`. Es `main_control.py` sin tocar, con
`placa_simulada` debajo:

- un `machine` de mentira;
- motores con inercia, fricción (no arrancan por debajo del 10 % de PWM) y
  reductora que frena en seco;
- encoder en cuadratura de 5500 pulsos por vuelta (los pines y los pulsos los
  lee de la tabla `MOTORES` del propio `main_control.py`);
- un puerto serie falso.

Como en Gazebo, **es un techo**: el brazo de verdad tiene holguras y tirones que
aquí no están.

Con los tres motores activos y la z de verdad:

| prueba | K | resultado |
|---|---|---|
| salto de 6.7 mm en el plano | 3 | τ **0.333 s** (teoría 0.333) |
| empujón de 4.7 mm (2 de ellos en z) | 3 | τ 0.404 s |
| recta de 33 mm, subiendo 10, a 20 mm/s | 3 | error medio **0.28 mm**, máximo 0.70 |
| círculo R 15 a 10 mm/s | 3 | error medio **0.25 mm**, máximo 0.87 |
| recta de 41 mm, bajando 20 | 5 | error medio 0.26 mm, máximo 1.56 |
| enlace | | 50 Hz, ninguna línea estropeada |

Los saltos más grandes, o de más de 3 mm en z con K = 3, piden más que el techo
(0.2 rad/s, 8 mm/s) y caen en rampa: `control_nodo` lo dice («SATURO») y su τ
no vale.

**Comparado con Gazebo**, donde el círculo daba 0.012 mm, aquí es unas 20 veces
peor. Es por los tirones: el motor no se mueve con menos del suelo de arranque,
y cuando se mueve avanza de golpe una o dos décimas de grado. En el hombro, una
décima de grado son de 0.4 a 0.8 mm en la punta.

**Por eso el error no baja de unos 0.5 mm en reposo**, y la medida del salto acaba
diciendo «no llega en 13 s». `control_nodo` espera a que el error baje del
0.2 % del salto, y eso no pasa. La τ que da sí vale: es la de la caída hasta el
37 %.

---

## 7. Límites conocidos

- **Probado en el brazo solo una tarde** (§9). KP, KI y KV siguen siendo las de
  `scara_esp32` pasadas de unidades; ARRANQUE y BANDA sí están medidos.
- **El `cero` de Teodoro es supuesto** (§9): su 0 es su tope de arriba de
  verdad (§9, punto 10), pero que ahí la z sea 220.5 mm no se ha medido. Si no
  lo es, la z del modelo no es la de verdad, pero los motores nunca salen de su
  recorrido, que se vigila en unidades de motor. El de Alvin y Simon sale de tu
  HOME.
- **Simon llega a −280**, pero el CAD lo deja en +150° de codo (Simon −245.4).
- **Alvin va a tirones** cuando va despacio: con PWM 700 de suelo, el plano
  lleva 5–10 mm de error en marcha (§9).
- **Si la placa suelta los motores en mitad de un salto**, la τ que escribe
  `control_nodo` justo después no vale («de 4.88 mm a 0.000 mm»): el `parar`
  pone la consigna donde está la punta. Es cosa de `control_nodo`, que no se ha
  tocado.
- **Si se va la corriente con los motores activos**, la posición no se ha
  guardado: hay que decírsela con `fijar`.
- **Con `husillo:=virtual`**, Teodoro está con PWM 0, igual que cuando lo para
  tu programa.
- **No se lanza con** `brazo.launch.py`, `efector.launch.py` ni
  `control.launch.py`: los dos primeros quieren el mismo puerto, y el tercero
  lanza otro `control_nodo` con el mismo nombre.

```bash
cd ~/ros2_ws && python3 -m pytest src/scara_kinematics/test/test_control_placa.py   # 16 tests, ~12 s
```

---

## 8. Lo que cambió al compararlo con tus programas (21/10, tarde)

Sergio pasó `Alvin.py`, `Simon.py` y `Teodoro.py`, los que mueven el brazo hoy.
Esto es lo que se ha cogido de ellos:

| qué | antes | ahora | por qué importa |
|---|---|---|---|
| IN1 / IN2 | Alvin 16/4, Simon 19/18, Teodoro 22/21 | **4/16, 18/19, 21/22** | con los de antes, Alvin y Simon empujaban al revés de lo que cuenta su encoder: el vigilante habría soltado los motores al primer movimiento |
| ENC_A / ENC_B de Teodoro | 14/27 | **27/14** | con IN1/IN2 cruzados también, el lazo cuadraba, pero su positivo era el contrario que en `Teodoro.py` |
| pulsos por vuelta | 22000 | **5500** (22000/4) | los grados salían 4 veces más pequeños: recorridos, velocidades, vigilantes y ganancias estaban en grados que no eran |
| Simon | 1 vuelta de motor = 1 de junta | **transmisión 21/18** | sin ella, el codo iba un 17 % equivocado |
| Teodoro | sin paso: la z, de mentira | **41.4 mm por vuelta**, en mm | la z es de verdad y Teodoro se activa |
| recorridos | −20..0, −45..0, −30..40 (1/10) | **0..100, −200..0, −100..0 mm** | los de los comentarios de tus programas |
| `h` | ponía a 0 los tres | **`h <motor>`, a Alvin no**, y `fijar` | «NO DARLE A LA H» |
| techo de velocidad | 15 °/s, el mismo para los tres | **15 °/s, 15 °/s, 10 mm/s** | Teodoro necesita unos 70 °/s de motor para los 8 mm/s del control |
| «va al revés» | 0.5° | **0.5° + inercia** | Teodoro a 10 mm/s contra el tope lo disparaba sin estar al revés |
| «se ha ido de su consigna» | 3° | **3° + inercia, que se va gastando** (después de «corrige todo») | Teodoro a 10 mm/s contra el tope se pasa 0.37 mm (3.2° de motor) y soltaba los tres |
| calibración | signo +1, +1, +1 | **signo −1, +1, −1; cero 0, 0, 20 mm** | sacado de los pines (§4, paso 4). Luego, con el brazo, otra (§9) |

**Lo que ya era igual:** el PWM a 1 kHz, la tabla `PASOS` del encoder, el
`Timer` de 10 ms y la función `motor()`.

**Lo que no se ha cogido:**

- **El lazo de RPM** (§1): no sigue velocidades tan bajas.
- **`posicion_guardada.txt`**: es el mismo archivo para los tres programas,
  así que cada uno arranca con lo que guardó el último. Aquí hay uno por motor.
- **La media de 8 RPM**: aquí no se mide la velocidad para controlar.

**Después, con «corrige todo»**, `main.py` (el de `vector`) se puso igual: los
mismos pines, pulsos, unidades y recorridos (un test comprueba que las dos
tablas `MOTORES` coinciden), y `config/control_placa.yaml` se juntó con
`config/micropython.yaml`: ahora hay **un solo archivo de calibración** para los
dos programas. Ver [RUTA.md](RUTA.md) §8.

---

Ver también: [CONTROL.md](CONTROL.md) (el mismo control en Gazebo),
[RUTA.md](RUTA.md) §8 (`main.py`, `vector` y la calibración) y
[NODOS.md](NODOS.md) (los seis nodos).

---

## 9. La primera vez con el brazo (21/10, tarde)

Con `main_control.py` en la placa (subido por el USB y comprobado byte a byte;
la placa no llevaba `main.py`, solo `boot.py` y registros de datos, copiados en
`~/esp32_respaldo/2026-10-21_archivos_placa/`).

**Lo que dijo Sergio** con el brazo delante: está en 0, 0, 0; Alvin va de −60 a
60, Simon de 0 a −280, Teodoro de 0 a −135; nada de pasos rápidos de más de 20
grados; los negativos son en sentido contrario a las agujas del reloj. Y, al
mover Teodoro −5.4 mm, **bajó**. En 0 el codo está doblado.

**La calibración** (`config/micropython.yaml`, el único sitio):

| | antes (supuesto) | ahora | de dónde sale |
|---|---|---|---|
| `limites_motor` | 0..100, −200..0, −100..0 | **−60..60, −280..0, −135..0** | Sergio |
| `signo` | −1, +1, −1 | **−1, −1, +1** | Sergio (Alvin y Simon); Teodoro, visto bajar con −5.4 mm |
| `cero` | 0, 0, 20 mm | **−43.0, −95.38, 155 mm** (desde el 23/10, −45.68, −90.01, 155) | Alvin y Simon: **tu HOME**. Teodoro: **SUPUESTO**, arriba en 0 y abajo del todo (−135) en d3 = 20 mm |

**El cero en tu HOME.** Según el plano de la ventana, HOME está en x 0,
y −250. Antes de eso dijiste que el brazo estaba en (200, −100). Esos dos
puntos están a 250 mm uno del otro, pero los encoders solo se habían movido
+5° y −20°: la placa había perdido cuentas mientras el brazo se movía a mano con
ella reiniciándose. Por eso se puso el cero en HOME. Con el brazo allí (lo
llevaste tú), `fijar alvin 0` y `fijar simon 0`. El `cero` de Alvin y Simon es la
cinemática inversa de (0, −250) en la rama down: θ1 = −43.0° y θ2 = −95.38°,
o sea, el codo doblado 120°. En el modelo el codo está recto con
θ2 = b1 − b2 = 24.62°, que es Simon en −120.

**Desde el 23/10 el HOME está en (0, −270)**, con Teodoro arriba del todo. El
`cero` pasa a ser la cinemática inversa de ese punto: **−45.68°, −90.01°, 155 mm**
(en `config/micropython.yaml` y en los valores por defecto de `alcance.py`), y
con el brazo allí se hace `fijar alvin 0`, `fijar simon 0` y `fijar teodoro 0`.
Simon sigue en su tope en HOME.

En HOME Simon está en 0, que es su tope. Antes de lanzar hay que meterlo un poco
hacia dentro (§3, «No arranques con un motor justo en su tope»).

El codo recto cae dentro del recorrido de Simon, así que el codo podría cruzar
la singularidad. Por eso las zonas (`zona_trabajo`, `contorno_planta`) se
calculan en la rama down, la de HOME, a 10° del codo recto (`A.lado_down`). Los
`signo` comprobados no rompen nada si estuvieran mal; un `cero` mal solo hace
que la punta del modelo no sea la de verdad.

**El PWM de arranque se midió en cada sentido**, con pasos de 1.5° (o 0.6 mm)
cada vez más fuertes, y está en la tabla `GANANCIAS` de `main_control.py`:

| motor | con el de antes | arranca con | banda | por qué la banda |
|---|---|---|---|---|
| Alvin | 123 (12 %): ni se mueve, zumba | **700**, los dos sentidos (660 a veces no) | **0.3°** | con 0.1 no se asienta |
| Simon | 123 | 350 hacia −, **450 hacia +** | **0.3°** | con 0.1 oscila ±350 alrededor de la consigna |
| Teodoro | 150 | 250 bajando; subiendo (el peso), 350 arriba y **450** más abajo | **1.0° de motor (0.12 mm)** | con 0.1 (0.01 mm), al corregir hacia abajo el peso lo pasa y oscila |

Con esas, a 2°/s Alvin y Simon hacen ±6° y Teodoro ±3 mm a 1 mm/s sin
vigilantes.

**Un ARRANQUE fijo no basta en la cremallera.** Teodoro se atascó subiendo dos
veces: con 350 en −39.5 mm y con 450 en −23.4 mm. Lo que pide para arrancar
cambia con la altura. Ahora, si un motor empuja y no se mueve ni 0.1°, el
`main_control.py` sube su PWM de ARRANQUE a TOPE en 0.2 s (`SEG_RAMPA`), y en
cuanto avanza vuelve a ARRANQUE. El vigilante de atasco sigue saltando a los
0.25 s si ni con TOPE se mueve. Con eso la fase 1 salió entera sin un solo
vigilante. **Teodoro no pasa de unos 2 mm/s** (es piñón y cremallera; tus
programas, hasta 60 mm/min): con el techo de 8 mm/s que tenía el launch, se
quedaba atrás y la placa soltaba los tres. Por eso el argumento `vel_z`, a
1 mm/s.

**La fase 1 de `control_puntos`** (siete puntos alrededor de HOME,
CONTROL_PUNTOS.md), con el cero en HOME, `k:=2 vel_max:=0.1`, en el brazo:

| punto | tarda | error medio | máximo | al final |
|---|---|---|---|---|
| P1 | 4.7 s | 2.2 mm | 7.2 mm | 3.53 mm |
| P2 | 16.7 s | 1.3 mm | 7.8 mm | 0.30 mm |
| P3 | 16.7 s | 0.9 mm | 5.0 mm | 1.93 mm |
| P4 | 16.7 s | 1.8 mm | 10.7 mm | 0.53 mm |
| P5 | 16.7 s | 0.8 mm | 4.7 mm | 0.64 mm |
| P6 | 16.7 s | 2.6 mm | 8.3 mm | 0.47 mm |
| P7 | 16.7 s | 0.8 mm | 6.2 mm | 0.69 mm |

Cada tramo tarda 16.7 s porque sube o baja 10 mm a 1 mm/s y `control_nodo`
frena el plano para ir a la par. La z casi no tiene error. El plano oscila
5–10 mm por los tirones de Alvin: su velocidad en la gráfica 4 sale dentada, y
el error en x de la gráfica 2 llega a ±1 cm. La primera vez, con el cero
supuesto de antes y otros puntos, el error medio fue de 0.5 a 3.9 mm. Las
gráficas, en `~/control_puntos_brazo.png` y `.csv`.

**Lo que falló antes de eso**, por orden (y cómo se arregló):

1. `ir` rechazado con Simon en 0: el tope (ver §3).
2. Con 8 mm/s, Teodoro se quedó atrás («se ha ido 0.52 mm de su consigna»): `vel_z`.
3. Teodoro no subía con 250: arranque 350.
4. Simon no giraba hacia + con 350: arranque 450.
5. Cuando la placa soltó los motores, `control_puntos` siguió mandando puntos al
   vacío: ahora escucha a `puente_placa` y corta la fase (CONTROL_PUNTOS.md).
6. La ventana no ponía la punta donde decías: cuentas perdidas al mover el brazo
   a mano. Se puso el cero en HOME (arriba).
7. Teodoro se atascaba subiendo, con 350 y luego con 450, a distinta altura: el
   suelo de arranque ahora sube solo hasta TOPE (arriba). En la placa simulada,
   Teodoro contra su tope a 10 mm/s se pasa ahora hasta 0.14 mm (su banda es
   0.12); el test lo admite.
8. Probando `traycho` (TRAYCHO.md) en la placa simulada, **Teodoro no volvía a
   subir después de bajar**, y sin ningún vigilante. Mientras `control_nodo`
   regula, pide velocidades diminutas, y la consigna de Teodoro iba subiendo
   hasta 1.5° por encima del motor (`ADELANTO_MAX`). Ahí se queda esperándolo.
   El motor, como la consigna no avanzaba, la daba por quieta, y asentado no
   empuja hasta 2 veces su banda: 2.0° desde que su banda es 1.0. Ninguno de los
   dos se movía. Ahora «quieta» es que el PC no pide moverse, no que la consigna
   no avance. Lo comprueba `test_teodoro_sube_despues_de_bajar`, y está subido a
   la placa.
9. `traycho` en el brazo (21/10, noche), primera vez: cortado yendo a P1.
   **Teodoro subió de más**: «se ha ido −0.46 mm de su consigna». Subiendo,
   el integral acumula el empuje que aguanta el peso. Cuando la consigna
   frenaba y el motor ya iba por delante, el suelo de ARRANQUE convertía ese
   empuje en 450 y lo seguía subiendo. Ahora el suelo, la rampa y el vigilante
   de atasco solo actúan cuando se empuja **hacia** la consigna. Si se empuja
   hacia el otro lado, es el integral solo, que sujeta y se va gastando.
10. **El 0 de Teodoro estaba corrido unos 38 mm.** Sergio lo vio subir «al lado
    incorrecto»: estaba a media carrera y la placa creía que estaba mucho más
    arriba. Lo subió a mano hasta arriba del todo y allí se le dijo
    `fijar teodoro 0`. **Desde entonces, el 0 de Teodoro es su tope de arriba de
    verdad.** Lo que sigue siendo supuesto es a qué z corresponde
    (d3 = 155 mm, z = 220.5 mm).
11. `traycho`, segunda vez: cortado bajando hacia P2, con Teodoro en −84 mm.
    Había bajado 2 min y 13 s con menos de 0.2 mm de error. De pronto **se
    embaló hacia abajo a 2 mm/s** y se fue 0.62 mm («se ha ido 0.62 mm de su
    consigna»). Bajando a tirones por el roce de la cremallera, el integral se
    había cargado hacia abajo. Cuando la cremallera rozó menos y el motor se
    pasó de la consigna, ese integral lo seguía empujando hacia abajo. Primero
    se dejó el integral de Teodoro solo hacia arriba, «para aguantar el peso».
12. `traycho`, tercera vez: cortado subiendo hacia P1, en −10.75 mm. Era lo
    mismo, pero hacia arriba: **a 1.8 mm/s** cuando se le pedía 0.8, y 0.45 mm
    por encima. Los registros de las dos veces enseñan además que **Teodoro se
    sostiene solo con el PWM a 0**: el peso no lo baja. Así que su integral no
    aguantaba nada, y el roce ya lo vence el suelo de ARRANQUE con su rampa.
    **Teodoro va ahora sin integral (KI 0 en `GANANCIAS`).** Lo comprueba
    `test_teodoro_no_se_embala_cuando_la_cremallera_se_suelta`, bajando y
    subiendo: va a tirones con roce, la cremallera se suelta de golpe y luego
    roza menos. Con el firmware de la mañana se va 0.72 mm en los dos sentidos;
    con el integral solo hacia arriba, 0.72 mm subiendo; con KI 0, 0.19 mm. Está
    subido a la placa.
13. `traycho`, cuarta vez (sin cortes; TRAYCHO.md §3): en P4 (1/4), **Alvin se
    quedó 10 s quieto al cambiar de sentido**. La punta se quedó hasta 53 mm
    atrás y luego recuperó. Su integral se había cargado en el tramo anterior,
    girando hacia el otro lado, y seguía empujando hacia allí: PWM de −311,
    que fue bajando hasta −23, mientras la consigna lo esperaba 1.5° hacia el
    otro lado (`ADELANTO_MAX`). Así el integral solo se gastaba a
    KI · 1.5 = 27 de PWM por segundo. Ahora **el integral nunca empuja en contra
    de la consigna**: si tiene el signo contrario al error, se pone a 0. Lo
    comprueba `test_alvin_cambia_de_sentido_sin_empujar_al_reves`: con el
    firmware de antes, a los 2 s del cambio todavía empuja hacia el lado de
    antes; con el nuevo, ya ha vuelto 1.5°.

