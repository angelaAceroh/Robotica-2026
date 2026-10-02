# Firmware de la ESP32 — SCARA `scara_urdf`

El PC resuelve la cinemática y la ESP32 cierra el lazo. Aquí no hay
cinemática: la placa recibe tres números de junta por USB y se ocupa de que
los ejes lleguen a ellos, que es justo el trabajo que en Gazebo hacen los
`JointPositionController`.

```
panel / RViz / Gazebo ──▶ scara_ik ──▶ esp32_bridge ──USB──▶ ESP32 ──▶ motores
                                            ▲                             │
                                            └────── posición medida ──────┘
```

Motores: **JGA25-370 12 V 60 RPM con encoder** en las tres juntas.

**Son 3 GDL: la muñeca (`manita`) no está construida y el firmware no la
contempla.** En ROS eso se activa con `config/scara_urdf3.yaml`
(`has_wrist: false`), que es lo que carga `hardware.launch.py` por defecto. El
URDF sigue siendo el de cuatro juntas: `manita` simplemente no aparece en
`/joint_states` y el eslabón `Ian` se dibuja en su cero.

---

## 1. Conexionado

Cada motor lleva el nombre del **eslabón que lo aloja**, no del que mueve. Por
eso el motor de la base ("Alvin") es el que gira el hombro y el orden no es el
que sugieren los nombres:

| j | junta | motor | IN1 | IN2 | ENA (PWM) | ENC A | ENC B |
|---|---|---|---|---|---|---|---|
| 0 | `juntura1` (hombro) | Alvin | 16 | 4 | 17 | 26 | 25 |
| 1 | `juntura2` (codo) | Simón | 19 | 18 | 5 | 12 | 13 |
| 2 | `vertiacal1` (husillo) | Teodoro | 22 | 21 | 23 | 14 | 27 |

**Simón va con un solo canal.** En las pruebas `A` del 2026-09-11, con 12 V, el
canal del GPIO 12 cambiaba ~22 veces en 0.2 s y el del 13 ninguna: ese canal no
llega. En el firmware es `PINES_J2 19, 18, 5, SIN_PIN, 12` (media resolución, y
el sentido lo pone el motor). **Alvin** contaba al revés de lo que empuja, así
que su `sentido` es −1. **Simón** giraba en horario con u + y el URDF lo quiere
antihorario visto desde arriba: lleva el motor invertido (`invMotor`, lo mismo
que `I 1 -1`). Comprobado a ojo: con ▶ (u +) Alvin y Simón giran en antihorario
y Teodoro sube.

Quedan libres **32, 33, 34, 35, 36 y 39** para finales de carrera o la pinza
(34–39 sólo valen como entrada). Evitados: 1/3 (el USB) y 6–11 (la flash).

### Los cuatro avisos que de verdad importan

1. **GPIO 12 es pin de *strapping* (MTDI), y en esta placa está neutralizado
   por eFuse.** Al soltarse el reset, MTDI fija la tensión de la flash: si está
   en alto, la ESP32 la pone a 1.8 V y una DevKit con flash de 3.3 V no
   arranca. El canal A del encoder de Simón va ahí, y como un encoder hall deja
   su salida donde le pille el imán, la placa arrancaba o no según dónde se
   hubiera parado el eje: la ROM decía `boot:0x33` (lo normal es `0x13`) e
   `invalid header: 0xffffffff`, y esptool, `Flash voltage set by a strapping
   pin: 1.8V`. Un pull-down no lo arregla (la salida del encoder es push-pull)
   y el montaje no deja mover el cable, así que se quemó el eFuse que fija la
   flash a 3.3 V:

   ```bash
   espefuse --chip esp32 -p /dev/ttyUSB0 set-flash-voltage 3.3V   # irreversible
   ```

   Desde entonces **esta** placa ignora el GPIO 12 al arrancar. Con otra ESP32
   sin ese eFuse, el canal A de Simón tiene que ir a otro pin (el 33) o no
   arrancará. Si se pierde un canal A, `SIN_PIN` en su lugar deja la junta con
   un solo canal (el B, con el sentido que marca el motor): media resolución y
   algo de deriva en cada cambio de sentido. En una junta así, si el motor gira
   al revés se arregla con `I j -1`, no con `D`.

2. **Los encoders son de 5 V y el ESP32 no tolera 5 V.** Aliméntalos desde el
   pin de 3.3 V — los hall del JGA25-370 funcionan de sobra — o mete un divisor
   10 k/20 k en cada canal. Conectarlos a 5 V directamente estropea la GPIO.

3. **Los seis pines de encoder de este cableado sí tienen pull-up interno**
   (12/13, 14/27, 26/25), así que no hacen falta resistencias externas aunque
   el módulo sea de colector abierto. Es la ventaja de no haber usado 34–39.

4. **Masa común.** La masa de la fuente de 12 V, la del driver y la de la ESP32
   tienen que estar unidas, o el encoder devuelve basura.

### Driver de potencia

El firmware está escrito para el **L298N con el puente de ENA quitado**: dos
pines de dirección y el PWM en el *enable*.

| u | IN1 | IN2 | ENA |
|---|---|---|---|
| `> 0` | 1 | 0 | duty |
| `< 0` | 0 | 1 | duty |
| freno | 0 | 0 | máximo |

El freno va **por abajo** (los dos bornes del motor a masa), no por arriba
(IN1 = IN2 = 1, los dos a +12 V). En el banco de pruebas
(`~/Arduino/prueba_teodoro`), con los IN de Simón o de Alvin a 1, Teodoro no se
movía nada; con ellos a 0, sí. Los dos frenan igual.

`PWM_FREQ` está en **8 kHz**: el L298N es lento y a 20 kHz sólo calienta. Si
algún día se cambia a un TB6612 o un BTS7960, ata su pin de *enable* (STBY /
R_EN+L_EN) a 3.3 V fijos, deja el PWM en ENA igual y sube `PWM_FREQ` a 20000
para salir del rango audible.

Si acabas usando otra topología, las dos únicas funciones que hay que
reescribir son `escribirMotor()` y `frenarMotor()`.

---

## 2. Compilar y cargar

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 scara_esp32
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 scara_esp32
```

o abriendo `scara_esp32/scara_esp32.ino` en el IDE con la placa **ESP32 Dev
Module**. Necesita el core `esp32:esp32` 3.x (el firmware usa la API `ledcAttach`
por pin, que no existe en la 2.x).

---

## 3. El arranque en frío, y por qué es así

La placa arranca **deshabilitada y sin cero**, siempre. No es prudencia
excesiva: los encoders son **incrementales**, así que al encender la ESP32 no
tiene ni idea de dónde está el brazo. Si se le diera potencia sin más, creería
estar en la pose de referencia y saldría corriendo hacia la primera consigna
desde una posición equivocada.

La secuencia es:

1. Coloca el brazo **a mano** en la pose de referencia del URDF (los dos
   eslabones alineados según su cero, el husillo abajo).
2. `Z` — fija esa pose como origen.
3. `E 1` — da potencia.

Desde ROS eso son dos llamadas de servicio:

```bash
ros2 service call /scara/hw/zero   std_srvs/srv/Trigger
ros2 service call /scara/hw/enable std_srvs/srv/SetBool "{data: true}"
```

El puente **rechaza** habilitar sin cero, y el firmware también: son dos
comprobaciones a propósito, porque es el error que más caro sale.

> Al deshabilitar, los motores quedan **frenados**, no sueltos. Con el eje
> vertical no hay alternativa: la reductora del JGA25-370 no es autoblocante y
> soltar el motor deja caer la herramienta.

**Hay que repetir la secuencia cada vez que el puente se conecta.** Medido con
el CH340 de esta placa: abrir el puerto levanta DTR/RTS y dispara el circuito de
auto-reset, así que la ESP32 arranca de cero. El puente intenta evitarlo (baja
las dos líneas antes de abrir) y no lo consigue; desactivar `HUPCL` tampoco.

No es mal comportamiento: un corte de USB deja la placa en un estado seguro que
**obliga a confirmar la referencia**, en vez de seguir moviéndose fiándose de un
cero que quizá ya no vale. Si algún día molesta, las dos salidas son cortar el
condensador de auto-reset de la placa, o montar finales de carrera en los pines
libres (32/33/34/35) y hacer un homing de verdad.

---

## 4. Calibración

Los valores que trae el firmware son **puntos de partida, no medidas**. Hay que
sacar tres números por junta.

### 4.1 Cuentas por unidad

Con la potencia cortada (`E 0`), pon la constante a 1 para que la telemetría
salga en cuentas crudas:

```
C 0 1
```

Gira el eje **una vuelta exacta** a mano y mira cuánto cambia el primer número
de las líneas `>`. Eso es lo que cuenta el decodificador ×4 en una vuelta de la
junta. Luego:

* junta rotativa: `cuentas_por_rad = cuentas_por_vuelta / 6.28319`
* husillo: recorre una distancia medida con calibre (50 mm, por ejemplo) y
  `cuentas_por_metro = cuentas / 0.050`

y se fija con `C j <valor>`. Cuando los tres estén bien, pásalos a
`CUENTAS_VUELTA_EJE`, `REDUCCION_EXTRA[]` y `PASO_HUSILLO` y recompila, para no
tener que repetirlo en cada encendido.

### 4.2 Sentido

Mueve la junta en el sentido **positivo del URDF**. Si la cuenta baja en vez de
subir, invierte el encoder:

```
D 0 -1
```

Si además el motor se va en sentido contrario al que le pides (el brazo se
escapa en vez de acercarse al objetivo), intercambia los dos cables del motor en
el driver. Es más rápido que discutirlo con el software.

> Haz esta comprobación **con el brazo desmontado o los motores en el aire**. Un
> lazo cerrado con el signo cambiado es realimentación positiva: acelera hasta
> el tope.

### 4.3 PWM mínimo

Por debajo de cierto ciclo de trabajo el JGA25-370 no arranca: la fricción de la
reductora se lo come. Busca ese umbral y ponlo con `P j <pwm_min> <pwm_max>`.
Sin él, el brazo se planta a unos grados del objetivo y se queda ahí.

---

## 5. Sintonía del PID

Se ajusta en caliente, sin recompilar, con `K j kp ki kd kv`.

Empieza con `ki`, `kd` y `kv` a cero y sube `kp` hasta que la junta siga la
consigna con un sobreimpulso pequeño. Luego `ki`, hasta que el error en régimen
permanente se vaya. `kd` el último y poco: amplifica el ruido de cuantización
del encoder. `kv` es realimentación anticipativa — como la consigna llega
rampeada y se sabe a qué velocidad va, se le adelanta al motor la tensión que
esa velocidad pide en vez de esperar a que el error la genere; sube mucho la
calidad del seguimiento en las trayectorias de demostración.

Los valores de partida en el sketch son conservadores a propósito.

---

## 6. Protocolo

Líneas de texto terminadas en `\n`. El checksum `*XX` (XOR de la carga, en hex)
es **opcional**: el puente siempre lo manda, pero se puede teclear a mano en el
monitor serie sin calcular nada.

### PC → ESP32

| orden | qué hace |
|---|---|
| `T q1 q2 d3` | consigna de junta (rad, rad, m) |
| `M j q` | lleva sólo la junta `j` a `q` (rad, o m si `j = 2`); las demás, quietas |
| `A j u ms` | prueba en lazo abierto: ciclo `u` (−1 a 1) en la junta `j` durante `ms` (máx. 1000) y cuántas cuentas se movió; sólo con la potencia cortada |
| `I j ±1` | invierte el motor por software, como cruzar sus cables (la única forma de arreglar el sentido en una junta de un canal) |
| `E 0` / `E 1` | corta o da potencia |
| `Z` | fija la pose actual como cero |
| `H` | vuelve a la pose de referencia |
| `K j kp ki kd kv` | ganancias de la junta `j` (0–2) |
| `C j cuentas` | cuentas de encoder por rad (por metro si `j = 2`) |
| `D j ±1` | invierte el encoder |
| `R j qmin qmax vmax` | recorrido y velocidad máxima |
| `P j pwmmin pwmmax` | PWM de arranque y tope, 0–1 |
| `F hz` | frecuencia de telemetría (0 la apaga) |
| `?` | vuelca toda la configuración |

`T` es un **flujo**: el puente la manda a 50 Hz y, si deja de llegar 500 ms, la
referencia se congela donde esté (sección 7). `M` y `H` son **metas**: se
teclean una vez y la junta llega aunque no llegue nada más, que es lo que hace
falta para mover el brazo a mano desde el monitor serie. Las dos exigen haber
dado potencia (`E 1`) y la siguiente `T` las pisa, así que con el puente
conectado no sirven: desde ROS el home se pide a `scara_ik` (`/scara/home`).

### ESP32 → PC

```
> ms q1 q2 q3 u1 u2 u3 flags           telemetría (25 Hz por defecto)
! texto                                 confirmación de una orden
# texto                                 mensaje informativo
```

`u` es el ciclo de trabajo con signo (−1 a 1). Las banderas son un mapa de bits:
`1` habilitado · `2` enlace vivo · `4` cero hecho · `8` alguna junta empujando
contra su tope · `16` el PID está saturando · `32` potencia cortada por bloqueo.

El bit `8` sólo se enciende cuando el lazo **sigue empujando hacia fuera** del
recorrido. Estar en el tope no basta: la pose de referencia deja el husillo justo
en `d3_min`, y avisar de eso sería avisar siempre.

---

## 7. Qué pasa si se corta el USB

Ojo con la distinción: esto cubre el caso de que el puente **deje de mandar
consignas con el puerto abierto** (el lado de ROS se atasca, `/joint_states` se
para). Si lo que se cae es el USB entero, al reconectar la placa se reinicia y
hay que volver a referenciar, como se explica arriba.

Pasados 500 ms sin recibir una `T`, el firmware **congela la referencia donde
esté y sigue sosteniendo el brazo**. No corta la potencia: soltar los motores
por perder el cable sería la peor reacción posible, porque el eje vertical se
viene abajo. La bandera de enlace se apaga y el puente lo reporta en
`/scara/hw/ok`.

El puente, por su parte, manda `E 0` al cerrarse, para no dejar los motores
alimentados sin nadie mirando.

---

## 8. Protección de bloqueo y de sentido invertido

Si una junta lleva **1.5 s saturada sin haberse movido**, el firmware corta la
potencia de todo el brazo y lo dice:

```
! BLOQUEO en j0 (juntura1): saturada 1500 ms sin moverse, potencia cortada.
```

Cubre el encoder desconectado y el eje agarrotado: en los dos, el PID ve un
error que nunca se cierra y empuja al 85 % indefinidamente — o sea, un motor
calentándose hasta que algo cede. El margen es holgado a propósito: una junta
sana recorre metros en 1.5 s, así que no salta por un roce.

El **signo del lazo invertido** necesita otra comprobación, porque ahí el motor
sí se mueve: se aleja de la consigna, a toda potencia, y el bloqueo sólo lo
vería al llegar al tope mecánico (en el husillo, la tuerca contra el final del
tornillo). Por eso, si una junta saturada avanza **10 bandas muertas en contra**
de lo que se le empuja —3 mm en el husillo, 2.3° en las rotativas— también se
corta:

```
! SENTIDO INVERTIDO en j2 (vertiacal1): empujando a tope se aleja de la consigna...
```

Sólo se mira con la junta saturada, así que antes de cortar habrá recorrido del
orden de un centímetro en el sentido malo. Con la potencia cortada, mueve la
junta a mano en su sentido positivo: si `q` baja, el que está al revés es el
encoder (`D j -1`); si sube, son los cables del motor. Una de las dos, no ambas.

Se rearma llamando otra vez a `/scara/hw/enable`, pero si la causa sigue ahí
volverá a saltar a los dos segundos. La bandera `32` de la telemetría marca el
fallo.

---

## 9. Límites reales del hardware

El `max_vel` del YAML de `scara_kinematics` está pensado para la simulación.
Con estos motores el techo real es otro, y `hardware.launch.py` se lo impone
tanto a `scara_ik` como a la placa:

| junta | YAML (simulación) | hardware | por qué |
|---|---|---|---|
| `juntura1` | 1.5 rad/s | 1.5 rad/s | 60 RPM son 6.28 rad/s; sobra, salvo que metas mucha reducción |
| `juntura2` | 1.5 rad/s | 1.5 rad/s | ídem |
| `vertiacal1` | 0.10 m/s | **0.008 m/s** | 1 vuelta/s × 8 mm de paso = 8 mm/s |

El husillo es **doce veces más lento** de lo que supone la simulación: recorrer
sus 15 cm de carrera lleva unos 19 segundos. Si no se corrigiera, `scara_ik`
mandaría una rampa que el brazo no puede seguir y RViz enseñaría una pose que el
robot todavía no ha alcanzado. Si le pones más reducción o un husillo de más
paso, actualiza `MAX_VEL_HW` en `launch/hardware.launch.py`.

---

## 10. Mover a Teodoro solo, sin ROS

Para probar el husillo antes de cablear los otros dos motores, o para
calibrarlo, no hace falta levantar ROS:

```bash
ros2 launch scara_kinematics teodoro.launch.py   # la ventana, con Gazebo copiando al real
ros2 run scara_kinematics teodoro_gui        # con ventana y botones
ros2 run scara_kinematics teodoro            # en la terminal; busca el puerto solo
ros2 run scara_kinematics teodoro --port /dev/ttyUSB0
```

`teodoro_gui` dibuja el husillo a escala —un clic lo manda a esa altura—,
enseña lo que mide el encoder con una gráfica de los últimos 20 s y tiene
botones para el cero, la potencia y pasos de 1 y 10 mm (también con las
flechas); Esc corta la potencia. `teodoro` es lo mismo en la terminal: `z` fija
el cero, `on`/`off` dan y quitan potencia, `50` lleva el husillo a d3 = 50 mm,
`+5`/`-5` lo mueven 5 mm, Enter dice dónde está y `q` corta la potencia y sale.
En los dos, lo demás (`?`, `K`, `C`, `D`, `P`…) pasa tal cual a la placa, y
ninguno puede correr a la vez que `esp32_bridge`: abren el mismo puerto.

Lo mismo a pelo, desde el monitor serie del IDE (115200 baudios, fin de línea
`\n`):

```
Z            con el husillo abajo del todo (d3 = 20 mm)
E 1
M 2 0.05     sube a d3 = 50 mm
M 2 0.02     vuelve abajo
E 0
```

Pines de Teodoro: IN1 **22**, IN2 **21**, ENA **23** (PWM), encoder A **14** y
encoder B **27**. Vale cualquiera de los dos canales del L298N siempre que las
tres señales sean del mismo: IN1/IN2/ENA, o IN3/IN4/ENB.

La primera vez, limita el tope de PWM (`P 2 0.15 0.5`): si el signo del lazo
está cambiado, llegará menos lejos y con menos fuerza antes de que la
protección de la sección 8 corte.
