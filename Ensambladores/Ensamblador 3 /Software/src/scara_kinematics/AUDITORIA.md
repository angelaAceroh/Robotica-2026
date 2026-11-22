# La transferencia de datos, y cómo se comprueba

Lo que se añadió el **20 de septiembre de 2026**: un séptimo nodo, `auditor`,
que no toca nada de la cadena y responde a una sola pregunta —**¿los datos
llegan de un nodo a otro enteros, todos, y a tiempo?**— con números.

No cambia nada de lo que ya había. Los seis nodos, el firmware y los dos
paquetes están como estaban; esto se pone al lado y mira.

> **28/09/2026:** al revisar la comunicación de los nodos, cada muestra de la
> placa pasó a viajar etiquetada y `cc` a publicar un `joint_states` por muestra
> ([NODOS.md §1](NODOS.md#cómo-viaja-una-muestra)). El salto `motores -> cc`
> dejó de casarse por valor y se casa por sello, como el primero; lo de abajo
> ya está al día.

> Va en un archivo aparte por no tocar [`NODOS.md`](NODOS.md). Si prefieres la
> norma de la casa —una sola referencia por paquete—, esto es su sección 13 y
> se funde en un minuto.

- [1. Por qué hace falta](#1-por-qué-hace-falta)
- [2. Arranque](#2-arranque)
- [3. El informe](#3-el-informe)
- [4. Los tres saltos, y qué se comprueba en cada uno](#4-los-tres-saltos-y-qué-se-comprueba-en-cada-uno)
- [5. La ida y vuelta con la placa](#5-la-ida-y-vuelta-con-la-placa)
- [6. La sonda](#6-la-sonda)
- [7. Parámetros](#7-parámetros)
- [8. Diagnóstico](#8-diagnóstico)
- [9. Lo que el auditor NO puede ver](#9-lo-que-el-auditor-no-puede-ver)

---

## 1. Por qué hace falta

En ROS 2 **nadie acusa recibo de nada**. Un nodo publica y se olvida; el de al
lado recibe, o no. Si un mensaje se pierde, llega cambiado, llega dos veces o
llega medio segundo tarde, no se entera ninguno de los dos y desde fuera se ve
como «el brazo va raro».

La cadena del SCARA tiene tres saltos, y en cada uno se puede perder algo
distinto:

```
   ESP32 ──USB──▶ [esp] ──scara/<motor>/aduana──▶ [alvin|simon|teodoro]
                                                         │
                                          scara/<motor>/junta
                                                         ▼
                                                       [cc] ──joint_states──▶
                                                            └─scara/ee_pose──▶
                    ┌──────────────────────────────────────────────┐
                    │  [auditor]  escucha los cuatro, no toca nada  │
                    └──────────────────────────────────────────────┘
```

El auditor sigue cada muestra por los tres saltos y lleva la cuenta. Y como
«que los datos lleguen bien» no es una sola cosa, hace tres:

| | qué | cuándo |
|---|---|---|
| **vigilancia** | cuenta pérdidas, alteraciones, duplicados y latencia salto a salto | siempre, en segundo plano |
| **ida y vuelta** | le pregunta a la placa lo que la aduana le mandó, y compara | a petición, y **no mueve ningún motor** |
| **sonda** | mete una rampa conocida por la cadena y mira si sale idéntica | a petición, y **sólo sin placa** |

---

## 2. Arranque

```bash
cd ~/ros2_ws
colcon build --packages-select scara_kinematics
source install/setup.bash

ros2 launch scara_kinematics auditar.launch.py            # la cadena + el auditor
ros2 launch scara_kinematics auditar.launch.py hw:=false  # sin placa: para la sonda
ros2 launch scara_kinematics auditar.launch.py informe:=~/auditoria.csv
```

Es `brazo.launch.py` tal cual —los seis nodos y la ventana— más el auditor. Sus
argumentos son los de siempre (`port`, `hw`, `ventana`, `rsp`, `registro`) y uno
nuevo, `informe`, que es el CSV donde anota.

Suelto, sobre una cadena que ya esté corriendo:

```bash
P=install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
ros2 run scara_kinematics auditor --ros-args --params-file $P
```

Y para mirarlo:

```bash
ros2 topic echo /scara/auditor/informe              # la tabla, cada 2 s
ros2 topic echo /scara/auditor/ok                   # enganchado: false si algo falla
ros2 service call /scara/auditor/comprobar std_srvs/srv/Trigger
```

---

## 3. El informe

```
AUDITORIA DE LA CADENA     38.3 s vigilando
   salto                      entran   salen perdidos alterados  lat_ms  max_ms   observaciones
OK aduana -> alvin                914     909        0        0     2.2    41.2
OK aduana -> simon                914     909        0        0     1.4    39.3
OK aduana -> teodoro              914     909        0        0     2.4   273.1
OK motores -> cc                 2742    2703        0        0     2.0    40.6  24 descartados en el arranque, 901 joint_states, uno por muestra
OK cc -> punta (ee_pose)          906     906        0        0     1.3    52.1  error FK max 0.00000 mm
veredicto: los datos llegan enteros por los tres saltos
```

| columna | qué es |
|---|---|
| `OK` / `!!` | si ese salto está limpio o no |
| `entran` / `salen` | mensajes que entraron por arriba y salieron por abajo |
| `perdidos` | entraron y **nunca** salieron |
| `alterados` | salieron con un valor distinto del que entró |
| `lat_ms` / `max_ms` | cuánto tarda el dato en cruzar el salto, media y peor caso |
| observaciones | duplicados, colados, desordenados, descartados del arranque, y lo propio de cada salto |

La cuenta cierra siempre: **entran = salen + perdidos + descartados** (más los
dos o tres que estén en vuelo cuando se imprime).

«Descartados en el arranque» no es una pérdida: son los mensajes que la aduana
publicó antes de que el motor —o el propio auditor— llegara a suscribirse. El
emparejado de cada suscripción pasa a su ritmo, así que **un salto no se juzga
hasta haber visto cruzar el primer mensaje**; lo de antes se tira sin contarlo
como culpa de nadie.

---

## 4. Los tres saltos, y qué se comprueba en cada uno

### La aduana reparte, cada motor republica

Se casa **por sello de tiempo**, y se puede porque cada motor (`alvin.py`,
`simon.py`, `teodoro.py`) republica el mensaje *tal cual*: el sello que puso la
aduana sobrevive. Con el sello se sabe
exactamente qué salió de qué, y se detecta lo cuatro:

- **perdido**: entró y no salió en medio segundo;
- **alterado**: comparación **exacta**, sin tolerancia, de nombre, posición,
  velocidad y esfuerzo. Entre nodos el valor viaja sin cuentas por medio, así
  que un solo bit distinto ya es un dato alterado;
- **duplicado**: salió dos veces;
- **desordenado**: salió antes que uno más viejo.

### De los motores a `cc`

También **por sello**. `cc` junta las tres piezas de una muestra y saca **un**
`joint_states` con el sello de esa muestra, así que cada `joint_states` tiene
que traer, exactos, los tres valores que los motores publicaron con ese mismo
sello. Se comprueba lo mismo que arriba —perdido, alterado, duplicado,
desordenado— y además:

| observación | qué significa |
|---|---|
| `colados` | un `joint_states` con un valor que ese motor no publicó con ese sello: `cc` mezcló muestras |
| `piezas de muestras incompletas` | piezas de una muestra a la que le faltó otra antes de llegar a `cc`. No es un fallo de `cc`: hace bien en no publicarla, porque mezclaría instantes. La pieza que falta sale como perdida en su salto de arriba |

Las piezas se apuntan **al llegar**, sin esperar a `t_gracia`: el motor las
publica antes de que `cc` pueda sacar su `joint_states`, así que cuando se juzga
el `joint_states` sus piezas ya están.

Hasta el 28/09/2026 este salto se casaba por valor, porque `cc` remuestreaba a
50 Hz y volvía a sellar. Tenía dos problemas: con valores que se repiten (una
junta que pasa dos veces por el mismo número) casaba con una medida de hacía
segundos y daba latencias absurdas (7 s), y cuando `cc` pasó a publicar al
instante, a veces juzgaba el `joint_states` antes de ver la pieza y daba por
**colado** un valor bueno. El comprobador externo demostró que los valores eran
exactos; era el auditor el que se equivocaba.

### De las juntas a la punta

Esto ya no es transporte: es comprobar que `cc` **usó bien** el dato. Se rehace
la cinemática directa con las juntas de ese ciclo y se compara con la pose que
publicó. `cc` sella las dos mitades igual, así que se emparejan sin ambigüedad,
y como el orden de entrega entre dos topics no está garantizado, se espera a
tener las dos antes de juzgar.

Medido contra la cadena real: **error máximo 0.00000 mm**. Si algún día no
cuadra, o `cc` cambió de fórmula o publicó una pose que no corresponde a sus
juntas.

### Y además: saltos físicamente imposibles

Desde el 28/09/2026 la aduana **comprueba el XOR** de cada línea y tira las
rotas, así que un byte cambiado ya no entra en la cadena. Pero el auditor sigue
mirando la consecuencia —una junta que se mueve más de lo que puede en el
tiempo que ha pasado—, porque delata también las muestras que no llegaron y la
línea rota que por casualidad cuadrase el checksum. Ojo: con el checksum sin
mirar, este techo **no** cazaba un byte cambiado en un decimal pequeño (0.01 rad
de Alvin queda por debajo de `1.5 × 0.04 × 3`): en la prueba del 28/09 se colaron
24 valores falsos y el veredicto fue «los datos llegan enteros».

El intervalo se mide con el **sello**, no con la hora de llegada (dos mensajes
pueden llegar pegados porque el ejecutor los sacó juntos de la cola), y el techo
es `max_vel_hw × dt × margen_salto`. Probado inyectando un salto de 1.7 rad cada
40 muestras: **los caza todos**.

**Un cero no es un salto.** Al fijar el cero (`Z`) con el brazo fuera de su 0,
la lectura salta a 0 de golpe sin que nada se mueva. Hasta el 29/09/2026 eso
daba «3 saltos físicamente imposibles» y el veredicto «HAY DATOS QUE NO LLEGAN
BIEN», una falsa alarma. Ahora la aduana cuenta cada «! cero establecido» de la
placa y lo pone en la etiqueta (`cero=N`): entre dos muestras con distinto `cero`
el auditor no mide salto. Probado contra una placa falsa que cambiaba de origen
0.22 / 0.35 rad / 30 mm: 0 saltos y veredicto «los datos llegan enteros».

---

## 5. La ida y vuelta con la placa

```bash
ros2 service call /scara/auditor/comprobar std_srvs/srv/Trigger
```

```
success=True, message='ida y vuelta correcta en 17 ms: las 3 juntas vuelven
                       con los limites que se les mandaron'
```

Cómo funciona, y por qué prueba algo de verdad: la aduana, nada más abrir el
puerto, le manda a la placa un `R j qmin qmax vmax` por junta con los límites
del YAML. El auditor manda un `?`, que hace que la placa **vuelva a leerlos de
vuelta**, y los compara con los del YAML. Si coinciden:

- el camino de ida —PC → `scara/hw/raw` → aduana → puerto → firmware— está
  probado con datos reales, no con un *ping*;
- el de vuelta —firmware → puerto → aduana → `scara/hw/log`— también;
- y **no se ha movido ni un motor**.

Probado al revés, cambiándole un límite a la placa por detrás:

```bash
ros2 topic pub --once /scara/hw/raw std_msgs/msg/String "{data: 'R 0 -1.5 1.5 1.5'}"
ros2 service call /scara/auditor/comprobar std_srvs/srv/Trigger
# success=False, 'juntura1: mande [-2.0944, 2.0944] y me devuelve [-1.5000, 1.5000]'
```

---

## 6. La sonda

El mismo servicio, cuando **no hay placa**, hace otra cosa: mete por la cadena
una rampa conocida —40 muestras por motor, publicadas en los mismos
`scara/<motor>/aduana`— y certifica que sale idéntica por el otro extremo.

```
success=True, message='sonda correcta: 40 muestras por motor entraron por
                       scara/<motor>/aduana y salieron identicas; cc las paso a
                       joint_states 120 veces, sin inventarse ninguna'
```

Es la prueba de la **cadena de nodos sola**, sin hardware de por medio: sirve
para comprobar que el software está bien antes de enchufar nada, y para
demostrarlo con números.

La rampa no pasa de la mitad de la velocidad real de cada junta. Hasta el
28/09/2026 movía el husillo a unos 74 mm/s, y el propio auditor cantaba su sonda
como «31 saltos físicamente imposibles» (el husillo real va a 8 mm/s).

**Dos cerrojos, y los dos importan.** La sonda publica en los mismos topics que
la aduana, así que:

1. **Se niega si la aduana está publicando.** Con dos fuentes en el mismo topic,
   lo que salga por el otro lado no es de nadie.
2. **Se niega con la potencia dada.** Si `esp_rec` corre con `reenviar:=true`, un
   dato de prueba acabaría convertido en una consigna `T` para la placa.

Por eso la forma normal de usarla es `hw:=false`.

---

## 7. Parámetros

| parámetro | defecto | qué |
|---|---|---|
| `hz_informe` | `0.5` | cada cuánto publica la tabla |
| `t_perdido` | `0.5` | segundos sin salir antes de dar un mensaje por perdido |
| `t_gracia` | `0.1` | cuánto reposa un mensaje antes de juzgarlo |
| `tol` | `0.0` | tolerancia al comparar valores. **Cero a propósito** |
| `tol_fk` | `1e-9` | tolerancia de la cinemática, donde sí se rehace la cuenta |
| `max_vel_hw` | `[1.5, 1.5, 0.008]` | el techo real; con él se calcula el salto imposible |
| `margen_salto` | `3.0` | cuántas veces ese techo antes de cantar |
| `sonda_n` / `sonda_hz` | `40` / `25.0` | muestras de la sonda y a qué ritmo |
| `t_espera` | `3.0` | cuánto se espera la contestación de la placa al `?` |
| `registro` | `''` | CSV con una fila por salto y por informe |

**`t_gracia` no es prudencia, es necesidad.** El ejecutor de ROS 2 no reparte
los callbacks en el orden en que llegaron los mensajes, sino según le toca a
cada suscripción: el mensaje de salida de un salto se puede procesar **antes**
que el de entrada. Juzgando al momento, eso sale como «este nodo publicó algo
que nadie le mandó», que es acusar de una pérdida a quien no la tuvo. Con una
décima de margen, el orden deja de importar.

---

## 8. Diagnóstico

| lo que ves | qué significa |
|---|---|
| `perdidos` > 0 en un salto | ese nodo no está republicando. Mira si va sobrecargado o si su cola de 10 se desborda |
| `alterados` > 0 | alguien está tocando el dato por el camino. Es lo más grave de la tabla |
| `colados` | salió un valor que no entró. Si es al principio, es el arranque; si no, mira quién más publica en ese topic. En `motores -> cc`: `cc` juntó piezas de muestras distintas |
| `piezas de muestras incompletas` en `motores -> cc` | no es de `cc`: a esas muestras les faltó una pieza antes. Mira el salto de arriba del motor que falta |
| `saltos imposibles` | se perdieron muestras de telemetría (la aduana ya tira las líneas rotas y lo dice en su log: «enlace con la placa: N línea(s) tirada(s)…») |
| `el auditor va con retraso` | el propio auditor no da abasto: **sus cuentas de ese informe no valen**, porque lo que pierde él lo apunta como perdido de otros. Baja `telemetria_hz` |
| `la placa no contesto al "?"` | no hay enlace, o la aduana no está corriendo |
| `la aduana esta publicando ahora mismo` | la sonda se niega, y hace bien: lanza con `hw:=false` |
| la latencia de `cc -> punta` es enorme | esa fila no mide a `cc`: mide lo que tarda **el auditor** en ver las dos mitades del mismo ciclo |

---

## 9. Lo que el auditor NO puede ver

Conviene tenerlo claro, porque un auditor que promete de más es peor que no
tenerlo:

**El XOR de la telemetría.** No viaja por ningún topic, así que desde el grafo
no hay forma de comprobarlo. Desde el 28/09/2026 lo comprueba la aduana, que es
quien tiene la línea, y cuenta en su log las que tira; el auditor solo ve lo que
ya pasó ese filtro.

**Quién tiene la culpa de una pérdida.** El auditor ve que un mensaje entró y no
salió; no ve si lo tiró la cola del publicador, la del suscriptor o el sistema
operativo.


**Su propia carga.** Si el auditor se satura, pierde mensajes y se los apunta a
los demás. Por eso se vigila a sí mismo —si su temporizador llega tarde, lo dice
en el informe— y por eso sus colas son de 200 y todo se casa por diccionario,
a coste constante: la primera versión recorría una lista por índice y era
**ella** la que perdía los mensajes que luego achacaba a los motores.

---

### Lo probado

Todo lo de aquí está medido contra la cadena real de los seis nodos, con la
ESP32 emulada por un puerto pty que habla el protocolo del firmware:

- **80 s de vigilancia**: 0 perdidos, 0 alterados, 0 duplicados en los tres
  saltos; error de FK máximo 0.00000 mm; latencias de 1 a 8 ms de media.
- **ida y vuelta**: correcta en 14–17 ms, y detecta el fallo cuando se le cambia
  un límite a la placa por detrás.
- **sonda**: 40 muestras por motor, todas idénticas al otro extremo.
- **telemetría rota**: inyectando un salto de 1.7 rad cada 40 muestras, las caza
  todas y deja los demás saltos en `OK`.

**28/09/2026**, tras etiquetar las muestras, con una placa falsa que además
metía a propósito líneas con un byte cambiado, muestras saltadas, líneas
repetidas y ráfagas de USB:

- **30 s de vigilancia**: los tres saltos en `OK`, `motores -> cc` a 2 ms de
  media (41 ms de máximo), un `joint_states` por muestra.
- un comprobador externo, que conoce lo que mandó la placa: cada
  `joint_states` idéntico a la muestra de su etiqueta, ninguno de una línea
  rota, velocidades iguales a las de la placa, de la aduana a `joint_states` en
  4 ms de media.
- **sonda**: correcta, y ya sin saltos imposibles.
- **ida y vuelta**: correcta en 7–10 ms.

Lo que no se ha probado con hardware de verdad son los tiempos: con la placa
física, las latencias de la [sección 3](#3-el-informe) incluirán el USB.
