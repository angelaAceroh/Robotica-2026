# Control sin temblor (24/10)

Lo pediste el 24/10: «mejora el control cinemático para que no tiemble tanto,
pon bien el home que es P1 y mejora a Simon». Aquí está lo que cambió, cómo se
probó (en el PC y en el brazo) y lo que **no** arregla el código: Simon resbala.

| | antes | ahora |
|---|---|---|
| Alvin parado, tras pasos rápidos (brazo, 5 pruebas) | se quedó temblando en 2 de 5: 31 y 90 idas y vueltas | 0 en las 5 |
| Banco, Alvin parado (2 s) | 28 idas y vueltas, PWM de +700 a −700 | 0 |
| Banco, lo que le falta tras un paso | hasta 2.8° | menos de 0.6° |
| Botón **Home** del traycho por juntas | los tres motores a 0 (Simon contra su tope) | **P1**: (0, −273), arriba |
| Simon (§8) | PWM hasta 870; 180°/s en el traycho por juntas | **PWM hasta 550; 30°/s** (en el brazo va a unos 20) |
| Alvin en P3 y P5 (§9) | se quedaba a 2.8° (14 y 18 mm del punto) | a medio grado como mucho (1.4 y 3.9 mm) |

---

## 1. Por qué temblaba

En la grabación del 24/10 (`grabar_placa`), Alvin, con la consigna quieta, iba
de 22.3° a 24.9° y vuelta unas 7 veces por segundo, con el PWM de +700 a −700.
Y no paraba: más de 50 s así.

- Alvin **no arranca con menos de 700** (21/10), pero **girando roza mucho
  menos**. Con el suelo de ARRANQUE (700) se embala: a unos 60°/s.
- A 60°/s avanza 0.6° en una vuelta del lazo (10 ms). La banda (±0.3°) mide
  0.6°, así que **la cruza entera entre dos vueltas**: el lazo no llega a verlo
  dentro, ve que se ha pasado y empuja al revés con −700.
- Dentro de la banda lo dejaba **suelto**, sin freno, y seguía por inercia.

Y un fallo que salió en el banco: al cambiar de sentido, el motor aún se iba un
poco hacia el otro lado por la inercia, y la **rampa del suelo** (ARRANQUE →
TOPE en 0.2 s si no avanza) lo contaba como «se ha movido». El PWM se quedaba
en ARRANQUE y a los 0.25 s saltaba el vigilante de atasco. Puede ser lo que le
pasó a Alvin el 23/10, atascado en 26.8° y 32.1° yendo a P2.

---

## 2. Lo que cambia en `main_control.py`

Cinco cosas, todas en el lazo de cada motor (`Motor.paso`):

| qué | cómo |
|---|---|
| **Freno** | Activo y sin empujar, el motor va **en corto** (IN1 = IN2 = 0 con ENA = FRENO), que frena más cuanto más rápido gira. Es el freno «por abajo»: el de arriba (IN1 = IN2 = 1) bloqueaba a Teodoro el 10/9. **Soltado (`q`) sigue suelto**, para moverlo a mano |
| **Corte antes de llegar** | Si va hacia la consigna tan deprisa que llegaría en ANTICIPO segundos (`v_med · ANTICIPO ≥ lo que falta − BANDA`), deja de empujar y frena. Si se queda a menos de dos bandas, ahí se queda |
| **Espera, no vuelve** | Yendo (el PC pide velocidad), si se ha adelantado un poco a la consigna (menos de 4 bandas), la espera frenado. Antes volvía hacia atrás con −700 y luego otra vez hacia delante |
| **A pulsos, cerca** | Con la consigna quieta y a menos de 3°, empuja a pulsos de PULSO ms y entre uno y otro espera frenado (40 ms como poco, y hasta que para). Si un pulso se pasa, el siguiente dura la mitad. Si no lo mueve, el siguiente con más PWM (de ARRANQUE a TOPE en 3 pulsos), y a los 6 sin moverse salta el vigilante: «N pulsos (hasta PWM …) y no se ha movido» |
| **Arranque por sentido** | ARRANQUE hacia + y ARR- hacia −. Simon hacia − se movió con 250 (24/10); con 450 se pasaba |

Y la rampa cuenta el avance **con signo** (el fallo del §1).

Las ganancias, en la tabla `GANANCIAS`:

| | ARRANQUE | ARR- | TOPE | FRENO | ANTICIPO | PULSO |
|---|---|---|---|---|---|---|
| Alvin | 700 | 700 | 870 | 1023 | 0.10 s | 20 ms |
| Simon | 450 | **300** (SUPUESTO: entre 250 y 350) | **550** (§8) | 1023 | 0.10 s | 20 ms |
| Teodoro | 450 | 450 | 920 | **0** | **0** | **0** |

**Teodoro va exactamente como antes**: sin freno, sin corte, sin pulsos y el
mismo arranque hacia los dos lados. Tiene una falla mecánica y lo dejamos para
el final.

Todas se cambian en caliente con `g` (no se guardan al reiniciar), por ejemplo:

```
g alvin anticipo 0.08     corta un poco más tarde
g alvin freno 0           sin freno: suelto en la banda, como antes
g alvin pulso 0           sin pulsos
g simon arranque_menos 350
```

Por el topic, igual que siempre:
`ros2 topic pub --once /scara/placa/orden std_msgs/String "data: 'g alvin freno 0'"`.

**Para volver al de antes:** en la placa sigue el firmware anterior como
`main_antes.py`. En Thonny, ábrelo de la placa y «Guardar como → Dispositivo
MicroPython → main.py». La copia también está en el scratchpad de la sesión del
24/10 (`antes_freno/main_placa.py`).

---

## 3. Probarlo sin el brazo: `banco_lazo`

`scara_kinematics/banco_lazo.py` (nuevo) ejecuta la clase `Motor` de
`main_control.py` **de verdad**, sin tiempo real, contra un motor de mentira
como los del brazo:

- **Arranque alto** (Alvin 0.66 del PWM) y **roce girando bajo** (0.30).
- **Freno en corto o suelto**, según IN1/IN2/ENA, como el L298N.
- **10 ms de retraso** del puente al motor.

Los números que no se han medido están ajustados para que el lazo de antes
tiemble como el brazo: ±1.4° a 7 Hz.

```
cd ~/ros2_ws/src/scara_kinematics
python3 -m scara_kinematics.banco_lazo                         # el firmware de ahora
python3 -m scara_kinematics.banco_lazo --firmware otro.py      # otro, para comparar
python3 -m scara_kinematics.banco_lazo --g alvin anticipo 0.05 # como la 'g' de la placa
```

Prueba cada motor con su física y con 12 variantes: más y menos roce, inercia,
velocidad y arranque, y sin retraso o con el doble. Hace tres ensayos:

- **sujetar:** la consigna 1° por delante y quieta.
- **escalones:** pasos de 0.5° a 15° a 84°/s, como juntas_nodo.
- **despacio:** a 3°/s, como control_nodo.

Antes y ahora, Alvin con su física:

| ensayo | antes | ahora |
|---|---|---|
| sujetar: idas y vueltas / cambios de PWM | 28 / 29 | 0 / 0 |
| escalones: se pasa / le falta / idas y vueltas | 2.37° / 1.37° / 87 | 1.16° / 0.50° / 0 |
| despacio: error máximo / cambios de PWM | 1.77° / 46 | 0.32° / 0 |

Con las 12 variantes, ahora no tiembla en ninguna: 0 idas y vueltas sujetando y
tras los pasos, y menos de 0.6° de error. La única excepción es el doble de
retraso yendo despacio, con algún cambio suelto. Lo comprueban los tests de
`test/test_banco_lazo.py`, entre ellos uno que exige que con el lazo de antes
el banco tiemble (si no, que ahora no tiemble no diría nada).

---

## 4. En el brazo (24/10)

Con el firmware nuevo cargado en la placa (comprobado byte a byte), solo Alvin
y Simon activos y Teodoro suelto, se hizo el mismo recorrido con el lazo de
antes y con el de ahora:

- Alvin ±12° a 84°/s, ida y vuelta dos veces; Simon −6° y +4° a 90°/s.
- 2.5 s quieto después de cada paso.
- El lazo de antes se puso con `g`: freno 0, anticipo 0, pulso 0 y el mismo
  arranque hacia los dos lados.

| prueba | antes | ahora |
|---|---|---|
| 1 (Alvin entre 0 y 12°) | 31 idas y vueltas tras el 3.er paso, quedó a 1.1° | 0 |
| 2 | 90: tembló en tres ratos quieto seguidos | 0 |
| 3 | 0 | 0 |
| 4 y 5 (Alvin entre 23° y 31°, pasos de 10°) | 0 y 0 | 0 y 0 |

Con el de antes no tiembla siempre, pero cuando empieza ya no para (como en la
grabación). Con el de ahora no tembló ninguna vez. Lo que falta al final de
cada paso, como mucho 0.5–0.6°: es lo que se deja quieto (dos bandas).

---

## 5. Home = P1

La orden `home` de `juntas_nodo` (el botón **Home** de la ventana del traycho
por juntas) va ahora a **P1**: (0, −273) arriba, con Simon 0.8° dentro de su
tope. Antes llevaba los tres motores a 0, con Simon contra el tope. En el plano
de la ventana, la cruz «home» está también en P1 (antes, en el home de Gazebo,
(300, 0)). Test: `test_home_va_a_p1`.

---

## 6. Simon resbala (esto no lo arregla el código)

Medido el 24/10, solo con su encoder (scripts del scratchpad: `juego_simon.py`,
`tope_simon.py`, `umbral_simon.py`, `resbala_simon.py`):

- **Con el codo contra su tope de HOME**, el motor siguió girando con PWM 400–650
  sin atascarse: unos 17° en total. El codo estaba en el tope; el motor giraba
  respecto a él.
- **Lejos del tope**, hacia + necesita 375–450 para mover el codo. Hacia − le
  basta 250.
- **En una ida y vuelta de 10° rápida** (PWM hasta 870), el tope «se corrió»
  2.2°.

**La transmisión motor → codo patina con casi el mismo par que hace falta para
moverlo.** Por eso empezaba bien y luego dejaba de moverse: el motor llega a su
consigna (el encoder está en el motor) y el codo se queda atrás. No hay PWM que
lo mueva sin resbalar, ni forma de buscar el cero empujando contra el tope.

Hay que revisarlo en el brazo:

- el prisionero de la polea o engrane del motor de Simon;
- el del lado del codo;
- o la correa, si salta dientes.

Para saber cuál: una raya de plumón que cruce eje y polea en cada lado; mueve
Simon y mira cuál se descoloca.

Después se comprueba con `umbral_simon.py`. Con el codo en el tope, ni con PWM
alto debe avanzar el motor.

---

## 7. Archivos

| archivo | |
|---|---|
| `firmware/micropython/main_control.py` | el lazo (§2), Simon a 550 (§8), los pulsos con signo (§9), llegar más fino y la frontera de los pulsos (§10); copia de antes en la placa (`main_antes.py`) |
| `scara_kinematics/banco_lazo.py` | **nuevo**: el banco (§3) |
| `test/test_banco_lazo.py` | **nuevo**: 11 tests |
| `scara_kinematics/juntas_nodo.py` | `home` a P1 (§5) |
| `scara_kinematics/juntas_gui.py` | la cruz «home» en P1 |
| `test/test_juntas.py` | + `test_home_va_a_p1` |
| `launch/traycho_juntas.launch.py` | Simon lento: `vel_simon` (§8); `husillo` y `z_inicial` (§9); Alvin lento: `vel_alvin` (§11) |
| `test/test_alvin_lento.py` | **nuevo**: 4 tests (§11) |
| `test/test_simon_lento.py` | **nuevo**: 5 tests (§8) |
| `scara_kinematics/banco_elastico.py` | **nuevo**: el banco con la transmisión que cede (§9) |
| `test/test_alvin_rebote.py` | **nuevo**: 7 tests (§9, §10) |
| `test/test_llegar_fino.py` | **nuevo**: 3 tests (§10) |

`setup.py` no cambia: los bancos se usan con `python3 -m`, sin ejecutable de ROS.

---

## 8. Simon lento pero seguro

Lo pediste el 24/10, después de arreglar a Simon: «limita a Simon en código,
mejor que sea lento pero seguro». Seguía resbalando: con el codo contra su
tope, cada empujón de PWM 450 avanzaba el motor 0.3–0.8°, y con 550, 0.6–0.8°.

**Desde el 19/11, el TOPE de Simon es 700** («déjalo fijo en 700 en el código»): con
550 se quedaba parado en −33.2 (29/10, «dale más fuerza»), y con 700 puesto en
caliente pasó y el traycho salió entero. Lo de abajo (550) es lo del 24/10.

| dónde | antes | ahora |
|---|---|---|
| `main_control.py`, TOPE de Simon (el PWM más alto que se le deja) | 870 | **550** (SUPUESTO: lejos del tope se mueve con 375–450; 550 deja margen sin llegar a lo de antes) |
| `traycho_juntas.launch.py`, velocidad de Simon | 180°/s (sus 30 RPM del yaml) | **30°/s** (SUPUESTO), con `vel_simon:=…` para otra |
| `puente_placa` en ese launch, lo más que deja a Simon | 190°/s | **36°/s** (un 20 % más de `vel_simon`) |

`config/micropython.yaml` **no cambia**: sus `velocidades` son también las de
`vector` (main.py). El launch le pasa a `juntas_nodo` las del yaml con la de
Simon cambiada, y al empezar lo dice: `Simon lento: 30 grados/s (el puente le
deja hasta 36)`. Con `control_nodo` (control_placa.launch.py) no cambia nada
más que el TOPE: allí el puente ya no le dejaba pasar de 15°/s.

```
ros2 launch scara_kinematics traycho_juntas.launch.py                  # Simon a 30°/s
ros2 launch scara_kinematics traycho_juntas.launch.py vel_simon:=20    # aún más lento
```

`vel_simon` no sube a Simon por encima de lo del yaml (180) ni de lo que deja la
placa (190). El TOPE se cambia en caliente con `g simon tope 600` (no se guarda
al reiniciar).

**En el brazo (24/10, firmware cargado y comprobado byte a byte):** solo Simon,
15° hacia la izquierda a 30°/s y vuelta, con Alvin y Teodoro sueltos. Sin
vigilantes ni errores. El PWM no pasó de 450, y se separó de la consigna 1.4°
como mucho. Fue a unos **20°/s** de media, no a 30: con el PWM limitado no da
más, y la consigna le espera (no se adelanta más de 1.5°). `juntas_nodo` manda
hasta que **mide** que ha llegado, así que solo tarda más.

Al volver, la cuenta decía −0.22 y Sergio vio el codo **en HOME**: en esa ida y
vuelta lenta resbaló 0.2° como mucho (lo que se ve a ojo). En los ciclos rápidos
de antes (90°/s, PWM hasta 870) se quedaba atrás 1.3° por ciclo. Después:
`fijar simon 0`.

En el banco, con TOPE 550, Simon no tiembla en ninguna de las 12 variantes y se
queda a menos de 0.5°. Tests (`test/test_simon_lento.py`):

- en la placa, Simon con TOPE 550;
- el launch, Simon a 30°/s y los otros dos como en el yaml;
- `vel_simon` no lo sube por encima del yaml ni de la placa;
- el traycho entero con Simon a 30°/s, y también si con 550 solo da 20: cada
  ida acaba en su punto.

**Lo que no arregla:** que resbale. Con menos par y más despacio, resbala menos
y menos de golpe, pero el encoder está en el motor: si resbala, la placa no lo
ve. Para saber si se ha quedado atrás, hay que mirar el codo. Y el homing por
atasco contra el tope sigue sin servir.

---

## 9. Alvin se quedaba a 2.8° de P3 y de P5

Pasó el 24/10, probando el traycho por juntas solo con Alvin y Simon (Teodoro
suelto, `husillo:=virtual`). Llegaba bien a todos los puntos menos a P3 y P5,
yendo hacia − de la placa: allí Alvin se quedaba siempre unos 2.8° por detrás,
a 14 y 18 mm del punto. No saltaba ningún vigilante, y las correcciones de
`juntas_nodo` no lo movían.

**Qué se vio.** Repitiendo el tramo de P3 con `juntas_nodo` y grabando la placa
(50 veces por segundo):

1. Alvin llegó hasta −30.9 (el objetivo era −31).
2. Después, frenado y sin empujar, el motor **volvió solo** hasta −28.1 en 2 s.
3. Mientras, la posición dibujaba dientes de sierra cada ~120 ms: un pulso lo
   llevaba 0.45° hacia la consigna y en unos 80 ms volvía 0.5°.

Suelto en −22.9 y en −28.5 no volvía, así que no hay nada tirando del brazo.

**Por qué (SUPUESTO, el encoder está en el motor y no se ve el brazo).** La
transmisión de Alvin cede. Empujando, el motor se adelanta al brazo; el brazo
se para antes por su rozamiento, y en cuanto el motor deja de empujar, la
transmisión tensa lo devuelve hacia el brazo. Un pulso de 20 ms solo la tensa
un poco: el brazo ni se entera.

**Por qué el firmware no hacía nada.** Dos cosas, que se arreglan:

| | antes | ahora |
|---|---|---|
| ¿Ha servido el pulso? | si el motor se había movido 0.1° desde el pulso anterior, **hacia donde fuera**. Como volvía un poco más atrás cada vez, «se movía»: ni subía el PWM ni saltaba el vigilante | si lo ha **acercado** a la consigna 0.1° más que lo más cerca que había estado (con signo, medido parado) |
| Los pulsos que no sirven | más PWM (hasta TOPE), mismo largo | más PWM y **más largos**: 20, 40, 60… ms, hasta 6 veces (`PULSO_LARGO_MAX`) |
| El corte antes de llegar (§2) | también cortaba los pulsos: con el motor yendo deprisa (el brazo no), a los 10 ms | no corta los pulsos que están subiendo; esos acaban al entrar en la banda o al **cruzar la consigna** |

Si después de 6 pulsos sigue sin acercarse, salta el vigilante como antes
(«6 pulsos (hasta PWM 870 y 120 ms) y no se ha acercado…»). Antes, con un
motor que va y vuelve, no saltaba nunca. *(Desde §10, si los pulsos mueven el
motor y vuelve, ya no es el vigilante: deja de empujar y avisa.)*

**Pruebas sin el brazo.**

- `test/test_alvin_rebote.py` (**nuevo**, 4 tests) usa un modelo sacado de la
  grabación: el motor se adelanta empujando, vuelve sin empujar y el brazo
  solo arranca con un empujón fuerte y largo. Con el firmware que había en la
  placa, el brazo se queda en −28.1 (a 2.9°) sin vigilante, como en el brazo.
  Con el nuevo llega a −30.96 en 0.8 s. Y si el brazo no arranca ni así, salta
  el vigilante.
- `scara_kinematics/banco_elastico.py` (**nuevo**) es un motor y un brazo
  unidos por un muelle (`python3 -m scara_kinematics.banco_elastico`). Es más
  tosco: en un barrido de 216 casos (muelle, roce, inercia), el brazo acaba
  bien en 59 con el firmware nuevo y en 38 con el de antes, y los fallos bajan
  de 96 a 45.
- `banco_lazo` (§3), sin cambios: con las 12 variantes, sin temblor.

**En el brazo (24/10, firmware cargado y comprobado byte a byte, 38 349
bytes).** El mismo tramo de P3: le faltan 0.24°, antes 1.68–2.73. Y el traycho
entero, solo con Alvin y Simon:

| | firmware de antes | ahora |
|---|---|---|
| error en P3 / P5 | 14.3 / 17.9 mm | **1.4 / 3.9 mm** |
| error medio de las 32 idas / máximo | 4.2 / 17.9 mm | **2.0 / 3.9 mm** |
| lo que tarda | 2 min 30 s | **1 min 21 s** |
| vigilantes | ninguno | ninguno |

Al acabar las dos vueltas, Simon marcaba −1.12 (P1 es −0.8) y Sergio vio el
codo **en P1**: con TOPE 550 y 30°/s (§8), en todo el traycho no resbaló más de
lo que se ve a ojo.

Alvin acaba como mucho a medio grado (0.41–0.56°). Es lo que la placa da por
bueno (dos bandas, 0.6°), pero `juntas_nodo` quiere 0.4°, así que algunos
puntos salen «FUERA de la tolerancia» en el registro: son 2–4 mm, no los 14–18
de antes. En P2, P4 y P7 quedó 1–1.4 mm peor que en la primera vuelta, dentro
de esa misma holgura.

**Para volver al de antes:** el firmware anterior (con Simon a 550) está en el
scratchpad de la sesión (`antes_elastico/main_control.py`). El de antes del
24/10 sigue en la placa como `main_antes.py`.

### Probar sin Teodoro: `husillo:=virtual`

`traycho_juntas.launch.py` tiene dos argumentos nuevos, y sin ellos todo va
como antes:

```
ros2 launch scara_kinematics traycho_juntas.launch.py husillo:=virtual z_inicial:=220
```

- `husillo:=virtual`: el puente activa solo Alvin y Simon (`a alvin simon`), y
  la z se la inventa integrando lo que se pide (como el `husillo` de
  `control_placa.launch.py`).
- `z_inicial`: la z de la que parte. Con Teodoro arriba del todo es 219.85,
  como P1.

---

## 10. Llegar más fino, y P3 otra vez (24/10, tarde)

Lo pediste el 24/10: «continúa mejorándolo, sigue». Tras §9, Alvin acababa
casi todos los puntos a 0.41–0.56° (2–4 mm), y `juntas_nodo` (que quiere 0.4°)
los daba por «FUERA de la tolerancia».

### Llegar más fino

Con la consigna quieta, la placa ya no se conforma con dos bandas (0.6°) tras
un corte o un pulso que se pasa: lo acercan los pulsos hasta la banda (0.3°).

| | antes | ahora |
|---|---|---|
| entra en la banda deprisa | asentado | asentado solo si va despacio (`V_PARADO`); si no, lo acercan los pulsos. Teodoro, sin pulsos, como antes |
| tras el corte antes de llegar (§2) | asentado si queda a menos de 2 bandas | no: los pulsos |
| un pulso que se pasa | a menos de 2 bandas, asentado | eso solo tras pasarse `PASADAS_MAX` (2) veces |
| a menos de 2 bandas y el pulso no lo acerca | más PWM | se queda ahí (con la transmisión de Alvin, forzar aquí lo dejaba peor) |

En el banco (12 pasos × 11 variantes): Alvin pasa de 0.4° en 5 de 132 pasos
(antes 37) y Simon en 3 (antes 21), sin temblor. `test/test_llegar_fino.py`
(**nuevo**, 3 tests).

**En el brazo (12:36, traycho entero).** P4, P5 y P6 mejoraron mucho (0.8 /
0.2 / 0.55 mm; antes 2.7 / 3.9 / 0.7), pero en **P3 se quedó a 13.5 mm**:
Alvin a 2.7° durante 35 s, la placa dando pulsos de 700 sin parar.

### Por qué en P3: la frontera de los pulsos

En la grabación, el brazo atascado en −28 y la consigna en −31. El motor iba y
volvía alrededor de −28, así que el error iba y venía alrededor de los 3° de
`PULSOS_HASTA`. Cada vez que pasaba de 3, salía de los pulsos y volvía a
empezar por el más flojo: en 35 s solo una vez llegó a 870. Este fallo también
lo tenía el firmware de §9 (en el banco hace lo mismo).

| | ahora |
|---|---|
| `PULSOS_SALE` 5° | ya empezados los pulsos, no se dejan hasta pasar de 5° |
| esperar parado de verdad | entre pulso y pulso, `ESPERA_PULSO_MS` (40 ms) **seguidos** despacio. Antes decidía en el punto en que el motor se da la vuelta, antes de que la transmisión lo devolviera: contaba «se ha pasado» sin haberse pasado |
| `PASAR_FORZANDO` 0.6° | un pulso a TOPE, si los de antes no lo acercaron (y aún no se ha pasado), puede llevar el motor 0.6° más allá de la consigna: llegar justo a ella no tensa más la transmisión |
| no llega: `AVISO`, no vigilante | si tras 6 pulsos no lo acerca pero los pulsos **sí mueven** el motor (≥ `GIRA_MIN`, 0.3°), deja de empujar hasta la siguiente consigna y lo dice (`AVISO Alvin no llega: …`, en el registro del puente). Con el motor bloqueado, el vigilante como siempre |
| «se ha ido», a pulsos | con los pulsos empezados y la consigna quieta, el vigilante deja hasta 5° (`PULSOS_SALE`) en vez de 3: frenado y quieto lejos de la consigna no es un motor que se escapa |

En el banco, con el brazo atascado a 1–2.9° y transmisiones y brazos de varias
durezas (90 casos, SUPUESTO: no se sabe la del brazo):

| | firmware de §9 | ahora |
|---|---|---|
| lo que le falta al brazo, media | 1.09° | **0.72°** |
| a más de 0.65° | 58 | **29** |
| vigilantes | 33 | **0** (28 acaban con AVISO) |

El barrido de §9 (216 casos): bien en 73 (antes 61), vigilantes 0 (antes 46).
Los pasos normales, idénticos a antes. Tests: 3 más en
`test/test_alvin_rebote.py` (la frontera, el AVISO y el vigilante a pulsos), y
uno cambiado («si no hay manera»: antes vigilante, ahora AVISO). En total, 173.

### En el brazo: algo empuja a Alvin hacia atrás en P3

Dos vueltas más con el firmware nuevo (cargado y comprobado byte a byte, 43 486
bytes). Las dos se cortaron en el mismo tramo, **P3 encima (4/4)**, con el
vigilante «Alvin se ha ido … de su consigna»:

| | hasta P3 (3/4) | en P3 (4/4) |
|---|---|---|
| 12:55 | 0.2–1.8 mm, ningún «FUERA» | el motor volvió solo de −28.6 a −25.9 en 0.4 s; quieto en −26.97, a 3.9° de la consigna: vigilante (el de 3°; de ahí el arreglo de la última fila) |
| 13:07 | 0.9–3.3 mm | yendo, el motor se frena hacia −26, y al parar **retrocede** a −24.9 en 0.15 s y luego, frenado, hasta −22.9: a 5° de la consigna, vigilante |

**Dónde se queda Alvin en P3, cada vez antes:** −28 (12:37), −27 (12:55),
−23 (13:07). A las 12:20 llegó bien (1.4 mm).

**Medido después, despacio (`alvin_carga.py` del scratchpad: solo Alvin, a
5°/s de grado en grado; en cada uno sujeto 1.5 s y suelto 1.5 s; Simon suelto
en −11.00, Teodoro en −0.66):**

| Alvin | sujeto | suelto 1.5 s |
|---|---|---|
| −23 → −28 | llega, sin PWM para sujetarlo | no se mueve |
| −29 | se queda en −27.2 | vuelve +0.33 |
| −30 | se queda en −29.2 (empujando con 756) | vuelve +0.39 |
| −31 | lo devuelven hasta −26.4 (empujando con 813) | vuelve +0.52 |

Desde unos −28, **algo empuja a Alvin hacia atrás** como un muelle: sujeto, lo
devuelve 2–4°, y suelto sigue volviendo. Hasta −28, nada. Teodoro no se ha
movido (su cuenta sigue en −0.66). Repetido un rato después, ya desde −26 (y
suelto volvía hasta 1.6°).

**Qué era (Sergio, 24/10): «si vas muy rápido la circunferencia de soporte se
cae».** El firmware no lo arregla, y el vigilante que suelta los motores es lo
que debe hacer ahí. Lo que en §9 parecía «la transmisión cede» puede ser, al
menos en parte, esto mismo. Ver §11.

**Para volver atrás** (scratchpad de la sesión): `antes_frontera/` (llegar más
fino, 39 880 bytes), `antes_banda/` (el de §9, 38 349 bytes). En la placa sigue
`main_antes.py` (el de antes del 24/10).

---

## 11. Alvin lento: la circunferencia de soporte

Lo dijiste el 24/10, después de §10: «lo que está pasando es que si vas muy
rápido la circunferencia de soporte se cae». En el traycho por juntas Alvin iba
a sus 14 RPM (84°/s), y caída, la circunferencia lo empujaba hacia atrás pasado
−26/−28 (§10).

| dónde | antes | ahora |
|---|---|---|
| `traycho_juntas.launch.py`, velocidad de Alvin | 84°/s (sus 14 RPM del yaml) | **30°/s** (SUPUESTO, como Simon), con `vel_alvin:=…` para otra |
| `puente_placa` en ese launch, lo más que deja a Alvin | 90°/s | **36°/s** (un 20 % más) |

Igual que `vel_simon` (§8): el yaml no cambia, `vel_alvin` no lo sube por
encima del yaml (84) ni de la placa (90), y al empezar lo dice: `Alvin lento: 30
grados/s (el puente le deja hasta 36)`.

```
ros2 launch scara_kinematics traycho_juntas.launch.py                   # Alvin y Simon a 30°/s
ros2 launch scara_kinematics traycho_juntas.launch.py vel_alvin:=20     # aún más lento
```

Alvin necesita PWM 700 para arrancar, así que despacio podría ir a tirones (para
y arranca), que es justo lo que no le conviene al soporte. En el banco, con las
11 variantes, 15° a 30°/s: va seguido y llega a menos de 0.6°.
`test/test_alvin_lento.py` (**nuevo**, 4 tests): el launch, que `vel_alvin` no
sube de lo que hay, el traycho entero con Alvin a 30°/s (simulado) y eso del
banco.

**En el brazo, en la zona libre (−23 a −6, solo Alvin, pasos de 0.5–10° a
84°/s, antes de saber lo del soporte):** le faltan 0.19° de media, sin temblor,
y 2–3 de 20 pasos a 0.47–0.59°. Con pulsos de 10 ms, peor (0.29°, 8 de 20); con
30 ms, igual (0.23°). El PULSO se queda en 20 ms.

**En el brazo (24/10, 14:54, circunferencia puesta, Alvin y Simon a 30°/s):**
el traycho entero llegó a todos los puntos, **P3 incluido (1.6 mm)**:

| | 12:20 (84°/s, firmware de §9) | 14:54 (30°/s, firmware de §10) |
|---|---|---|
| error medio de las idas / máximo | 2.0 / 3.9 mm | **1.6 / 3.9 mm** (31 idas) |
| P3 / P5 | 1.4 / 3.9 mm | **1.6 / 3.9 mm** |
| «FUERA de la tolerancia» | muchos (Alvin 0.41–0.56°) | **3** (P5, Alvin 0.50°) |
| AVISO o vigilantes de Alvin | ninguno | ninguno |

Pero en la última ida (P8 → P1) la placa soltó los motores: **Simon** se paró
en seco en −3.59 yendo hacia su tope de HOME, a 30°/s y con PWM 482 («0.25 s
empujando de 450 a 550 y no se ha movido»). Al empezar, en P1, había llegado a
−1.68 sin tocar nada: si lo que lo paró es su tope, en esta vuelta resbaló unos
2–3°, aun despacio y con PWM ≤ 482. Sergio miró: «está en home» → `fijar
simon 0` (la cuenta decía −3.59). Así que **Simon resbaló 3.6° en una vuelta**
a 30°/s: lento no basta para que no resbale.

**Otra vuelta (15:10, Simon fijado en HOME):** el encoder de Simon contó de 0
a −41.4 y, volviendo, se paró en seco en −12.01 empujando hacia HOME (PWM 498;
saltó el vigilante). Sergio: «Simon no gira nada». Escribí aquí que la
transmisión no transmitía (el motor gira y el codo no). **Era falso**; Sergio:
«no te creo, cuando le daba a la recta sí iba, revisa bien a Simon».

Revisado (15:17–15:30, solo con la placa):

- **El código:** los pines (18, 19, 5, 12, 13), el encoder en cuadratura y la
  cuenta `5500·21/18` son los del `Simon.py` de Sergio. El camino de las
  órdenes también es el mismo que el de «Ir en recta» en esta ventana (las dos
  mandan `ir X Y Z` a juntas_nodo). Lo que cambia es la fuerza: su programa
  sube el PWM hasta 1023, y este firmware lo deja en TOPE 550 desde el 24/10
  por la mañana (§8).
- **El codo sí sigue al motor:** volviendo despacio hacia HOME, Simon choca
  siempre con un tope duro en la misma cuenta (−12.01, donde se paró a las
  15:11). Si el motor girara en vacío no habría tope. Así que en la vuelta de
  las 15:10 el codo se movió, pero motor y codo se desfasaron unos 12°.
- **Simon solo apenas se desfasa:** 6 pasos de 5° a 30°/s con FRENO 1023, y
  vuelta, dejan el tope en +0.39 (`simon_tope.py`).
- **Moviendo a Alvin, sin aclarar:** con Simon quieto (su encoder sin moverse
  ni 0.01°), 6 idas y vueltas de Alvin de 15° dejaron el tope en +2.36, y con
  rampas de 0.4 s en +4.94. Pero cada vez que se empuja contra el tope también
  se corre algo (lo que crece de una prueba a otra apunta a eso), así que no
  separa lo que hace Alvin de lo que hace el tope.
- **Error mío al medir:** usar el tope de HOME como referencia empujando
  contra él. El tope se fue corriendo hasta la cuenta 0 (el MAX de la placa).
  Ahí el vigilante no saltó, porque la cuenta seguía moviéndose, y el script
  siguió empujando: Sergio, «se está chocando, a la derecha». Cortó los 12 V.
  El codo quedó en HOME → `fijar simon 0` (marcaba −0.22). **No volver a
  buscar el tope de Simon a motor.**

Lo que queda: entre el motor (donde está el encoder) y el codo algo se corre
cuando hay carga, unos grados por vuelta del traycho (3.6° a las 14:54, 12° a
las 15:10). No es que no transmita.

**16:33, con Sergio mirando** (Simon solo, tres idas de −2 a −20 a 10°/s, dos
veces): «choca con la base, o sea con el robot, creo que dejamos el home muy
pegado» y «giró el codo». **El «tope» de HOME era el antebrazo contra la
base.** Al volver a P1, que estaba 0.8° más afuera, en cuanto la cuenta se
descuadraba un poco, la placa empujaba el antebrazo contra la base, la
transmisión cedía y la cuenta se descuadraba más. Ahora el HOME está más lejos
(Simon −17.5, P1 en (0, −331)) y Simon no pasa de −5 hacia la base:
TRAYCHO_JUNTAS.md.
