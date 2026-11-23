# «traycho»: la trayectoria de tu código, con un botón

Añadido del **21 de octubre de 2026** (noche). Es la ventana del brazo de verdad
([CONTROL_PLACA.md](CONTROL_PLACA.md) §3) con un botón más, **traycho**. Hace la
trayectoria multipunto de la fase 1 del ejemplo de control cinemático que
pasaste. El resto de la ventana sigue igual: «Ir a un punto», salto, círculo,
soltar y activar los motores.

| archivo | qué es |
|---|---|
| `scara_kinematics/traycho.py` | los puntos de tu código, cómo se colocan en el brazo y el que manda los `ir` uno detrás de otro (sin ROS) |
| `scara_kinematics/traycho_gui.py` | la ventana: la de `control_placa_gui` heredada, más el botón y el recorrido en la planta |
| `launch/traycho.launch.py` | `control_placa.launch.py` entero, con esta ventana en vez de la suya |
| `test/test_traycho.py` | 10 tests |
| `setup.py` | +1 línea en `entry_points`: `traycho_gui` |

El mismo botón, pero **por juntas** (cada junta a su velocidad fija y corrigiendo
al llegar): [TRAYCHO_JUNTAS.md](TRAYCHO_JUNTAS.md).

No toca `control_placa.launch.py`, `control_placa_gui`, `control_nodo` ni
`puente_placa`.

---

## 1. Cómo se usa

Desde HOME:

1. **Los motores sueltos y el brazo en HOME.** Si la placa no sabe dónde está
   (lo moviste a mano con ella reiniciándose), díselo en Thonny:
   `fijar alvin 0` y `fijar simon 0`.
2. **Simon unos grados hacia dentro.** En HOME Simon está en 0, su tope, y
   `control_nodo` rechazaría el primer `ir`. En Thonny: `a simon`, dos o tres
   `V 0 -3 0` (−1° cada una) y `q`. Luego **cierra Thonny**.
3. Lanza:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch scara_kinematics traycho.launch.py k:=2 vel_max:=0.1
```

4. Pulsa **traycho**. En la planta, en morado, están los ocho puntos y las
   rectas entre ellos. Debajo del botón pone a qué punto va y por qué recta va.

Admite los mismos argumentos que `control_placa.launch.py`: `simular:=true` (sin
el brazo), `vel_z`, `activar`, `margen`...

**Lo corta** cualquier otra orden: **Parar (Esc)**, «Ir en recta», «Soltar
motores»... `control_nodo` hace una sola cosa a la vez. También lo corta un
vigilante de la placa. Si se corta, el brazo se queda regulando donde esté (o
suelto, si fue la placa), y en los mensajes sale por qué. Para repetirlo, vuelve
a pulsar el botón: sale desde donde esté la punta.

---

## 2. Los puntos

Tu código es para otro SCARA (`a1 = 16.5`, `a2 = 13.5` cm), que llega a 30 cm
de la base. Este llega de 27 cm a 50 cm: en HOME, (0, −27) desde el 23/10, no
se acerca más a la base. Además, con los recorridos de los motores solo llega
por delante. Por eso tus puntos, tal cual, casi no caben. Se han **movido todos
juntos** +8 cm en X y −29 cm en Y. En planta no se han girado ni escalado: la
figura y las distancias entre los puntos son las de tu código. Con el HOME en
(0, −27) quedan a 21° de junta de los topes en el punto más justo (con el de
antes, 25°). Es un **SUPUESTO**: si los quieres en otro sitio, cambia
`DESPLAZAMIENTO_CM` en `traycho.py`. El test comprueba que sigan cabiendo.

**Las z, las de tu código** (`ESCALA_Z` 1) (desde el 23/10: «bajaste mucho a
Teodoro», y luego el soporte cilíndrico: a media escala chocó con él, y a un
cuarto, bajando a P4, casi). P1, la más alta, va arriba del todo (22.0 cm,
Teodoro a 0.5 mm de su tope), y las demás bajan lo que bajan en tu código:
Teodoro baja 115 mm como mucho.
`ESCALA_Z` (**SUPUESTO**) y `Z_ARRIBA_MM` en `traycho.py`.
Hasta el 29/10 iban a 0.04 (4.6 mm): «calibra a Teodoro porque casi no está
bajando». El «casi» de P4 a −10 mm no vale: ese día la placa había perdido el
HOME y Teodoro se fijó a 0 donde estaba, no arriba. Con el cero arriba, chocó a
−47. Luego, a 0.10 (11.5 mm); y el mismo 29/10, «se supone que debe bajar casi
todo y bajó menos de un cuarto»: a 1, con el soporte aún puesto (los puntos están
9 cm más afuera que el 23/10). Teodoro no baja de −120 (`TEODORO_MIN_MM`, en el
traycho por juntas).
En el traycho por juntas, además, P3, P5 y P7 no bajan de −50 (29/10, «es el
soporte, limita P3 P5 P7 y sigue»), y P2, P4 y P6 no bajan de −52.1 (30/10,
bajando a P2 lo paró en −52.2: «deja a Teodoro que baje hasta ahí, porque se
iba a chocar»; P4 y P6, **SUPUESTO**, sin probar). `PUNTOS_SOPORTE`,
`PUNTOS_HONDOS` y sus z, en `juntas.py`.

| | tu código (cm) | en el brazo (cm) | Teodoro |
|---|---|---|---|
| P1 | 5.5, −6.5, 21.5 | 13.5, −35.5, 22.0 | −0.5 mm |
| P2 | −10, −15, 10 | −2, −44, 10.5 | −115.5 mm |
| P3 | 19, 6, 15 | 27, −23, 15.5 | −65.5 mm |
| P4 | 0, −15, 10 | 8, −44, 10.5 | −115.5 mm |
| P5 | 23.5, 6, 15 | 31.5, −23, 15.5 | −65.5 mm |
| P6 | 10, −15, 10 | 18, −44, 10.5 | −115.5 mm |
| P7 | 28.5, 6, 15 | 36.5, −23, 15.5 | −65.5 mm |
| P8 | = P1 | = P1 | |

**Lo que cambia respecto a tu código:**

- **De punto a punto va en recta** (el `ir` de `control_nodo`). Tu código
  interpola las juntas, así que allí la punta hace curvas entre los puntos.
- **Ninguna junta gira más de 20° en un `ir`** («que no haga pasos rápidos a más
  de 20 grados»). Las rectas largas se parten en trozos iguales, siempre sobre
  la misma recta: desde HOME salen **27 rectas** (28 con el HOME y las z de
  antes). Por ejemplo, de P2 a P3 Alvin gira unos 70°, y esa recta va en 5 trozos.
- **Las velocidades son las del launch**, no las de tu `q_dot_max`: 0.1 rad/s en
  hombro y codo, y **1 mm/s en Teodoro**, que no pasa de unos 2. Con las z de
  tu código la z subía y bajaba 50 cm en total y la vuelta tardaba 12 minutos
  (en el brazo, 12 min 5 s); a un cuarto, 12 cm; ahora, a 0.04, 2 cm. Mientras, el
  plano va a la par de la z.
- La z: el 0 de Teodoro es su tope de arriba de verdad (CONTROL_PLACA.md §9,
  punto 10). Que ese tope sea z 22.05 cm es **supuesto**.

---

## 3. Probado

**En la placa simulada** (`main_control.py` de verdad con motores simulados,
`vel_z:=8` para no esperar 9 minutos), dos vueltas seguidas enteras: cada recta,
con 1 mm de error medio y de 2 a 5 de máximo. Una vez hubo un pico de 24 mm. Tarda
2.5 minutos. Ahí salió que Teodoro no volvía a subir después de bajar
(CONTROL_PLACA.md §9, punto 8); está arreglado.

**En el brazo, el 21/10 por la noche: la vuelta entera, sin un solo corte**,
a la cuarta. Fueron 26 rectas en 12 minutos, saliendo de al lado de P1, donde
se había cortado la tercera. Acabó a 3.8 mm de P1 en el plano y 0.0 en z.

| | |
|---|---|
| error medio de la punta, en cada recta | de 0.7 a 4.7 mm (media 3.2) |
| máximo, en cada recta | de 3 a 17 mm, salvo una: 53 mm |
| z (Teodoro) | 0.04 mm de error medio, 0.6 de máximo |
| Teodoro, de su consigna en la placa | nunca más de 0.17 mm. Empuja el 15 % del tiempo; el resto se sostiene solo |

Los 11–17 mm de máximo son los tirones de Alvin (CONTROL_PLACA.md §7). Los
53 mm fueron en P4 (1/4): Alvin se quedó 10 s quieto al cambiar de sentido y
luego recuperó. Eso ya está arreglado y subido a la placa (CONTROL_PLACA.md §9,
punto 13), pero **todavía no se ha vuelto a probar en el brazo**.

Antes, tres intentos cortados por la placa, con el mismo motivo de fondo. Los
detalles están en CONTROL_PLACA.md §9, puntos 9 a 12:

1. Yendo a P1, Teodoro se pasó 0.46 mm hacia arriba. El suelo de ARRANQUE
   empujaba aunque ya fuese por delante de la consigna.
2. Entre el primer intento y el segundo, se vio que el 0 de Teodoro estaba
   corrido 38 mm, y se volvió a fijar arriba del todo. En el segundo, bajando
   a P2, se embaló hacia abajo (0.62 mm).
3. Subiendo a P1, se embaló hacia arriba (0.45 mm). Los dos embalamientos eran
   su integral, cargado con los tirones de la cremallera. **Teodoro va ahora
   sin integral.**

La vuelta entera está en `~/traycho_brazo.csv` (el registro de
`control_nodo`) y `~/traycho_brazo_placa.csv` (el seguimiento y el PWM de cada
motor en la placa, 5 veces por segundo).

```bash
cd ~/ros2_ws/src/scara_kinematics && python3 -m pytest test/test_traycho.py
```
