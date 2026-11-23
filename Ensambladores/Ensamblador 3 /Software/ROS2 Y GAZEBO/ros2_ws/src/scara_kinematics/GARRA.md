# La garra (19/11)

La manita del SCARA es un servo en el **GPIO 33** de la ESP32. El código probado
abría y cerraba solo cada 2 s; ahora se mueve cuando se le manda:

| orden | qué hace | ángulo |
|-------|----------|--------|
| `1`   | abre     | 10° (`ABIERTA`: como se montó en HOME) |
| `0`   | cierra   | 50° (`CERRADA`: «chocolate») |

Al arrancar la placa, la garra se pone **abierta**.

El servo va al **3V3 de la ESP32**, eso no se puede cambiar, y le falta fuerza. Por eso
`garra.py` (§7):

- va **de golpe**, como el código probado: es como más empuja;
- a **50 pulsos/s**, como el código probado (0.6 ms abierta, 1.05 ms cerrada);
- **empuja 1 s y suelta** (`EMPUJON_MS`): después, sin señal, el servo no tira corriente.
  La mano se queda donde la deja, y cada orden vuelve a empujar aunque ya esté ahí;
- **mide su 3V3** mientras empuja (GPIO 32, libre, con pull-up y pull-down al ADC) y
  dice si al servo le llegó corriente: debajo de Abrir/Cerrar, «servo con corriente» o,
  en rojo, «servo SIN corriente: mira el contacto del rojo y el marrón»;
- **apaga el detector de caídas de tensión** de la ESP32 durante los 2 s siguientes a
  cada orden (`AGUANTAR_MS`), que es cuando el servo tira corriente. El resto del
  tiempo, y en particular cuando el brazo se mueve, el detector está encendido.

Y en el traycho por juntas, en cada punto:

| dónde | garra |
|-------|-------|
| al salir (HOME) | `1` abre |
| P1 | `1` abre |
| P2 | `0` cierra (agarra el chocolate) |
| P3 | `1` abre |
| P4 | `0` cierra |
| P5 | `1` abre |
| P6 | `0` cierra |
| P7, P8 | nada: sigue cerrada y vuelve a P1 (que abre) |

(20/11: P1..P6 al revés que el 19/11, «en p2 debe cerrar y está abriendo».)

La garra se mueve **al llegar** al punto: en los que están abajo (P2..P7), después
de bajar y antes de volver a subir. Cuando la placa contesta, espera `ESPERA_S`
(2 s) a que llegue el servo y sigue. Mientras tanto el brazo se queda quieto en el
punto.

**Abajo, hasta −25 mm** (`GT.TEODORO_SUELO_MM`): con la garra montada, la mano llega al
suelo con Teodoro a −27.8, y si baja más **aplasta los cables del servo** y la placa se
reinicia sin parar (19/11). `traycho_garra.launch.py` pone `teodoro_min` a −25 y la
ventana sube los puntos de abajo (P2..P7) hasta ahí; se cambia con `teodoro_min:=...`.

## 1. Archivos

Todo es nuevo, menos `main_control.py` y una línea en `setup.py`:

| archivo | qué es |
|---------|--------|
| `firmware/micropython/garra.py` | el código de la garra: `1` abre, `0` cierra, arranca abierta. Va en la placa al lado del `main.py`; solo, sirve para probar la garra |
| `firmware/micropython/main_control.py` | **ampliado**: si `garra.py` está en la placa, lo importa y le pasa el `1` y el `0`. Sin `garra.py` hace lo mismo que antes y al `1` contesta `[x] no hay garra.py` |
| `scara_kinematics/garra_traycho.py` | la secuencia: `ACCIONES` (qué hace en cada punto) y `RecorridoGarra` (el Recorrido del traycho con la garra en medio) |
| `scara_kinematics/puente_garra.py` | nodo: `puente_placa` heredado, que también deja pasar `1`/`abrir` y `0`/`cerrar` a la placa |
| `scara_kinematics/garra_gui.py` | la ventana del traycho por juntas heredada, con la garra debajo del botón traycho |
| `launch/traycho_garra.launch.py` | `traycho_juntas.launch.py` tal cual, con `puente_garra` y `garra_gui` |
| `test/test_garra.py` | 20 pruebas |

`puente_placa.py`, `juntas_gui.py`, `traycho_juntas.launch.py` y los demás no se
han tocado.

## 2. Cargarla en la placa

**Ya está cargada (19/11, 21:45):** la placa lleva `garra.py` (9555 bytes) y el
`main.py` nuevo (44653 bytes), comprobados byte a byte. En la placa
también quedan `main_probado.py`, el código probado tal cual (792 bytes, abre y cierra
solo cada 2 s), y `main_sin_garra.py`, el de antes (43486 bytes, el del 24/10). Para volver a él, en Thonny:
borrar `main.py` y renombrar `main_sin_garra.py` a `main.py`. Copias en
`~/esp32_respaldo/2026-11-19_garra/`.

Para volver a cargarla, en Thonny, con la placa conectada:

1. `firmware/micropython/garra.py` → «Guardar como → Dispositivo MicroPython» →
   **`garra.py`**
2. `firmware/micropython/main_control.py` → «Guardar como → Dispositivo
   MicroPython» → **`main.py`** (el de siempre, ahora con la garra)

Al reiniciar dice `GARRA abierta 10` y, en la línea de órdenes, `1/0 =
abrir/cerrar la garra`. Para quitarla basta con borrar `garra.py` de la placa.

**Para probar la garra sola**, sin el brazo: abrir `garra.py` en Thonny, darle a
ejecutar y teclear `1` o `0`.

## 3. Usarla

    ros2 launch scara_kinematics traycho_garra.launch.py                  # el brazo
    ros2 launch scara_kinematics traycho_garra.launch.py activar:=false   # motores sueltos
    ros2 launch scara_kinematics traycho_garra.launch.py simular:=true    # sin el brazo

Los argumentos son los de `traycho_juntas.launch.py` (`vel_simon`, `vel_alvin`,
`teodoro_min`, `husillo`, …). En la ventana:

- **traycho**: el de siempre, con la garra (la tabla de arriba). Mientras mueve la
  garra, debajo del botón pone «garra en P3: cierra».
- **Abrir (1)** y **Cerrar (0)**, debajo del botón traycho: a mano. Al lado, lo último que ha dicho la placa
  (`abierta 10°` / `cerrada 50°`). Como cualquier otra orden, cortan el traycho si
  va.

Con `simular:=true` la placa simulada carga también `garra.py` (con un servo de
mentira) y contesta como la de verdad.

Por ROS, sin la ventana: `ros2 topic pub --once scara/placa/orden std_msgs/String
"data: '0'"`.

## 4. El protocolo

    PC -> placa   1          placa -> PC   GARRA abierta 10
    PC -> placa   0          placa -> PC   GARRA cerrada 50
                             1 s después   GARRA corriente si 403   (mV que bajó el ADC)
                                     o     GARRA corriente NO 5 (no le llega: ...)

Con rampa (`VELOCIDAD` en grados/s), antes dice «GARRA abriendo 10» o «GARRA cerrando
50», y la de arriba llega al terminar.

`puente_garra` saca la respuesta por /rosout como `placa: GARRA ...`, y de ahí la
leen la ventana y `RecorridoGarra`. El traycho se corta si la placa no contesta en
`CONTESTA_S` (5 s), si dice `[x]` (por ejemplo `[x] no entiendo: 1`: lleva el
`main.py` sin la garra) o si suelta los motores.

## 5. Para cambiar algo

- **Dónde abre y cierra**: `ACCIONES` en `garra_traycho.py` (`{punto: '0' o '1'}`).
- **Los ángulos**: `ABIERTA` y `CERRADA` en `garra.py` (y volver a cargarlo en la placa).
- **Lo que espera al servo**: `ESPERA_S` en `garra_traycho.py`.
- **De golpe o en rampa**: `VELOCIDAD` en `garra.py`. `None` es de golpe; un número son
  grados/s (con 20 la placa no se reiniciaba nunca, pero el servo no se movía).
- **Cuánto empuja antes de soltar**: `EMPUJON_MS` en `garra.py` (1000 ms).
- **Pulsos por segundo**: `FRECUENCIA` en `garra.py` (50). `mover_servo()` mantiene
  los anchos de pulso del código probado a cualquier frecuencia.
- **El detector de tensión**: `AGUANTAR_MS` en `garra.py` (2000 ms apagado tras cada
  orden).

## 6. Supuestos

- **El servo va en la ESP32 del brazo** (la del USB), en el GPIO 33, que ahí está
  libre. Si va en otra placa, esa lleva `garra.py` como `main.py` y el traycho no
  puede mandarle nada (no hay un segundo puerto).
- **«P1» es el P1 de la ventana, que es el HOME**: el traycho sale con la garra
  abierta y lo primero que hace, ya en P1, es cerrarla. Si lo que querías es que
  agarrara en el primer punto después del HOME, se cambia `ACCIONES` por
  `{2: '0', 3: '1', 4: '0', 5: '1', 6: '0', 7: '1'}`.
- **Cerrada = 50°**: en el código probado el texto dice «chocolate (30°)», pero
  manda 50.
- **2 s** para que el servo llegue (`ESPERA_S`), como en el código probado.

## 7. Lo que se vio en la placa (19/11)

- Lleva **MicroPython v1.29.0**: admite PWM a frecuencias distintas a la vez (el
  servo a 50 Hz y los motores a 1 kHz, en timers distintos).
- **Crear un PWM a 1 kHz con `duty=` la cuelga** (Guru Meditation y se
  reinicia), con los de los motores ya creados. Por eso `garra.py` crea el suyo
  como el código probado: a 50 Hz y sin `duty=`. Así nace con duty 0 (sin pulso) y
  el servo no da un tirón al encender.
- Al abrir el puerto sin reiniciar la placa, la basura que entra hace que
  `leer_consola()` de `main_control.py` se caiga con `UnicodeError`, y la placa se
  queda en el REPL (los PWM, como estuvieran). Con `puente_placa`, que reinicia la
  placa al abrir, no pasa. No se ha cambiado.
- **El servo al 3V3 de la ESP32** (rojo a 3V3, marrón a GND y señal en D33; no se
  puede pasar a VIN). Con el código probado tal cual, mandado de golpe, **la placa se
  reinició 35 y 31 veces seguidas** (`rst:0x3 (SW_RESET)`), siempre justo después de
  «chcolate», es decir, al mandar cerrar. Cuando no se reiniciaba, la garra unas veces
  abría y cerraba, otras solo abría y otras nada. Con la señal medida en el pin todo
  era igual: 50 Hz, 0.6 y 1.05 ms. **En rampa** (1° cada 50 ms): 0 reinicios en 16
  ciclos. Ya con `main.py` y la rampa: 0 reinicios en 54 s, 21 s de ellos cerrada, y
  el lazo de los motores sin parar.
- Lo que la placa no puede arreglar: si con 3.3 V el servo no tiene fuerza para
  cerrar la mano, la rampa no se la da. Lo seguro sigue siendo darle 5 V (VIN), con
  GND común.
- **De golpe con el detector apagado** (19/11, 20:32, motores sueltos): a 50, 100 y 200
  pulsos/s, 0 reinicios. Sergio: «lo escuché, pero se movió muy poco, está sin fuerza»,
  y más a 200. A las 20:38, ni a 200 cerrando a 90° ni a 300 se movió nada. El detector
  de la placa ya estaba en su umbral más bajo (2.43 V, registro `0x43ffc000`), así que
  la única opción era apagarlo.
- Desde la placa no se puede dar más fuerza al servo: la señal no lleva potencia, y
  toda le llega por el rojo, a 3.3 V. Con esta configuración la garra se moverá poco y
  no siempre. Lo que sí está asegurado es que la placa contesta y que el traycho
  sigue.
- **La causa era un falso contacto** (19/11, 21:31) en el rojo o el marrón del servo. La
  placa mide su 3V3 en el GPIO 32 (con pull-up y pull-down, el ADC lee ~1400 mV, el 0.43
  del 3V3; encender el WiFi lo baja ~75). Mandando cerrar/abrir sin tocar los cables, el
  3V3 no bajaba nada: al servo no le llegaba corriente. Con Sergio moviendo los dupont,
  bajó 22 veces, hasta 437 mV (~1 V en el 3V3), y la garra se abrió. Así que las pruebas
  de antes (rampa, 100/200 pulsos/s, «sin fuerza») no valen: el contacto iba y venía solo.
- **Sujetando, la placa se reiniciaba** (21:34, 15 s después de cerrar, con el detector ya
  puesto): el servo bloqueado tiraba corriente sin parar. Por eso ahora empuja 1 s y suelta.
- **Con el contacto apretado** (21:46, 50 pulsos/s, empujón de 1 s): cerrar, el 3V3 bajó
  403 mV; abrir, 416. Sin reinicios.
- **Los cables aplastados bajo la mano** (19/11, 21:52–22:05): con la mano en el suelo
  (Teodoro −27.88), la placa se reiniciaba en bucle antes de arrancar MicroPython
  (`E BOD`); con la mano levantada, o el rojo del servo suelto, arrancaba. Teodoro se subió
  15 mm con su motor (`a teodoro` + `V`) y Simon se separó 10° de la base, uno a uno.
- **Traycho con la garra y el suelo de −25** (22:15): subió, P1, P2 a −25 sin tocar el
  suelo; la garra sin corriente (el rojo, suelto). Se cortó bajando a P3 al reiniciarse la
  placa (Sergio desconectó algo sin querer). **Al reiniciarse, la placa vuelve a las
  posiciones de la flash** (las de antes del traycho): se fijaron a mano las de donde se
  paró (Alvin −10.87, Simon −34.95, Teodoro −23.11), calculadas con la IK de la última punta.
