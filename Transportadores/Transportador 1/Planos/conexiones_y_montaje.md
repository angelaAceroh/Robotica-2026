# Montaje físico — conexiones

## Mapa de pines

### Encoders → ESP32

Solo se usan pines **rotulados con número** en tu placa de 38 pines.

| Motor | Canal A (amarillo) | Canal B (blanco) |
|---|---|---|
| M1 — base (J1) | **GPIO 34** | **GPIO 35** |
| M2 — hombro (J2) | **GPIO 32** | **GPIO 33** |
| M3 — codo (J3) | **GPIO 25** | **GPIO 26** |

Las 34 y 35 son de **solo entrada** y no tienen pull-up interno, pero el encoder
TSINY ya trae uno de 10 kΩ en su placa, así que valen tal cual.

Sí: **se usan los dos canales de cada encoder**. Con uno solo sabes cuánto se
movió pero no hacia dónde, y además pierdes tres cuartas partes de la
resolución.

### L298N #1 → motores 1 y 2

| L298N | ESP32 | Qué hace |
|---|---|---|
| ENA | **GPIO 27** | PWM del motor 1 |
| IN1 | **GPIO 14** | sentido motor 1 |
| IN2 | **GPIO 13** | sentido motor 1 |
| ENB | **GPIO 4** | PWM del motor 2 |
| IN3 | **GPIO 16** | sentido motor 2 |
| IN4 | **GPIO 17** | sentido motor 2 |

### L298N #2 → motor 3 (le sobra un canal)

| L298N | ESP32 | Qué hace |
|---|---|---|
| ENA | **GPIO 21** | PWM del motor 3 |
| IN1 | **GPIO 19** | sentido motor 3 |
| IN2 | **GPIO 18** | sentido motor 3 |

### Gripper = J4 — motorreductor N20 con encoder (canal B del L298N #2)

El plan del servo se descartó. En su lugar va un N20 con encoder de cuadratura,
y **la GPIO 23, que era la señal del servo, es ahora IN4**.

| Hilo del motor | A dónde |
|---|---|
| blanco (M1) | **OUT3** del L298N #2 |
| rojo (M2) | **OUT4** del L298N #2 |
| negro (VCC) | **3,3 V** de la ESP32 — funciona, no hacen falta divisores |
| azul (GND) | GND común |
| verde (C1) | **GPIO 36**, rotulada **VP** |
| amarillo (C2) | **GPIO 39**, rotulada **VN** |

| L298N #2 | ESP32 | Qué hace |
|---|---|---|
| ENB | **GPIO 2** | PWM del gripper |
| IN3 | **GPIO 22** | sentido |
| IN4 | **GPIO 23** | sentido |

La hoja del TSINY-8370 **no sirve para este motor**: ahí negro y azul están al
contrario, y cablear por ella invierte la alimentación del encoder.

#### Por qué ENB va a la GPIO 2 y no al jumper

Con el jumper puesto, el enable queda fijo a 5 V y el canal está habilitado
**desde el instante del reset**. Y al reiniciar la placa, IN3 e IN4 quedan
flotando hasta que arranca MicroPython. Resultado: el motor empujaba solo, se
quedaba **bloqueado contra el tope de entrada**, y ahí seguía hasta que alguien
apagaba la fuente. Es casi seguro lo que coció el colector del primer motor —
el multímetro daba OL justo con las palas recogidas, que es esa posición.

A eso se sumaba que `control_vel.py` configuraba la GPIO 23 como PWM de servo a
50 Hz y le mandaba pulsos al arrancar, o sea que además de flotar, el firmware
empujaba activamente.

La **GPIO 2 lleva pull-down interno durante el reset, por hardware**. El canal
arranca deshabilitado sin depender del software, y eso lo resuelve de raíz.
Verificado: 5 s después de un reinicio, 0 cuentas de deriva.

**Importante: quita los jumpers de ENA y ENB** de los dos L298N. Vienen puestos
de fábrica y fuerzan el enable a 5 V fijo; con el jumper puesto el PWM no hace
nada y el motor va siempre a tope.

#### Límites medidos (2026-11-13)

| | Cuentas |
|---|---|
| Tope del lado de entrada = cero | 0 |
| Cerrado, posición de trabajo | 70 |
| Abierto, posición de trabajo | 510 |
| **Máximo seguro** | **580** |

**Pasado 580 no hay tope: las palas se salen de la guía.** El lado de entrada
sí tiene tope firme, y por eso el referenciado solo busca hacia ahí.

Velocidad: **~770 cuentas/s al 25 % de duty**, la carrera entera en 0,8 s.

#### Trama serie

```
v1,v2,v3[,ignorado[,g_ref]]
```

Las tres primeras en grados/s de junta. El cuarto campo era el servo y el
firmware lo **ignora**, para que los programas de trayectoria sigan mandando su
trama sin cambios. El quinto es la posición del gripper en cuentas, y
`g_ref = -1000` le pide al firmware que lo referencie.

Telemetría: `th1,vel1,th2,vel2,th3,vel3,g_cuentas`.

---

## Pines que NO se pueden usar, y por qué

Cambié el mapa que traía el código anterior. Usaba la **GPIO 12** como DIR2 del
motor 1, y esa es de *strapping*: si está en alto al arrancar, la ESP32
configura la flash a 1,8 V y **puede no arrancar**. Es un fallo que aparece de
forma intermitente y vuelve loco a cualquiera.

Déjalas libres:

| Pin | Por qué |
|---|---|
| GPIO 0, 2, 5, 12, 15 | strapping: deciden cómo arranca la placa |
| GPIO 1, 3 | son TX y RX: por ahí habla con el PC |
| GPIO 6 … 11 | van a la memoria flash, no están disponibles |
| GPIO 36, 39 | existen, pero vienen rotuladas **VP** y **VN** |

**Una comprobación sobre las GPIO 16 y 17:** están libres en los módulos
ESP32-WROOM, que son los de la gran mayoría de placas de 38 pines. Si la tuya
lleva un **WROVER** (tiene PSRAM), esas dos están ocupadas internamente y hay
que moverlas — se nota porque el motor 2 no responde en absoluto. En ese caso
pásalas a la 22 y a la 5… o dime y te rehago el mapa.

---

## Alimentación

Tu planteamiento es el correcto. Concretando:

```
Fuente 12 V ──┬── L298N #1  (+12V)
              └── L298N #2  (+12V)

ESP32 ── por USB o por su propio 5 V

GND de la fuente ──┬── GND L298N #1
                   ├── GND L298N #2
                   └── GND de la ESP32      <- IMPRESCINDIBLE
```

Sin la tierra común, la ESP32 y los drivers no comparten referencia: las
señales IN1/IN2/ENA quedan flotando y el comportamiento es errático. Es el
fallo número uno en estos montajes.

### Los encoders: 5 V o 3,3 V

La hoja del TSINY dice **Vcc 5–24 V**, y lleva un regulador en la placa. Pero
hay un problema: **las GPIO de la ESP32 son de 3,3 V y no toleran 5 V**. Si
alimentas el encoder a 5 V, sus salidas A y B oscilan entre 0 y 5 V y le estás
metiendo 5 V a la ESP32.

Dos caminos:

**a) Alimentarlos a 3,3 V** (del pin 3V3 de la ESP32). Queda por debajo del
rango de la hoja, pero estos encoders Hall suelen funcionar. **Pruébalo
primero**: conecta uno, gira el eje a mano y mira si las cuentas suben y bajan
limpio. Si funciona, es la solución más simple y no hace falta nada más.

**b) Si a 3,3 V no va: alimentar a 5 V y bajar las señales** con un divisor en
cada línea A y B:

```
  A del encoder ──[ 10 kΩ ]──┬── GPIO de la ESP32
                             │
                          [ 20 kΩ ]
                             │
                            GND
```

5 V × 20/(10+20) = 3,3 V. Son seis divisores, uno por señal. El 5 V puedes
sacarlo del propio L298N, que trae regulador.

**No conectes A y B a 5 V directamente a la ESP32.** Puede que aguante un rato
y luego se muera el pin.

---

## Antes de lanzar la trayectoria: prueba motor por motor

1. Carga el firmware y abre el monitor serie a 115200.
2. Coloca el brazo **plegado** (ver abajo) y enciende.
3. Manda una consigna pequeña a una sola junta, por ejemplo `0,80,-150` y luego
   `10,80,-150`. La base debería girar 10°.
4. Mira la telemetría: la segunda cifra de cada par es el ángulo medido.
   - Si no se mueve → revisa el jumper de ENA, la alimentación, la tierra común.
   - Si se mueve **al revés** → intercambia IN1 e IN2 de ese motor, o los dos
     cables del motor.
   - Si **sale disparado y no para** → el encoder está al revés respecto del
     motor: intercambia A y B de ese encoder. Esto es importante, porque con la
     realimentación invertida el lazo es inestable y el motor acelera hasta el
     tope.
5. Comprueba la escala: pide 90° y mide con un transportador que giren 90° de
   verdad. Si gira el triple o la tercera parte, el que está mal es `N_CORREA`;
   si se queda corto o largo por otro factor, es el `PPR`.

Haz esto con **las tres juntas por separado** antes de lanzar el ciclo.
