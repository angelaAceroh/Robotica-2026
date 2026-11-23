"""main_control.py: el control cinematico en el brazo de verdad (ESP32, MicroPython).

Es el OTRO programa de la placa. El main.py de siempre (el del nodo `vector`)
recibe un vector de POSICIONES y lleva cada motor a la suya a una velocidad fija.
Este recibe VELOCIDADES, unas 50 veces por segundo, que es lo que manda el
control cinematico del PC (control_nodo: qd = J^-1 (xd_dot + K e)). Es lo mismo
que se hizo en Gazebo con las juntas por velocidad, pero con los motores.

Para usarlo, en Thonny: abrir este archivo y "Guardar como -> Dispositivo
MicroPython -> main.py" (sustituye al de `vector`; para volver, se guarda otra
vez aquel). La posicion de cada motor se guarda en posicion_Alvin.txt,
posicion_Simon.txt y posicion_Teodoro.txt, en pulsos.

LAS UNIDADES son las de Alvin.py, Simon.py y Teodoro.py (los de Sergio, 21/10):
Alvin y Simon en grados de su junta, Teodoro en mm. Lo que se teclea, lo que se
lee y los recorridos de la tabla MOTORES van en esas unidades. Por dentro, el
lazo de cada motor cuenta en grados del eje del motor, que es igual en los tres
(el mismo motorreductor, 5500 pulsos por vuelta): asi valen las mismas ganancias.

COMO MUEVE CADA MOTOR
Cada 10 ms integra la velocidad que le llega en una consigna de posicion y la
sigue con un PI mas anticipativa:

    ref += v * dt
    pwm  = KP*(ref - pos) + integral + KV*v        (y nunca menos de ARRANQUE)

Cerca de la consigna no empuja y FRENA (el motor en corto): parado no tiembla.
Y si va hacia ella tan deprisa que llegaria en ANTICIPO segundos, deja de
empujar ya y frena, antes de pasarse (24/10: Alvin arranca con 700 pero
girando roza mucho menos, se embalaba, se pasaba la banda y volvia con -700:
temblaba +-1.3 grados unas 7 veces por segundo).

Por que no con el lazo de RPM de Alvin.py/Simon.py/Teodoro.py: el control pide
menos de 3 RPM en el hombro y el codo, y ese lazo esta probado entre 10 y 60. Tan
despacio el motor va a tirones por la friccion, y la velocidad medida es ruido;
una consigna de posicion que avanza despacio si se puede seguir. Las ganancias de
partida son las del PID de scara_esp32 que movio el brazo el 11 de septiembre
(que contaba 5440 pulsos por vuelta: casi los 5500 de ahora), pasadas a grados y
a PWM de 0..1023. Se pueden cambiar en caliente con 'g' para sintonizar.

LO QUE LLEGA (lo manda el nodo `puente_placa`; tambien se puede teclear en Thonny)
    V va vs vt *HH     velocidad de Alvin y Simon (grados/s) y Teodoro (mm/s).
                       *HH = suma de los bytes de antes del ' *', modulo 256, en
                       hexadecimal. Si no cuadra se tira la linea: llega otra en
                       20 ms. Tecleada a mano puede ir sin checksum.
    a                  activar los tres: cada uno se engancha donde esta
    a alvin simon      activar solo esos
    q                  soltar los tres (PWM 0) y guardar las posiciones en flash
    fijar simon -35.2  "Simon esta en -35.2": para cuando la flash no dice la verdad
                       (por ejemplo, si lo has movido con Simon.py)
    h simon            home: fijar simon 0. A Alvin no (Alvin.py: NO DARLE A LA H)
    ?                  donde esta cada uno
    l                  los limites (MIN y MAX de la tabla MOTORES)
    g                  las ganancias de cada motor
    g alvin kp 60      cambia una (kp, ki, kv, arranque, arranque_menos, tope, banda,
                       freno, anticipo); no se guarda
    1 | 0              abrir | cerrar la garra (garra.py, GPIO 33; GARRA.md)
    (fijar y h, solo con los motores sueltos)

LO QUE CONTESTA (el nodo `puente_placa` lee estas lineas: no cambies su formato)
    CONTROL listo: Alvin, Simon, Teodoro             al arrancar
    P ms pa ps pt EEE *HH       50 veces por segundo, siempre: el reloj de la placa
                                (ms), la posicion de cada motor y su estado
                                (A activo, L suelto). Con checksum, como la V
    D ms ea es et wa ws wt *HH  5 veces por segundo con alguno activo: lo que va
                                cada uno por detras de su consigna y su PWM
    ACTIVO EEE pa ps pt         respuesta a 'a'
    RECHAZADO <motivo>          'a' no se hace: alguno esta fuera de su recorrido
    PARADO pa ps pt             los tres sueltos ('q', un fallo o sin PC)
    VIGILANTE <motivo>          el PC ha dejado de mandar V
    ERROR <motor> <motivo>      un vigilante ha saltado: se sueltan los tres
    POS pa ps pt | LIMITES ... | GANANCIAS ...       respuestas a '?', 'l' y 'g'
    GARRA abierta 10 | GARRA cerrada 50              respuestas a '1' y '0' (de golpe)
    GARRA abriendo 10 | GARRA cerrando 50            al empezar, si garra.py va en rampa
"""
from machine import Pin, PWM, Timer
import time
import sys
import select

# =================================================
# LOS TRES MOTORES: los pines y las cuentas de Alvin.py, Simon.py y Teodoro.py
# =================================================
# PULSOS_REV  pulsos del encoder por vuelta del motor (22000/4, la prueba de
#             posicion de Sergio)
# POR_VUELTA  lo que avanza su junta en una vuelta del motor, en SUS unidades:
#             Alvin 360 grados; Simon 360*18/21 (Simon.py: 5500*21/18 pulsos por
#             vuelta de junta, la transmision); Teodoro los mm de Teodoro.py
# MIN, MAX    su recorrido, en sus unidades: los que dio Sergio con el brazo
#             delante (21/10). Tienen que ser los de `limites_motor` en
#             config/micropython.yaml (el PC los compara al conectar)
# VEL_MAX     lo mas rapido que se deja ir a su consigna (grados/s o mm/s). El
#             puente no manda mas de su vel_max_motor: 15, 15 y 10 con
#             control_nodo; 90, 190 y 10 por juntas (traycho_juntas.launch.py:
#             Alvin a 14 RPM, 84 grados/s, y Simon a 30, 180)
#
# Teodoro.py: AVANCE_POR_VUELTA_MM = (3.14159265*(0.5 * 20)) + 10 (pinon M0.5 de
# 20 dientes, 31.4 mm, mas 10)
TEODORO_POR_VUELTA = 3.14159265 * (0.5 * 20) + 10
SIMON_POR_VUELTA = 360.0 * 18 / 21
#
#            nombre  IN1  IN2  ENA  ENC_A  ENC_B  PULSOS_REV  POR_VUELTA          MIN     MAX  VEL_MAX
MOTORES = [
    ('Alvin',      4,  16,  17,   26,    25,   5500,  360.0,              -60.0,   60.0,  90.0),
    ('Simon',     18,  19,   5,   12,    13,   5500,  SIMON_POR_VUELTA,  -280.0,    0.0, 190.0),
    ('Teodoro',   21,  22,  23,   27,    14,   5500,  TEODORO_POR_VUELTA, -135.0,   0.0,  10.0),
]
UNIDAD = {'Alvin': 'grados', 'Simon': 'grados', 'Teodoro': 'mm'}
SIN_H = ('Alvin',)        # Alvin.py: "NO DARLE A LA H"

# =================================================
# LAS GANANCIAS DE CADA MOTOR (punto de partida)
# =================================================
# Todo en grados del eje del MOTOR (no de la junta), que es igual en los tres:
# KP        PWM por grado de error
# KI        PWM por grado y por segundo (el integral)
# KV        PWM por grado/s de la consigna (anticipativa: la tension que esa
#           velocidad necesita, sin esperar a que haya error)
# ARRANQUE  PWM minimo cuando hay que moverse hacia +: por debajo el motor no
#           arranca (si con ese no se mueve, sube hasta TOPE en SEG_RAMPA)
# ARR-      lo mismo hacia - (Simon y Teodoro no piden lo mismo hacia cada lado)
# TOPE      PWM maximo que se le deja dar
# BANDA     grados: con menos error que esto no se empuja (si no, el motor
#           tiembla alrededor de la consigna). Un pulso son 0.065 grados
# FRENO     sin empujar (en la banda, o cortando antes de llegar), el motor en
#           corto con este ENA (IN1 = IN2 = 0, el freno «por abajo»: el de
#           arriba, con IN1 = IN2 = 1, bloqueaba a Teodoro el 10/9). 0 = suelto,
#           como antes del 24/10. Soltado ('q') va siempre suelto, para moverlo a mano
# ANTICIPO  s: yendo hacia la consigna a v_med (grados/s, medida), si
#           v_med * ANTICIPO ya pasa de lo que falta, deja de empujar y frena.
#           Y yendo, si se ha adelantado a la consigna poco, la espera frenado en
#           vez de volver hacia atras. 0 = nada de esto
# PULSO     ms: con la consigna quieta y a menos de PULSOS_HASTA, empuja a
#           pulsos de este largo y entre uno y otro espera parado (frenando).
#           Alvin, con lo poco que roza girando, con el suelo de 700 no sabe dar
#           pasos pequenos. 0 = sin pulsos
#
# Son las de scara_esp32 (kp 3, ki 1, kv 0.15, pwmMin 0.12, pwmMax 0.85, en
# radianes y de 0 a 1) pasadas a grados y a 0..1023. ARRANQUE y BANDA de Alvin
# y Simon, medidos en el brazo el 21/10: con 123 Alvin ni se mueve (zumba),
# arranca entre 660 y 700 (en los dos sentidos); Simon arranca con 350 hacia -
# pero hacia + necesita 450, y con banda 0.1 oscila alrededor de la consigna;
# Teodoro baja con 250 pero para SUBIR (contra el
# peso) le hacen falta 350 arriba y 450 mas abajo (con 350 se atasco en -39.5 mm
# y con 400 tampoco subio), y con banda 0.1 (0.01 mm) oscila: con 450 y banda
# 1.0 (0.12 mm) sube y baja a 1 mm/s sin vigilantes. KP, KI y KV, sin sintonizar.
#
# Teodoro, SIN integral (KI 0): con el PWM a 0 se sostiene solo (el peso no lo
# baja), y el roce ya lo vence el suelo de ARRANQUE con su rampa. Su integral
# solo servia para cargarse mientras la cremallera rozaba y, cuando se soltaba,
# embalarlo: bajando a 2 mm/s (se fue 0.62 mm) y subiendo a 1.8 mm/s (0.45 mm),
# y la placa soltaba los tres (21/10, traycho en el brazo).
#
# ARR-, FRENO y ANTICIPO (24/10), con el banco de pruebas (banco_lazo.py, un
# motor de mentira que tiembla como Alvin con el lazo de antes). Simon hacia -
# se movio con 250 (24/10; el 21/10, con 350): 300. Teodoro, como estaba (ni
# freno ni corte: se le mira al final).
#
# Simon, con TOPE 550 (24/10, «limita a Simon, mejor lento pero seguro»): su
# transmision resbala. Contra el tope, con 450 y 550 el motor seguia avanzando
# 0.3-0.8 grados por empujon; con 870, mas. Lejos del tope se mueve con 375-450.
# Pero con 550 se quedaba parado en -33.2 (29/10, «dale mas fuerza»): con 700,
# puesto en caliente, paso y el traycho salio entero. Fijo en 700 desde el 19/11
# («dejalo fijo en 700 en el codigo»).
#
#              nombre      KP     KI    KV   ARRANQUE  ARR-  TOPE  BANDA  FRENO  ANTICIPO PULSO
GANANCIAS = [
    ('Alvin',     54.0,  18.0,  2.7,     700,   700,   870,  0.30,  1023,  0.10,   20),
    ('Simon',     54.0,  18.0,  2.7,     450,   300,   700,  0.30,  1023,  0.10,   20),
    ('Teodoro',   54.0,   0.0,  2.7,     450,   450,   920,  1.00,     0,  0.0,     0),
]

PERIODO_MS = 10           # el lazo de cada motor: 100 veces por segundo
TELEMETRIA_CADA = 2       # una P cada 2 vueltas del lazo: 50 por segundo
DIAGNOSTICO_CADA = 20     # una D cada 20: 5 por segundo

# Vigilantes, tambien en grados del motor. Con el lazo cerrado en la placa, un
# encoder que no cuenta o un motor al reves de su encoder no se quedan en "no
# llega": el motor se embala.
SIN_ORDENES_MS = 300      # sin una V buena en este tiempo: velocidad 0 (sujeta)
SOLTAR_MS = 3000          # y sin ninguna en este: suelta los motores y guarda
ADELANTO_MAX = 1.5        # grados: la consigna no se adelanta a la posicion mas
ERROR_MAX = 3.0           # grados: la posicion se ha ido de la consigna: fallo
GRADOS_AL_REVES = 0.5     # empujando hacia un lado, ha ido esto hacia el otro: fallo
INERCIA_S = 0.05          # ... mas lo que lleve de inercia: velocidad * esto
SEG_ATASCO = 0.25         # tanto tiempo empujando sin parar...
MOVIDO_MIN = 0.1          # ... sin moverse ni esto: atascado o sin encoder
SEG_RAMPA = 0.2           # mientras, el suelo de ARRANQUE sube hasta TOPE en esto
FUERA_TOL = 2.0           # grados fuera del recorrido con los que aun se activa
VEL_QUIETA = 0.5          # grados/s: con la consigna mas lenta que esto, esta quieta
SALIDA_BANDA = 2.0        # quieta y asentado, no empuja hasta este multiplo de BANDA
ESPERA_HASTA = 4.0        # yendo, adelantado a la consigna menos de esto * BANDA: la espera
PULSOS_HASTA = 3.0        # grados: mas cerca, con la consigna quieta, a pulsos (si PULSO)
PULSOS_SALE = 5.0         # ... y ya a pulsos, no se dejan hasta pasar de esto
V_PARADO = 3.0            # grados/s: entre pulso y pulso se espera a ir mas despacio...
ESPERA_PULSO_MS = 40      # ... y por lo menos esto (el motor tarda en responder)
PULSOS_RAMPA = 3          # pulsos seguidos sin moverse: el PWM ya llega a TOPE
PULSOS_ATASCO = 6         # y estos: atascado (el vigilante, como SEG_ATASCO)
PULSO_LARGO_MAX = 6       # sin acercarse, cada pulso mas largo: hasta PULSO * esto
PASADAS_MAX = 2           # a pulsos, se para dentro de BANDA; tras pasarse estas veces,
                          # ya vale SALIDA_BANDA * BANDA (para que no vaya y venga)
PASAR_FORZANDO = 0.6      # grados: un pulso a TOPE, si los de antes no lo acercaron,
                          # puede llevar el motor esto mas alla de la consigna
GIRA_MIN = 0.3            # grados: si los pulsos lo mueven esto (y vuelve), no es un
                          # motor atascado ni un encoder que no cuenta: AVISO, no vigilante

PASOS = [
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
]

GANANCIA_DE = {g[0]: list(g[1:]) for g in GANANCIAS}
CLAVES = ('kp', 'ki', 'kv', 'arranque', 'arranque_menos', 'tope', 'banda', 'freno',
          'anticipo', 'pulso')


def limitar(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Motor:
    """Un motor: su encoder, su consigna y su lazo de posicion.

    Por dentro todo va en grados del eje del motor; `f` pasa de sus unidades
    (grados de junta o mm) a esos grados.
    """

    def __init__(self, nombre, in1, in2, ena, enc_a, enc_b, pulsos_rev, por_vuelta,
                 lim_min, lim_max, vel_max):
        self.nombre = nombre
        self.unidad = UNIDAD[nombre]
        self.lim_min = lim_min    # en sus unidades, como en la tabla
        self.lim_max = lim_max
        self.f = 360.0 / por_vuelta              # grados de motor por unidad
        self.lo = lim_min * self.f               # y en grados de motor
        self.hi = lim_max * self.f
        self.vel_max = vel_max * self.f
        self.IN1 = Pin(in1, Pin.OUT)
        self.IN2 = Pin(in2, Pin.OUT)
        self.ENA = PWM(Pin(ena), freq=1000)
        self.ENC_A = Pin(enc_a, Pin.IN)
        self.ENC_B = Pin(enc_b, Pin.IN)
        self.PULSOS_REV = pulsos_rev
        self.archivo = "posicion_" + nombre + ".txt"
        (self.kp, self.ki, self.kv, self.arranque, self.arranque_menos, self.tope,
         self.banda, self.freno, self.anticipo, self.pulso) = GANANCIA_DE[nombre]

        self.pulsos = self.leer_posicion_flash()
        self.estado_anterior = (self.ENC_A.value() << 1) | self.ENC_B.value()
        self._cb = self.encoder_callback      # una sola vez: el IRQ no crea objetos
        self.ENC_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)
        self.ENC_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)

        self.activo = False
        self.vel = 0.0            # grados/s de motor pedidos: la ultima V
        self.ref = self.posicion()
        self.e = 0.0              # ref - pos
        self.integral = 0.0
        self.pwm = 0.0
        self.asentado = False
        self.signo_empuje = 0     # hacia donde empuja ahora (0 = no empuja)
        self.pos_empuje = 0.0     # donde estaba la ultima vez que avanzo empujando
        self.ms_empuje = 0        # y cuanto lleva empujando desde entonces
        self.holgura_reves = GRADOS_AL_REVES     # lo que se le perdona hacia atras
        self.holgura_e = ERROR_MAX                # y lo que se puede separar de la consigna
        self.ms_pulso = 0         # lo que le queda al pulso de ahora (ms)
        self.pos_pulso = None     # donde empezo el ultimo pulso
        self.signo_pulso = 0      # y hacia donde
        self.largo_pulso = self.pulso   # ms: se acorta cada vez que se pasa
        self.ms_espera = 0        # lo que queda de espera tras el ultimo pulso
        self.pulsos_quietos = 0   # pulsos seguidos que no lo han acercado
        self.pos_mejor = 0.0      # lo mas cerca de la consigna que ha llegado a pulsos
        self.pasadas = 0          # veces que un pulso se ha pasado de la consigna
        self.giro = 0.0           # lo mas que lo han movido los pulsos hacia la consigna
        self.rendido = False      # a pulsos no llega: no empuja hasta otra consigna
        self.aviso = None         # para el lazo principal, como fallo (pero no suelta)
        self.pos_ant = self.ref
        self.v_med = 0.0          # grados/s de motor, medidos
        self.fallo = None
        self.motor(0)

    # ---------------------------------------------------------------- hardware
    def encoder_callback(self, pin):
        estado_actual = (self.ENC_A.value() << 1) | self.ENC_B.value()
        resultado = PASOS[(self.estado_anterior << 2) | estado_actual]
        if resultado != 0:
            self.pulsos += resultado
        self.estado_anterior = estado_actual

    def motor(self, pwm):
        pwm_val = int(max(-1023, min(1023, pwm)))
        if pwm_val > 0:
            self.IN1.value(1)
            self.IN2.value(0)
            self.ENA.duty(pwm_val)
        elif pwm_val < 0:
            self.IN1.value(0)
            self.IN2.value(1)
            self.ENA.duty(abs(pwm_val))
        else:
            self.IN1.value(0)
            self.IN2.value(0)
            self.ENA.duty(0)

    def frenar(self):
        """Activo y sin empujar: el motor en corto (IN1 = IN2 = 0 con ENA = FRENO),
        que le frena mas cuanto mas rapido gira. Con FRENO 0, suelto."""
        self.signo_empuje = 0              # no es empujar: ni rampa ni vigilante
        self.ms_pulso = 0
        self.pwm = 0.0
        if self.freno > 0:
            self.IN1.value(0)
            self.IN2.value(0)
            self.ENA.duty(int(min(1023, self.freno)))
        else:
            self.motor(0)

    def posicion(self):
        """Grados del eje del motor (lo que usa el lazo)."""
        return (self.pulsos / self.PULSOS_REV) * 360.0

    def medida(self):
        """En sus unidades: grados de junta o mm (lo que se lee y se teclea)."""
        return self.posicion() / self.f

    # ------------------------------------------------------------------- flash
    def guardar_posicion_flash(self):
        try:
            with open(self.archivo, "w") as f:
                f.write(str(self.pulsos))
        except Exception as e:
            print("[x] " + self.nombre + ": error al guardar en flash:", e)

    def leer_posicion_flash(self):
        try:
            with open(self.archivo, "r") as f:
                return int(f.read().strip())
        except:
            return 0  # Si el archivo no existe, arranca en 0

    def fijar(self, valor):
        """Donde esta de verdad, en sus unidades. La h es fijar(0)."""
        self.pulsos = int(round(valor * self.f / 360.0 * self.PULSOS_REV))
        self.ref = self.posicion()
        self.guardar_posicion_flash()
        print("[f] %s: ahora esta en %.2f %s" % (self.nombre, self.medida(), self.unidad))

    # --------------------------------------------------------- activar/soltar
    def fuera(self):
        """Motivo si esta demasiado fuera de su recorrido para activarlo, o None."""
        pos = self.posicion()
        if pos < self.lo - FUERA_TOL or pos > self.hi + FUERA_TOL:
            return (self.nombre + " esta en " + "%.2f" % self.medida() + " y su recorrido es ["
                    + "%g, %g" % (self.lim_min, self.lim_max) + "]: si no es verdad, "
                    + "fijar " + self.nombre.lower() + " <donde esta>")
        return None

    def activar(self):
        self.activo = True
        self.ref = self.posicion()
        self.vel = 0.0
        self.integral = 0.0
        self.e = 0.0
        self.asentado = False
        self.signo_empuje = 0
        self.ms_pulso = 0
        self.ms_espera = 0
        self.pos_pulso = None
        self.pulsos_quietos = 0
        self.rendido = False
        self.fallo = None

    def soltar(self):
        self.activo = False
        self.vel = 0.0
        self.integral = 0.0
        self.pwm = 0.0
        self.motor(0)

    # ------------------------------------------------------------------ el lazo
    def paso(self, dt):
        """Cada PERIODO_MS: la consigna avanza con la velocidad pedida y se sigue."""
        pos = self.posicion()
        s = dt / 1000.0
        if s > 0.0:
            self.v_med += 0.5 * ((pos - self.pos_ant) / s - self.v_med)
        self.pos_ant = pos
        if not self.activo:
            self.ref = pos
            self.e = 0.0
            return
        v = limitar(self.vel, -self.vel_max, self.vel_max)
        nuevo = self.ref + v * s
        # Hacia fuera del recorrido no se integra; hacia dentro si (por si se
        # activo un poco fuera). Y la consigna no se adelanta a la posicion mas
        # de ADELANTO_MAX: si el motor no puede seguirla, le espera.
        if v > 0.0:
            tope = max(self.ref, self.hi)
            nuevo = min(nuevo, tope, max(self.ref, pos + ADELANTO_MAX))
        elif v < 0.0:
            tope = min(self.ref, self.lo)
            nuevo = max(nuevo, tope, min(self.ref, pos - ADELANTO_MAX))
        if v != 0.0 and nuevo == tope:
            # La consigna se para en seco contra el tope. El integral, que
            # empujaba hacia alla, haria que el motor se pasara (en la placa
            # simulada, casi un grado a 30 grados/s): fuera.
            self.integral = 0.0
        v_ref = (nuevo - self.ref) / s if s > 0.0 else 0.0
        self.ref = nuevo
        e = self.ref - pos
        self.e = e

        # Con perdon por la inercia que lleva, que se va gastando (1 s para
        # gastar ERROR_MAX): contra el tope a VEL_MAX, Teodoro se pasa 0.37 mm
        # (3.2 grados de motor) en la simulada, cuando ya casi esta parado
        self.holgura_e = max(ERROR_MAX + abs(self.v_med) * INERCIA_S,
                             self.holgura_e - ERROR_MAX * s)
        # A pulsos (ya empezados) y con la consigna quieta, el motor frenado
        # lejos de ella no se ha ido: es la transmision de Alvin, que lo
        # devuelve hacia el brazo (24/10, P3: frenado y quieto a 3.9 grados).
        # Hasta PULSOS_SALE
        limite = self.holgura_e
        if self.pos_pulso is not None and abs(v) < VEL_QUIETA:
            limite = max(limite, PULSOS_SALE)
        if abs(e) > limite:
            self.fallo = ("se ha ido %.2f %s de su consigna. Motor al reves de su "
                          "encoder (cambia IN1 por IN2 en MOTORES) o lo han empujado"
                          % (e / self.f, self.unidad))
            return

        # Dentro de la banda no se empuja (el integral se queda como esta). Si
        # ademas la consigna esta quieta, no vuelve a empujar hasta que el error
        # pase de SALIDA_BANDA veces la banda: si no, con el suelo de ARRANQUE el
        # motor tiembla de un lado al otro de la consigna. «Quieta» es que el PC
        # no pide moverse (v), no que la consigna no avance: parada en
        # ADELANTO_MAX esperando al motor tampoco avanza, y con BANDA 1.0
        # (Teodoro) el motor esperaba a 2.0 y la consigna a el, para siempre.
        quieta = abs(v) < VEL_QUIETA
        # Un pulso forzando (ver los pulsos) cruza la banda empujando
        forzando = self.ms_pulso > 0 and self.pulsos_quietos > 0
        pasar = forzando and self.pulsos_quietos >= PULSOS_RAMPA and self.pasadas == 0
        if not pasar and (abs(e) < self.banda or (quieta and self.asentado
                                                  and abs(e) < SALIDA_BANDA * self.banda)):
            # Asentado si entra en la banda ya casi parado. Si la cruza
            # deprisa, se frena mas alla, y con SALIDA_BANDA se quedaba ahi
            # (hasta 0.6 grados); asi lo acercan los pulsos. Sin pulsos
            # (Teodoro), como siempre
            self.asentado = quieta and (self.pulso <= 0 or abs(self.v_med) < V_PARADO)
            self.frenar()
            return
        self.asentado = False
        # Corte antes de llegar: si va hacia la consigna tan deprisa (respecto a
        # ella) que llegaria en ANTICIPO, deja de empujar y frena. Alvin arranca
        # con 700 pero girando roza mucho menos: con el suelo de ARRANQUE se
        # embalaba, cruzaba la banda entre dos vueltas del lazo y volvia con
        # -700 (24/10, +-1.3 grados a 7 Hz, parado). Si se queda a menos de
        # SALIDA_BANDA bandas, ahi se queda (asentado, como en la banda), pero
        # solo si la consigna se mueve: quieta, lo que falte lo acercan los
        # pulsos hasta la BANDA (24/10: con 2 bandas, los puntos del traycho
        # quedaban a 0.4-0.56 grados de Alvin, 2-4 mm). Menos en un pulso de los que suben porque los de antes no lo
        # acercaron: ese, hasta la banda. Si no, la transmision de Alvin, que
        # cede, no dejaba ni uno: el motor iba deprisa, el brazo no (24/10, P3)
        acerca = self.v_med - v_ref
        if (self.anticipo > 0.0 and acerca * e > 0.0 and not forzando
                and abs(acerca) * self.anticipo >= abs(e) - self.banda):
            self.asentado = False
            self.frenar()
            return
        # Yendo, si se ha adelantado un poco a la consigna, la espera frenado: si
        # volviera hacia atras con el suelo de ARRANQUE, iria a tirones de un lado
        # al otro (en banco_lazo, con poco roce, 60 cambios en 3 s)
        if (self.anticipo > 0.0 and not quieta and e * v < 0.0
                and abs(e) < ESPERA_HASTA * self.banda):
            self.frenar()
            return
        # Con la consigna quieta y cerca: a pulsos. Un empujon de ARRANQUE entero
        # lo lanza mas alla de la banda (Alvin: 700 para arrancar, y girando
        # roza mucho menos); un pulso corto lo mueve poco. Entre pulso y pulso,
        # frenando hasta que para. Si un pulso se pasa, el siguiente dura la
        # mitad (hasta un PERIODO_MS); se para dentro de la BANDA, o de
        # SALIDA_BANDA bandas si ya se ha pasado PASADAS_MAX veces (si no,
        # Alvin, que no da pasos tan cortos, iria y vendria). Si un pulso no lo ACERCA (con
        # signo: lo mas cerca que ha estado, medido parado), el siguiente con
        # mas PWM (de ARRANQUE a TOPE en PULSOS_RAMPA) y mas largo (hasta
        # PULSO_LARGO_MAX veces), y si ni asi: atascado.
        #
        # Con signo por Alvin (24/10, yendo a P3): su transmision cede, y cada
        # pulso movia el motor medio grado y lo devolvia, un poco mas atras cada
        # vez. Como «se movia», ni subia el PWM ni saltaba el vigilante: 2.8
        # grados por detras de la consigna para siempre
        #
        # Y otra vez en P3 (24/10, 12:37): el brazo atascado en -28 y la
        # consigna en -31. El motor volvia hasta -28 y el error iba y venia
        # alrededor de los 3 grados de PULSOS_HASTA: cada vez que pasaba, fuera
        # de los pulsos y vuelta a empezar con el mas flojo (35 s de pulsos de
        # 700 sin llegar). Ya a pulsos, se sigue hasta PULSOS_SALE. Y como un
        # pulso que llega a la consigna no tensa la transmision mas que eso,
        # los de TOPE pueden pasarse PASAR_FORZANDO (si aun no se ha pasado
        # ninguno). Si ni asi, y los pulsos lo mueven y vuelve (la
        # transmision cede, el motor y el encoder van bien), deja de empujar
        # hasta otra consigna y lo dice (AVISO): el vigilante, solo si los
        # pulsos no lo mueven
        hasta = PULSOS_SALE if self.pos_pulso is not None else PULSOS_HASTA
        if self.pulso > 0 and quieta and abs(e) < hasta:
            self.integral = 0.0
            if self.rendido:
                self.frenar()
                return
            if self.ms_pulso <= 0:
                # Parado de verdad: ESPERA_PULSO_MS seguidos despacio. Si no,
                # con la transmision tensa se decidia en el punto en que se da
                # la vuelta (un momento parado), antes de que la transmision
                # lo devolviera: «se ha pasado» sin haberse pasado
                if abs(self.v_med) > V_PARADO:
                    self.ms_espera = max(self.ms_espera, ESPERA_PULSO_MS)
                if self.ms_espera > 0:
                    self.ms_espera -= dt
                    self.frenar()
                    return
                signo = 1 if e > 0.0 else -1
                if self.pos_pulso is None:
                    self.largo_pulso = self.pulso
                    self.pulsos_quietos = 0
                    self.pos_mejor = pos
                    self.pasadas = 0
                    self.giro = 0.0
                elif signo != self.signo_pulso:            # se ha pasado
                    self.pulsos_quietos = 0
                    self.pos_mejor = pos
                    self.largo_pulso = max(PERIODO_MS, self.largo_pulso // 2)
                    self.pasadas += 1
                    if self.pasadas >= PASADAS_MAX and abs(e) < SALIDA_BANDA * self.banda:
                        self.asentado = True
                        self.frenar()
                        return
                elif (pos - self.pos_mejor) * signo >= MOVIDO_MIN:     # se ha acercado
                    self.pulsos_quietos = 0
                    self.pos_mejor = pos
                elif abs(e) < SALIDA_BANDA * self.banda:
                    # Cerca (donde antes ya se daba por bueno) y este pulso no
                    # lo ha acercado: ahi se queda. Forzarlo aqui con pulsos
                    # mas fuertes, con una transmision que cede como la de
                    # Alvin, lo dejaba a veces mas lejos y al final saltaba
                    # el vigilante (banco_elastico). Sin olvidar los pulsos:
                    # si la transmision lo devuelve mas alla, siguen subiendo
                    self.asentado = True
                    self.frenar()
                    return
                else:
                    self.pulsos_quietos += 1
                if self.pulsos_quietos >= PULSOS_ATASCO and self.giro >= GIRA_MIN:
                    self.rendido = True
                    self.aviso = ("no llega: le faltan %.2f %s. %d pulsos (hasta PWM %d) lo "
                                  "mueven hasta %.2f y vuelve: la transmision cede o el brazo "
                                  "esta frenado. No empuja hasta otra consigna"
                                  % (e / self.f, self.unidad, PULSOS_ATASCO, self.tope,
                                     self.giro / self.f))
                    self.frenar()
                    return
                if self.pulsos_quietos >= PULSOS_ATASCO:
                    self.fallo = ("%d pulsos (hasta PWM %d y %d ms) y no se ha acercado ni "
                                  "%.3f %s: atascado o sin encoder"
                                  % (PULSOS_ATASCO, self.tope, self.largo_pulso * PULSO_LARGO_MAX,
                                     MOVIDO_MIN / self.f, self.unidad))
                    return
                self.pos_pulso = pos
                self.signo_pulso = signo
                self.ms_pulso = self.largo_pulso * min(1 + self.pulsos_quietos, PULSO_LARGO_MAX)
            signo = self.signo_pulso
            self.giro = max(self.giro, (pos - self.pos_pulso) * signo)
            if e * signo <= 0.0 and not (pasar and -e * signo < PASAR_FORZANDO):
                # Se ha pasado a mitad de pulso: no se le empuja de vuelta, se
                # espera parado y el siguiente ya sabe que se ha pasado
                self.ms_espera = ESPERA_PULSO_MS
                self.frenar()
                return
            arranque = self.arranque if signo > 0 else self.arranque_menos
            u = signo * (arranque + (self.tope - arranque)
                         * min(1.0, self.pulsos_quietos / PULSOS_RAMPA))
            self.ms_pulso -= dt
            if self.ms_pulso <= 0:
                self.ms_espera = ESPERA_PULSO_MS
            self.signo_empuje = 0
            self.pwm = u
            self.motor(u)
            return
        self.pos_pulso = None
        self.pulsos_quietos = 0
        self.rendido = False
        # El integral nunca empuja en contra de la consigna: lo que cargo yendo
        # hacia un lado no sirve para volver. Si no, se gasta a KI * ADELANTO_MAX
        # (27 de PWM por segundo) y mientras el motor empuja al reves sin moverse
        # (21/10, traycho: Alvin, al cambiar de sentido, 10 s quieto con PWM de
        # -311 a -23, y la punta se quedo 53 mm atras)
        if self.integral * e < 0.0:
            self.integral = 0.0
        u_pd = self.kp * e + self.kv * v_ref
        tentativo = u_pd + self.integral
        saturado = tentativo > self.tope or tentativo < -self.tope
        if not saturado or (tentativo > 0.0) != (e > 0.0):
            self.integral = limitar(self.integral + self.ki * e * s, -self.tope, self.tope)
        u = limitar(u_pd + self.integral, -self.tope, self.tope)
        # Suelo de arranque: si el lazo pide algo, que pida al menos lo que hace
        # falta para que el eje se mueva. Si aun asi no se mueve nada (la
        # cremallera de Teodoro pide mas o menos segun la altura), el suelo sube
        # hasta TOPE en SEG_RAMPA; en cuanto avanza, vuelve a ARRANQUE. Si ni
        # con TOPE se mueve, a los SEG_ATASCO salta el vigilante.
        #
        # Solo cuando empuja hacia la consigna. Si empuja hacia el otro lado es
        # el integral, y cuando el motor ya va por delante de la consigna el
        # suelo convertia ese empuje en ARRANQUE y lo seguia llevando (21/10,
        # traycho: Teodoro, cuando aun tenia integral, se paso 0.46 mm). Sin
        # suelo, el integral solo lo sujeta, y se va gastando.
        if (u > 0.0) != (e > 0.0):
            self.signo_empuje = 0          # no es empujar: ni rampa ni vigilante
            self.pwm = u
            self.motor(u)
            return
        # La rampa, mientras no avance hacia donde empuja. Con signo: al cambiar
        # de sentido aun se iba hacia el otro lado por la inercia, y eso contaba
        # como «se ha movido»: el suelo no subia y a los SEG_ATASCO saltaba el
        # vigilante con el PWM en ARRANQUE (24/10, en banco_lazo; como Alvin en
        # el brazo el 23/10, atascado en 26.8 y 32.1 grados)
        signo = 1 if u > 0.0 else -1
        arranque = self.arranque if signo > 0 else self.arranque_menos
        suelo = arranque
        if signo == self.signo_empuje and (pos - self.pos_empuje) * signo < MOVIDO_MIN:
            suelo += (self.tope - arranque) * min(1.0, self.ms_empuje / (SEG_RAMPA * 1000.0))
        if 0.0 < u < suelo:
            u = suelo
        elif -suelo < u < 0.0:
            u = -suelo
        self.vigilar_empuje(dt, pos, u)
        self.pwm = u
        self.motor(u)

    def vigilar_empuje(self, dt, pos, u):
        """Mientras empuja, que el encoder avance y hacia donde se empuja.

        Con el lazo cerrado en la placa, un encoder que no cuenta deja al motor
        girando a ciegas, y uno al reves lo embala: hay que verlo pronto. Al
        cambiar de sentido sigue un poco hacia el otro lado por inercia, y mas
        cuanto mas rapido iba (contra el tope a toda velocidad, por ejemplo):
        eso se le perdona; GRADOS_AL_REVES mas, ya no.
        """
        signo = 1 if u > 0.0 else -1
        if signo != self.signo_empuje:
            self.signo_empuje = signo
            self.pos_empuje = pos
            self.ms_empuje = 0
            self.holgura_reves = GRADOS_AL_REVES + abs(self.v_med) * INERCIA_S
            return
        avance = (pos - self.pos_empuje) * signo
        if avance < -self.holgura_reves:
            self.fallo = ("va al reves: empujando hacia %s ha ido %.2f %s hacia el otro "
                          "lado. Motor al reves de su encoder: cambia IN1 por IN2 en MOTORES"
                          % ('+' if signo > 0 else '-', -avance / self.f, self.unidad))
        elif avance >= MOVIDO_MIN:
            self.pos_empuje = pos          # avanza: se vuelve a contar desde aqui
            self.ms_empuje = 0
        else:
            self.ms_empuje += dt
            if self.ms_empuje >= SEG_ATASCO * 1000:
                self.fallo = ("%.2f s empujando (de PWM %d a %d) y no se ha movido ni %.3f %s: "
                              "el encoder no cuenta, falta potencia (12 V) o esta bloqueado"
                              % (SEG_ATASCO, self.arranque if signo > 0 else self.arranque_menos,
                                 int(abs(u)), MOVIDO_MIN / self.f, self.unidad))


# =================================================
# ARRANQUE
# =================================================
motores = [Motor(*m) for m in MOTORES]
POR_NOMBRE = {m.nombre.lower(): m for m in motores}

# La garra (un servo en el GPIO 33), si garra.py esta en la placa: al importarlo se
# abre. Despues de los motores, que es como se probo en la placa (19/11). Sin el, o
# si no arranca, los motores van igual
try:
    import garra
except ImportError:
    garra = None
except Exception as e:
    print("[x] garra.py no arranca: " + str(e))
    garra = None

flag_muestreo = False


def tick(timer):
    global flag_muestreo
    flag_muestreo = True


timer = Timer(0)
timer.init(period=PERIODO_MS, mode=Timer.PERIODIC, callback=tick)


def checksum(texto):
    """Suma de los bytes, modulo 256, en hexadecimal: la misma cuenta que el PC."""
    return "%02X" % (sum(texto.encode()) & 0xFF)


def con_checksum(texto):
    return texto + " *" + checksum(texto)


def posiciones():
    return " ".join("%.2f" % m.medida() for m in motores)


def estados():
    return "".join("A" if m.activo else "L" for m in motores)


def guardar():
    for m in motores:
        m.guardar_posicion_flash()


def soltar_todos(motivo=None):
    for m in motores:
        m.soltar()
    guardar()
    if motivo:
        print("VIGILANTE " + motivo)
    print("PARADO " + posiciones())


def leer_v(linea):
    """'V va vs vt *HH' -> 3 velocidades. Con checksum malo, ValueError."""
    if '*' in linea:
        cuerpo, llegado = linea.rsplit('*', 1)
        cuerpo, llegado = cuerpo.strip(), llegado.strip().upper()
        if checksum(cuerpo) != llegado:
            raise ValueError("checksum")
        linea = cuerpo
    partes = linea.split()
    if len(partes) != 1 + len(motores):
        raise ValueError("hacen falta %d velocidades" % len(motores))
    return [float(p) for p in partes[1:]]


def activar(nombres):
    elegidos = motores if not nombres else [POR_NOMBRE.get(n) for n in nombres]
    if None in elegidos:
        print("RECHAZADO no conozco " + " ".join(n for n in nombres if n not in POR_NOMBRE))
        return
    fuera = [m.fuera() for m in elegidos if m.fuera()]
    if fuera:
        print("RECHAZADO " + "; ".join(fuera))
        return
    for m in elegidos:
        m.activar()
    print("ACTIVO " + estados() + " " + posiciones())


def cambiar_ganancia(palabras):
    if len(palabras) == 1:
        for m in motores:
            print("GANANCIAS " + m.nombre + " " + " ".join(
                "%s %g" % (c, getattr(m, c)) for c in CLAVES))
        return
    if len(palabras) != 4 or palabras[1] not in POR_NOMBRE or palabras[2] not in CLAVES:
        print("[x] uso: g | g <motor> <" + "|".join(CLAVES) + "> <valor>")
        return
    m = POR_NOMBRE[palabras[1]]
    try:
        valor = float(palabras[3])
    except ValueError:
        print("[x] " + palabras[3] + " no es un numero")
        return
    setattr(m, palabras[2], valor)
    print("GANANCIAS " + m.nombre + " " + " ".join("%s %g" % (c, getattr(m, c)) for c in CLAVES))


def atender(linea):
    global t_ultima_v, sin_ordenes
    if linea[0] in 'Vv' and (len(linea) == 1 or linea[1] == ' '):
        try:
            v = leer_v(linea)
        except ValueError as e:
            print("ERROR V " + str(e))
            return
        for m, vel in zip(motores, v):
            m.vel = vel * m.f if m.activo else 0.0
        t_ultima_v = time.ticks_ms()
        sin_ordenes = False
        return
    palabras = linea.lower().split()
    orden = palabras[0]
    if orden == 'q':
        soltar_todos()
    elif orden == 'a':
        t_ultima_v = time.ticks_ms()      # tiempo para que el PC empiece a mandar
        sin_ordenes = False
        activar(palabras[1:])
    elif orden == '?':
        print("POS " + posiciones())
    elif orden == 'l':
        print("LIMITES " + " ".join("%s %g %g" % (m.nombre, m.lim_min, m.lim_max)
                                    for m in motores))
    elif orden == 'g':
        cambiar_ganancia(palabras)
    elif orden in ('h', 'fijar'):
        fijar(orden, palabras[1:])
    elif orden in ('1', '0'):
        if garra is None:
            print("[x] no hay garra.py en la placa (GARRA.md)")
        else:
            garra.orden(orden)
    else:
        print("[x] no entiendo: " + linea)


def fijar(orden, args):
    """'h simon' o 'fijar simon -35.2': donde esta el motor de verdad."""
    if any(m.activo for m in motores):
        print("[x] h y fijar, solo con los motores sueltos: manda q antes")
        return
    uso = "h <motor>" if orden == 'h' else "fijar <motor> <donde esta, grados o mm>"
    if len(args) != (1 if orden == 'h' else 2) or args[0] not in POR_NOMBRE:
        print("[x] uso: " + uso)
        return
    m = POR_NOMBRE[args[0]]
    if orden == 'h' and m.nombre in SIN_H:
        print("[x] a %s no se le da la h (NO DARLE A LA H): su cero es el bueno. Si de "
              "verdad hay que cambiarlo: fijar %s 0" % (m.nombre, args[0]))
        return
    try:
        valor = 0.0 if orden == 'h' else float(args[1])
    except ValueError:
        print("[x] " + args[1] + " no es un numero")
        return
    m.fijar(valor)


def leer_consola():
    """Las lineas que hayan llegado (puede haber varias: el PC manda 50 por segundo)."""
    lineas = []
    while len(lineas) < 8 and select.select([sys.stdin], [], [], 0)[0]:
        linea = sys.stdin.readline().strip()
        if linea:                 # un Enter suelto o el \r\n del PC: no es una orden
            lineas.append(linea)
    return lineas


print("CONTROL listo: " + ", ".join(m.nombre for m in motores)
      + " (control cinematico: velocidades por el USB)")
for m in motores:
    print(" %s: ultima posicion en flash %.2f %s, recorrido [%g, %g]"
          % (m.nombre, m.medida(), m.unidad, m.lim_min, m.lim_max))
print("Ordenes: a = activar | q = soltar | V va vs vt = velocidades (grados/s, mm/s) | "
      "fijar <motor> <pos> | h <motor> | ? | l | g"
      + (" | 1/0 = abrir/cerrar la garra" if garra else " | (sin garra.py)"))
print("=" * 50)

t_ultima_v = time.ticks_ms()
sin_ordenes = False
tiempo_anterior = time.ticks_ms()
tic = 0

while True:
    for linea in leer_consola():
        atender(linea)

    if flag_muestreo:
        flag_muestreo = False
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, tiempo_anterior)
        tiempo_anterior = ahora

        # El PC ha dejado de mandar: primero quietos (sujetando), luego sueltos.
        if any(m.activo for m in motores):
            callado = time.ticks_diff(ahora, t_ultima_v)
            if callado > SOLTAR_MS:
                soltar_todos("sin ordenes del PC en %.1f s: suelto los motores"
                             % (SOLTAR_MS / 1000))
            elif callado > SIN_ORDENES_MS and not sin_ordenes:
                sin_ordenes = True
                for m in motores:
                    m.vel = 0.0
                print("VIGILANTE sin ordenes del PC en %d ms: quietos, sujetando"
                      % SIN_ORDENES_MS)

        for m in motores:
            m.paso(dt)
        if garra is not None:
            garra.paso()                   # rampa y detector de tension (garra.py)

        # Si un vigilante salta en uno, se sueltan los tres: el movimiento es
        # conjunto y los otros dos solos llevarian la punta a donde no debe.
        for m in motores:
            if m.aviso:
                print("AVISO " + m.nombre + " " + m.aviso)
                m.aviso = None
        caidos = [m for m in motores if m.fallo]
        if caidos:
            for m in caidos:
                print("ERROR " + m.nombre + " " + m.fallo)
                m.fallo = None
            soltar_todos()

        tic += 1
        if tic % TELEMETRIA_CADA == 0:
            print(con_checksum("P %d %s %s" % (ahora, " ".join(
                "%.3f" % m.medida() for m in motores), estados())))
        if tic % DIAGNOSTICO_CADA == 0 and any(m.activo for m in motores):
            print(con_checksum("D %d %s %s" % (ahora, " ".join(
                "%.3f" % (m.e / m.f) for m in motores), " ".join(
                "%d" % int(m.pwm) for m in motores))))
