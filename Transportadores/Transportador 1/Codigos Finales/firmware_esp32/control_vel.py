"""
CONTROL DE VELOCIDAD DE LAS TRES JUNTAS  -  MicroPython en la ESP32
Robot transportador 3R - UMNG

Es el lazo que diseno el equipo en Simulink: la ESP32 regula la VELOCIDAD
de cada junta, no su posicion.  La posicion se cierra fuera, en el PC, que
es quien conoce la trayectoria y la cinematica.

    PC    trayectoria -> cinematica inversa diferencial -> q_punto deseada
          (con correccion cartesiana:  xe_punto = xd_punto + K*error)
            |  serie, cada 50 ms:  "v1,v2,v3[,-,g_ref]" en GRADOS/SEGUNDO (g_ref en cuentas)
            v
    ESP32   lazo de velocidad por motor  ->  PWM del L298N

--------------------------------------------------------- el controlador
Del Simulink del equipo, para el motor 3:

    C(z) = (0.551 - 0.4 z^-1) / [ (1 - z^-1)(1 - 0.03448 z^-1) ]

Desarrollando el denominador, la ecuacion en diferencias es

    u(k) = 1.03448 u(k-1) - 0.03448 u(k-2) + 0.551 e(k) - 0.4 e(k-1)

El polo en z = 1 es el integrador: es lo que anula el error de velocidad
en regimen permanente.  (En el .ino original estaba puesto s0 = -0.00001
en vez de -1, lo que lo dejaba SIN integrador pese al comentario que decia
lo contrario.)

Las plantas identificadas en el Simulink:
    M1   3.600 / (0.25 s + 1)
    M2   41.58 / (s + 4.761)  =  8.733 / (0.21 s + 1)
    M3   3.348 / (0.25 s + 1)

Solo se veia el numerador del controlador del motor 3.  Los de M1 y M2 se
estiman escalando con la ganancia de su planta -a planta mas viva, menos
ganancia de controlador-, que conserva el lazo abierto.  Si consigues los
numeradores reales en Simulink, se cambian en CONTROLADORES y listo.
"""
import math
import select
import sys
import time

from machine import PWM, Encoder, Pin

# =====================================================================
#  MEDIDAS   (confirmadas en el robot: 90 grados = 1620 cuentas)
# =====================================================================
PPR_BASE = 540           # 12 ppr del encoder x 45 de la caja (motores viejos)
FASES = 4                # decodificacion en cuadratura
N_CORREA = 3             # polea 60/20

# CUENTAS POR VUELTA DE JUNTA, UNA POR JUNTA.  Dejo de ser un numero unico el
# 2026-10-28, cuando J2 monto el motor de 60 rpm: su caja es 98:1 en vez de
# 45:1, asi que da mas del doble de cuentas por el mismo angulo.
#
#   J1, J3 (130 rpm, 45:1):  12 x 45 x 4 x 3 = 6480
#   J2     ( 60 rpm, 98:1):  12 x 98 x 4 x 3 = 14112
#
# La reduccion de J2 se MIDIO el 2026-10-28 girando el eje de salida tres
# vueltas con el propio motor: 13963 cuentas, o sea 97,0:1, que con un 1 %
# de margen por el tiempo de reaccion es 98:1.
POR_VUELTA = (6480, 14112, 6480)
GRADOS_J   = tuple(360.0 / v for v in POR_VUELTA)   # grados por cuenta

# se conserva para el codigo que todavia asume un valor unico (herramientas
# de banco); el lazo de control usa el de cada junta
GRADOS = GRADOS_J[0]                            # 0.0556 deg/cuenta

TS_MS = 50
TS = TS_MS / 1000.0
PWM_FREQ = 1000              # el L298N es un puente bipolar lento: por
                             # encima de 2-3 kHz pierde bastante
U_SAT = 1.0
TIMEOUT_MS = 500             # en velocidad el paro tiene que ser RAPIDO
FILTRO = 0.30                # suavizado de la velocidad medida (0 a 1)

# PROTECCION DE EMBALAMIENTO
# Si se pide velocidad en un sentido y la junta se mueve en el
# CONTRARIO, el lazo esta realimentado al reves: satura y empuja cada
# vez mas fuerte en la direccion equivocada.  Es lo que tiro el hombro
# de 80 a -70 grados acelerando hasta 238 deg/s.  Esto lo corta en
# tres ciclos, 150 ms.
CICLOS_FUGA = 3
# deg/s en sentido opuesto que se toleran, POR JUNTA.
# 2026-11-24: en J2 y J3 la GRAVEDAD tambien las mueve al reves de lo pedido
# (bajando de golpe tras pasarse hacia arriba).  Con 15 deg/s esto saltaba
# en casi todas las pasadas al empezar la aproximacion, cortaba el motor y
# vaciaba el integrador: la junta caia libre por su peso.  El 24-nov J3 cayo
# 1.5 s, de -83 a -121 grados, con la PC pidiendole SUBIR a 60 deg/s.  Un
# lazo invertido de verdad ACELERA sin parar (238 deg/s aquel dia); una caida
# con reductora se queda en 30-50 deg/s.  Por eso en J2 y J3 el umbral es 80.
VEL_CONTRARIA = {'J1': 15.0, 'J2': 80.0, 'J3': 80.0}
VEL_ABSURDA = 300.0          # deg/s: por encima algo va muy mal

# Numerador del controlador por motor:  u += b0*e(k) + b1*e(k-1)
# y polos en z=1 (integrador) y z=p (el filtro rapido del Simulink).
#
# OJO CON LAS UNIDADES: en el Simulink las plantas dan RADIANES/segundo
# por unidad de entrada, asi que estos coeficientes esperan el error en
# rad/s.  Aqui se trabaja en grados/s de cara al usuario, de modo que el
# error se convierte a rad/s ANTES de entrar al controlador.  Sin esa
# conversion la ganancia queda 57.3 veces mas alta y el lazo se va a
# bang-bang: u saltando entre +1 y -1 en ciclos alternos.
#              b0      b1      p
# ESCALADOS el 2026-10-28 midiendo la ganancia de planta de cada junta
# (escalon en lazo abierto, velocidad de regimen contra mando):
#
#     J1  K = 445.0 deg/s por unidad de mando   -> factor 0.30
#     J2  K = 131.8                              -> factor 1.00 (referencia)
#     J3  K = 247.2                              -> factor 0.53
#
# El factor es 131.8/K: se lleva a las tres a la misma ganancia de LAZO que
# tiene J2, que es la unica que iba bien.  J1 bailaba porque tenia 3.4 veces
# mas planta que J2 Y ademas el controlador mas alto; el producto de ambos
# era lo que la desestabilizaba.
CONTROLADORES = {
    'J1': (0.152, -0.110, 0.02744),
    'J2': (0.211, -0.153, 0.02850),
    'J3': (0.294, -0.213, 0.03448),
}

# =====================================================================
#  PREALIMENTACION DE ROZAMIENTO
#
#  Medido con barrido(0) en el robot:
#        40 deg/s -> u 0.735      120 deg/s -> u 0.893
#  Ajustando la recta:   u = 0.656 + 0.001975 * v
#
#  Ese 0.656 es rozamiento puro: dos tercios del PWM se van en vencerlo
#  antes de mover nada.  Por eso por debajo de unos 30 deg/s el lazo no
#  seguia la consigna: se quedaba peleando dentro de la zona muerta.
#
#  Dandosela hecha al controlador, el integrador solo tiene que corregir
#  lo que sobra, y las velocidades bajas -por las que SIEMPRE pasa un
#  perfil trapezoidal al arrancar y al frenar- dejan de ir a tirones.
#
#                 u0      kv        (u = u0 + kv*|v|) , por junta
# MEDIDO con arranque():  J2 subiendo 0.806, bajando 0.013
#   subir:  G + F = 0.806      ->  G = 0.397  (gravedad)
#   bajar:  G - F = -0.013     ->  F = 0.410  (rozamiento)
# El rozamiento es apenas mayor que la gravedad, por eso el brazo se
# sostiene solo pero por los pelos.
# u0 de J2 remedido el 2026-10-23 con rampa en lazo abierto, en el home
# (brazo vertical, donde la gravedad no aporta):
#     subiendo arranca con u = 0.190
#     bajando  arranca con u = 0.228
# o sea rozamiento estatico ~0.21, no 0.520.  El 0.520 venia del brazo viejo
# y metia 2.5 veces mas prealimentacion de la necesaria.
# J1 y J3 siguen pendientes de remedir igual.
# REMEDIDOS el 2026-10-28 en el mismo ensayo, como el corte u0 de la recta
# v = K(u - u0).  J2 subio porque monta el motor de 60 rpm (98:1) y J3 porque
# se tenso la correa, que antes patinaba (arrancaba a 0.102, anormalmente
# bajo; muy tensa se fue a 0.391; ajustada queda en 0.266).
ROZAMIENTO = {
    'J1': (0.637, 0.001975),
    'J2': (0.349, 0.002000),
    'J3': (0.266, 0.001636),
}

# PREALIMENTACION DE GRAVEDAD
# J1 gira sobre el eje vertical y no carga peso, asi que le basta con el
# rozamiento simetrico.  J2 (y en menor medida J3) sostienen el brazo: si
# se les aplica la prealimentacion simetrica, al pedirles BAJAR se les
# empuja hacia abajo mientras la gravedad ya tira de ellas, y se caen.
# Eso tiro el hombro de 50 a -1 grados.
#
# Por eso la prealimentacion de estas juntas es asimetrica:
#     ff = GRAVEDAD + rozamiento * signo(velocidad)
# GRAVEDAD es el PWM necesario para SOSTENER el brazo quieto, y se mide
# con sostener(junta): pidiendo velocidad cero, lo que el integrador acabe
# entregando es exactamente ese numero.
# PREALIMENTACION DE GRAVEDAD
# El par de gravedad del hombro va con el COSENO de su angulo: maximo con
# el brazo horizontal (q2 = 0) y nulo con el brazo vertical (q2 = 90).
# Por eso no sirve un numero fijo: el 0.397 se midio a 50 grados, y a 0
# grados haria falta 0.618.
#
#     G(q2) = G0 * cos(q2)
#
# J1 gira sobre el eje vertical y no carga peso.  J3 tiene su propio
# termino, pendiente de medir con arranque(2, -1).
# ------------------------------------------------------- gravedad del hombro
# PROBADO Y DESCARTADO el 2026-10-23: un modelo de dos terminos
#     tau = A*cos(th2) + B*cos(th2+th3)   con A=0.80, B=0.38
# calcula bien el par ESTATICO (0.344 N.m en el home), pero el motor no lo
# siente: el reductor 45:1 se autobloquea y absorbe la gravedad.  Prueba que
# lo demuestra: arrancar J2 en el home cuesta 0.190 subiendo y 0.228 bajando,
# casi lo mismo.  Con 0.344 N.m de gravedad real subir costaria mucho mas.
# Con los dos terminos J2 dejaba de bajar (0.0 grados/s pidiendole -25).
#
# Se vuelve al modelo de un termino, que es el que mejor numeros dio:
#     a 40 grados/s -> bajando +10 %, subiendo -39 %
GRAVEDAD = {'J1': 0.0, 'J2': 1.0, 'J3': 0.0}

# GRAVEDAD CONSTANTE, que se suma siempre a la prealimentacion (2026-11-19).
# Para J3: medido barriendo a +-30 deg/s, subiendo el mando real es ~0.68
# casi clavado entre 34 y 75 grados de antebrazo, con una prealimentacion de
# rozamiento de solo 0.32: el integrador sostenia ~0.35 todo el rato.  Y
# bajando, la prealimentacion empujaba hacia abajo (-0.32) A LA VEZ que la
# gravedad: J3 se embalaba a 2x la velocidad pedida y oscilaba.
#
# Banco, barrido de J3 a -30 deg/s (desviacion de la velocidad al BAJAR):
#     0.0 -> 13.9 14.1 18.8 14.6       0.2 -> 8.8 9.5 9.8
#     0.3 -> 15.7   0.4 -> 13.6        ganancia x1.4 -> 14.7 13.7 (no ayuda)
# 0.2 reduce la oscilacion al bajar un tercio, repetible, y no cambia nada
# al subir.  NO la elimina (el pico sigue en 1.8x).  Es un valor EMPIRICO: el
# modelo del coseno no cuadro con los datos (el mando subiendo es casi
# constante entre 34 y 75 grados de antebrazo).
GRAV_CTE = {'J1': 0.0, 'J2': 0.0, 'J3': 0.2}
FF_MIN = 2.0                 # deg/s: por debajo no se prealimenta nada

# RAMPA DEL ROZAMIENTO (2026-11-19).  Antes, por encima de FF_MIN se metia el
# rozamiento ENTERO: en J1 eso es u0 = 0.637, dos tercios del PWM, de golpe.
# Con J1 quieta mientras se mueven J2 y J3, las reacciones la sacudian unos
# grados, el PC mandaba correcciones de 2-8 deg/s, y cada vez que cambiaban
# de signo J1 recibia +-0.64 de un ciclo a otro: salia disparada, se pasaba,
# y el ciclo se sostenia solo a ~2.5 Hz.  Medido: J1 a 6-9 deg/s rms cuando
# el plan la queria QUIETA (J2 y J3 a 1-2).  Es el ciclo limite clasico de
# compensar el rozamiento con un signo.  Ahora entre FF_MIN y FF_RAMPA la
# prealimentacion sube de 0 a su valor completo, y el integrador pone el
# resto.  Solo J1: J2 y J3 se quedan como estaban, porque funcionan.
FF_RAMPA = {'J1': 12.0, 'J2': 0.0, 'J3': 0.0}   # 0 = sin rampa

# Postura de arranque: el brazo se coloca A MANO aqui y luego se
# enciende.  La anterior (0, 80, -150) dejaba a J2 a 10 grados de
# su tope y a J3 justo ENCIMA del suyo, asi que ninguna de las dos
# tenia recorrido para moverse.  Esta sigue plegada -par bajo en el
# hombro- pero con margen por todos lados.
# HOME EN L: brazo vertical, antebrazo horizontal hacia +X.
# Antes era (0, 50, -110), que no corresponde a ninguna pose de referencia.
# No se puede usar (0,0,0) como home: ahi el brazo queda estirado, sin(th3)=0
# y el jacobiano es singular.
# Tiene que coincidir con HOME_GRADOS de config.py (lado PC).
POSE_INICIAL = (0.0, 90.0, -90.0)

#             enc_A enc_B  pwm  dir1 dir2   limites
JUNTAS = (
    # nombre  encA encB  pwm  dir1 dir2   limites
    #
    # SENTIDOS VERIFICADOS EN EL ROBOT el 2026-10-23:
    #
    #  Encoders, girando cada junta 90 grados A MANO (test_encoders.comprobar):
    #     J1  horario visto desde arriba  -> -1632 cuentas   OK, es el convenio
    #     J2  bajando el brazo            -> -1636           OK, subir = positivo
    #     J3  doblando el codo adelante   -> -1631           OK (tras intercambiar
    #                                                        sus pines a 26,25)
    #
    #  Motores, con un pulso corto pidiendo u > 0:
    #     las TRES contaban negativo, o sea que giraban al reves del convenio.
    #     Por eso dir1 y dir2 van intercambiados en las tres respecto al
    #     cableado fisico: J1 (14,13), J2 (16,17), J3 (18,19).
    #
    #     El cruce que J2 tenia de antes (17,16) se DESHACE aqui: se habia
    #     puesto con la convencion de signos vieja, y ahora que el convenio es
    #     el de la guia le toca el mismo tratamiento que a las otras dos.
    ('J1', 34, 35, 27, 14, 13,  -95.0,  95.0),   # medido a mano: -101.2 / +100.5
    #                                            (lo limitan los cables, no un tope)
    # LIMITES DE J2 MEDIDOS el 2026-10-23 moviendo a mano con angulos():
    #   topes mecanicos reales: -73.8  y  +138.3 (arriba choca con la cola
    #   del motor).  Se dejan 5 grados de margen a cada lado.
    # J2 CON EL MOTOR DE 60 RPM (montado el 2026-10-28).  Respecto al motor
    # viejo hay que invertir LOS DOS signos:
    #   encoder  (32,33) -> (33,32):  con el motor nuevo, subir el brazo
    #            contaba NEGATIVO.  Comprobado bajando 45 grados: el firmware
    #            leia -45 pero el brazo subia hacia el tope del motor.
    #   motor    (16,17) -> (17,16):  si solo se invierte el encoder, el lazo
    #            empuja al lado contrario y se desboca.  Van juntos.
    # 2026-10-29: lmin -68.0 -> -71.0.  MARGEN_TOPE (5 grados) se SUMA a
    # este limite, asi que con -68 el firmware dejaba de empujar hacia
    # abajo ya en -63 y J2 se quedaba en -64.9 sin llegar al -68 que pide
    # la trayectoria de la caja.  Con -71 el freno cae en -66 y quedan
    # 2.8 grados hasta el tope mecanico medido (-73.8).
    ('J2', 33, 32,  4, 17, 16,  -71.0, 133.0),
    # J3 lmin: -123 -> -108 el 2026-11-19.  CON EL GRIPPER MONTADO, J3 choca
    # con algo en -112.6 (antebrazo 22.6 grados bajo la horizontal, con J2
    # vertical): un barrido de prueba se quedo ahi 2.6 s con el mando al
    # -100 %.  El tope mecanico de octubre (-127.6) era SIN gripper.
    # J3 lmax: 105 -> 140 el 2026-11-20, pedido por Julian: una trayectoria
    # necesita J3 a 135.  El tope de +110 de octubre era SIN gripper; con el
    # gripper montado Julian la subio a mano hasta 147.7 sin choque.  Con
    # MARGEN_TOPE = 2 el control llega hasta 138.
    ('J3', 26, 25, 21, 18, 19, -108.0, 140.0),   # medido: -127.6 / +110.0 (sin gripper)
)
# ---------------------------------------------------------------- GRIPPER (J4)
# El servo de GPIO 23 ya no existe: ese plan se descarto y en su sitio va el
# motorreductor N20 de las palas.  Aquella PWM de servo a 50 Hz sobre la 23
# era lo que empujaba el gripper contra su tope en cada arranque del panel,
# porque la 23 es ahora IN4 del L298N #2.
#
#   ENB -> GPIO 2   (PWM.  Pull-down interno durante el reset: el canal
#                    arranca DESHABILITADO por hardware, no por software)
#   IN3 -> GPIO 22   IN4 -> GPIO 23      sentido, digitales
#   C1  -> GPIO 36 (VP)   C2 -> GPIO 39 (VN)      encoder, unidad PCNT 3
#
# Medido el 2026-11-13: cero en el tope de entrada, maximo seguro en +580
# cuentas, y ~770 cuentas/s al 25 % de duty.  Pasado +580 NO hay tope: las
# palas se salen de la guia.
PIN_G_EN, PIN_G_D1, PIN_G_D2 = 2, 22, 23
PIN_G_CA, PIN_G_CB = 36, 39
G_UNIDAD = 3                 # 0, 1 y 2 las usan J1, J2 y J3
G_MIN, G_MAX = 0, 580
G_MARGEN = 70                # cuentas de respeto a cada lado
G_TOL = 25                   # cuentas: dentro de esto se considera llegado.
                             # 25 sobre 580 es un 4 %, de sobra para unas
                             # palas, y es lo mas fino que se puede afinar
                             # SIN bloquear el lazo de las juntas.
G_CERCA = 60                 # dentro de esto, zona fina (empuje/freno
                             # alternos).  Mas lejos el motor va continuo.
G_DUTY = 0.25                # el minimo que arranca
G_DUTY_MAX = 0.60            # ~6 V de los ~10 que llegan: el N20 es de 6 V
G_EMPUJON = 0.05             # +5 % por cada ciclo empujando sin moverse
G_DUTY_ARRANQUE = 0.60       # tope del empujon (= G_DUTY_MAX, los 6 V nominales
                             # del N20).  Era 0.45: recogiendo las palas de
                             # debajo de la caja se quedaron 1.7 s pegadas.
                             # Medido el 2026-11-19: cerca
                             # del cierre (45-56 cuentas) el 25 % NO arranca
                             # desde parado y el 35 % si.  Una vez en marcha
                             # el 25 % basta, asi que solo se sube para
                             # romper el rozamiento estatico.
G_CICLOS_FRENO = 1.8         # se frena si a la velocidad actual (cuentas por
                             # ciclo) no le da para parar antes del objetivo.
                             # Medido el 2026-11-19: al final de la carrera va
                             # a ~95 cuentas/ciclo (1900/s, no las 770 de los
                             # movimientos cortos) y soltando rodaba 60 mas:
                             # abriendo a 510 llego a 591, PASADO el maximo
                             # seguro de 580.
G_V_QUIETO = 5               # cuentas/ciclo: por debajo se considera parado
G_V_MIN = 15                 # cuentas/ciclo: velocidad minima util en la zona
                             # rapida.  El empujon antes solo subia el duty si
                             # estaba PARADO (<3); si se arrastraba a 4-5 por
                             # ciclo se quedaba al 25 % y en 3 s se paraba a
                             # mitad (320, 341, 458 pidiendo 510).  Abrir
                             # cuesta mas que cerrar, y depende de la postura.
G_HOME = -1000.0             # consigna especial: referenciar
G_RECORRIDO_MIN = 25         # sin esto, un tope no se acepta
G_RETIRADA = 60              # cuanto se separa antes de volver a buscar
G_DUTY_REF = 0.35            # duty para buscar el tope.  NO el 25 %: con el
                             # limitador apagando el motor, el 25 % puede no
                             # rearrancarlo en la zona pegajosa del cierre, y
                             # el detector daria un tope FALSO a media carrera.
                             # El 35 % arranca desde parado en esa zona.
G_REF_VMAX = 300             # cuentas/s maximas buscando el tope: se llega a
                             # el DESPACIO.  Antes iba a toda velocidad, hasta
                             # 1900 cuentas/s, y se estampaba.
G_REF_SIN_MOVER_MS = 600     # si al empezar a buscar no se mueve en este
                             # tiempo, ya esta en el tope (o trabado): se deja
                             # de empujar.  Antes empujaba 3 s enteros contra
                             # el tope, y el 2026-11-19 las palas acabaron
                             # trabadas despues de un referenciado de 7.4 s.

RODEO = 8.0          # grados de rodeo cuando se pide llegar por un lado
MARGEN_TOPE = 2.0            # grados antes del limite donde se frena.
                             # 2026-11-13: eran 5.0.  Con lmin -71 el permiso
                             # para bajar se cortaba en -66, y la trayectoria
                             # ensenada pedia -73.29: J2 se quedaba 6.3 grados
                             # corta el 47 % de los ciclos y el freno del plan
                             # caia a 0.44.  Con 2.0 el permiso llega a -69 y
                             # quedan 4.8 grados hasta el tope mecanico medido
                             # (-73.8).


def cos_grados(g):
    """Coseno de un angulo en grados, sin importar math en la clase."""
    return math.cos(g * 0.017453292519943295)


class Junta:
    def __init__(self, idx, nombre, pa, pb, ppwm, pd1, pd2, lmin, lmax, ini):
        self.nombre = nombre
        self.enc = Encoder(idx, Pin(pa), Pin(pb), phases=FASES)
        self.grados = GRADOS_J[idx]        # grados por cuenta DE ESTA junta
        self.pwm = PWM(Pin(ppwm), freq=PWM_FREQ, duty_u16=0)
        self.d1 = Pin(pd1, Pin.OUT, value=0)
        self.d2 = Pin(pd2, Pin.OUT, value=0)
        self.lmin, self.lmax, self.ini = lmin, lmax, ini
        self.b0, self.b1, self.p = CONTROLADORES[nombre]
        self.u0, self.kv = ROZAMIENTO[nombre]
        self.rampa = FF_RAMPA[nombre]
        self.signo_ref = 0       # sentido de la ultima consigna que movia
        self.ug = GRAVEDAD[nombre]
        self.gc = GRAV_CTE[nombre]
        self.vcontra = VEL_CONTRARIA[nombre]

        self.cuenta_ant = 0
        self.theta = ini
        self.vel = 0.0           # velocidad medida, grados/s
        self.ref = 0.0           # velocidad pedida, grados/s
        self.e1 = 0.0
        self.u1 = self.u2 = 0.0
        self.u = 0.0
        self.ff = 0.0
        self.contra = 0
        self.fuga = False

    def leer(self):
        c = self.enc.value()
        bruta = (c - self.cuenta_ant) * self.grados / TS
        self.cuenta_ant = c
        # filtro de primer orden: a baja velocidad la cuantizacion del
        # encoder (0.056 grados) mete mucho ruido en la derivada
        self.vel += FILTRO * (bruta - self.vel)
        self.theta = self.ini + c * self.grados
        return self.vel

    def parar(self):
        self.pwm.duty_u16(0)
        self.d1.value(0)
        self.d2.value(0)
        self.u = self.u1 = self.u2 = 0.0
        self.e1 = 0.0
        self.ff = 0.0

    def paso(self):
        self.leer()

        # topes por software: cerca del limite solo se deja salir de el
        ref = self.ref
        if self.theta <= self.lmin + MARGEN_TOPE and ref < 0:
            ref = 0.0
        elif self.theta >= self.lmax - MARGEN_TOPE and ref > 0:
            ref = 0.0

        # --- proteccion de embalamiento ---
        if (abs(ref) > FF_MIN and self.vel * ref < 0
                and abs(self.vel) > self.vcontra):
            self.contra += 1
        else:
            self.contra = 0
        if self.contra >= CICLOS_FUGA or abs(self.vel) > VEL_ABSURDA:
            self.fuga = True
            self.parar()
            return 0.0

        # VACIAR EL INTEGRADOR AL CAMBIAR DE SENTIDO (2026-11-19, solo juntas
        # con rampa).  Con la rampa, una correccion pequena en espera ya no
        # mueve J1, asi que el integrador se carga durante la espera.  Si el
        # siguiente movimiento va en sentido CONTRARIO, esa carga tira en
        # contra del rozamiento: medido, J1 se quedo 0.7 s clavada pidiendole
        # -30 deg/s hasta que el integrador se descargo, y luego salio
        # disparada.  Lo acumulado empujando hacia un lado no vale para el
        # otro, porque el rozamiento tambien cambia de signo.
        if self.rampa and abs(ref) > FF_MIN:
            s = 1 if ref > 0 else -1
            if self.signo_ref and s != self.signo_ref:
                self.u1 = self.u2 = 0.0
                self.e1 = 0.0
            self.signo_ref = s

        # el error entra al controlador en RAD/s (ver nota de unidades)
        e = (ref - self.vel) * 0.017453292519943295

        # INTEGRACION CONDICIONAL (2026-11-19, solo juntas con rampa).  Por
        # debajo de la rampa el rozamiento compensado no basta para despegar;
        # si el integrador acumula mientras la junta esta atascada, al cruzar
        # FF_RAMPA se suma al rozamiento completo y J1 sale disparada: medido,
        # le pedian 14 deg/s y fue a 62, y oscilo despues.  Vaciar al cambiar
        # de sentido no lo cubria, porque esto pasa arrancando hacia el MISMO
        # lado.  Ahora solo se integra si la consigna supera la rampa o la
        # junta ya se mueve; si no, la salida del controlador se mantiene.
        if (self.rampa and abs(ref) < self.rampa
                and abs(self.vel) < FF_MIN):
            u = self.u1
        else:
            # u(k) = (1+p)u(k-1) - p*u(k-2) + b0*e(k) + b1*e(k-1)
            u = ((1.0 + self.p) * self.u1 - self.p * self.u2
                 + self.b0 * e + self.b1 * self.e1)

        # guardar el estado del controlador SIN la prealimentacion, para
        # que el integrador no se pelee con ella
        if u > U_SAT:
            u = U_SAT
        elif u < -U_SAT:
            u = -U_SAT
        self.u2 = self.u1
        self.u1 = u
        self.e1 = e

        # --- prealimentacion: gravedad (con la postura) + rozamiento ---
        # la gravedad del hombro cae con el coseno de su angulo
        ff = self.ug * cos_grados(self.theta) if self.ug else 0.0
        ff += self.gc
        if abs(ref) > FF_MIN:
            roz = self.u0 + self.kv * abs(ref)
            if self.rampa and abs(ref) < self.rampa:
                roz *= (abs(ref) - FF_MIN) / (self.rampa - FF_MIN)
            ff += roz if ref > 0 else -roz
        u += ff
        self.ff = ff

        if u > U_SAT:
            u = U_SAT
        elif u < -U_SAT:
            u = -U_SAT
        self.u = u

        self.d1.value(1 if u >= 0 else 0)
        self.d2.value(0 if u >= 0 else 1)
        d = int(abs(u) * 65535)
        self.pwm.duty_u16(d if d < 65535 else 65535)
        return e


class Gripper:
    """Las palas, en control de POSICION por cuentas de encoder.

    No lleva lazo de velocidad como las juntas: la carrera entera son 580
    cuentas y se recorre en menos de un segundo.
    """

    def __init__(self):
        # el enable primero y a cero: hasta que no este sujeto, nada se mueve
        self.en = PWM(Pin(PIN_G_EN), freq=PWM_FREQ, duty_u16=0)
        self.d1 = Pin(PIN_G_D1, Pin.OUT, value=0)
        self.d2 = Pin(PIN_G_D2, Pin.OUT, value=0)
        self.enc = Encoder(G_UNIDAD, Pin(PIN_G_CA), Pin(PIN_G_CB),
                           phases=FASES)
        self.base = self.enc.value()
        self.referenciado = False
        self.fase = 0                  # 0 parado, 1 empujando, 2 frenando
        self.atascos = 0               # ciclos seguidos empujando sin moverse
        self.ref = self.cuentas()      # arranca donde este: no se mueve
        self.pos_ant = self.ref

    # ------------------------------------------------------------- motor
    def mover(self, duty, sentido):
        if duty > G_DUTY_MAX:
            duty = G_DUTY_MAX
        if duty < 0:
            duty = 0
        if sentido >= 0:
            self.d1.value(1)
            self.d2.value(0)
        else:
            self.d1.value(0)
            self.d2.value(1)
        self.en.duty_u16(int(duty * 65535))

    def parar(self):
        self.en.duty_u16(0)

    def frenar_sin_esperar(self):
        """Pone el freno y vuelve.  Se queda frenando hasta que otro ciclo
        diga lo contrario: asi no hay que dormir dentro del lazo."""
        self.d1.value(1)
        self.d2.value(1)
        self.en.duty_u16(65535)

    def frenar(self, seg=0.05):
        """Freno CON espera.  Solo para referenciar(), que de todas formas
        bloquea: dentro del lazo de control no se puede dormir."""
        self.frenar_sin_esperar()
        time.sleep(seg)
        self.parar()
        self.d1.value(0)
        self.d2.value(0)

    def cuentas(self):
        return self.enc.value() - self.base

    def cero(self):
        self.base = self.enc.value()
        self.referenciado = True

    # ------------------------------------------------------------- control
    def paso(self):
        """Un ciclo de control.  NO BLOQUEA: ni un time.sleep aqui dentro.

        El lazo de las juntas calcula con TS = 50 ms fijo; cualquier espera
        aqui cambia el periodo real y les multiplica las ganancias (medido: el
        periodo se iba de 49 a 99 ms y J2 dejaba de responder).

        Orden de decision, de mas a menos urgente:
          1. llegado: si aun rueda, FRENO (antes soltaba y rodaba 60 cuentas)
          2. limites de software
          3. FRENO PREDICTIVO: si a la velocidad actual no le da para parar
             antes del objetivo, frena ya
          4. lejos: motor continuo
          5. cerca: alterna un ciclo empujando y otro frenando
        En todos los empujes se aplica el empujon de arranque.
        """
        pos = self.cuentas()
        v = pos - self.pos_ant                 # cuentas por ciclo
        self.pos_ant = pos
        e = self.ref - pos
        # empujon: en la zona rapida se exige G_V_MIN; en la fina, solo no
        # estar parado.  Sube de 5 en 5 y BAJA de uno en uno, para no volver
        # a caer al 25 % de golpe y atascarse otra vez.
        lento = abs(v) < (G_V_MIN if abs(e) > G_CERCA else 3)
        if self.fase == 1 and lento:
            self.atascos += 1
        elif not lento and self.atascos > 0:
            self.atascos -= 1
        duty = G_DUTY + G_EMPUJON * self.atascos
        if duty > G_DUTY_ARRANQUE:
            duty = G_DUTY_ARRANQUE
            self.atascos = int((G_DUTY_ARRANQUE - G_DUTY) / G_EMPUJON)
        s = 1 if e > 0 else -1

        # 1. llegado
        if abs(e) <= G_TOL:
            if abs(v) > G_V_QUIETO:
                self.frenar_sin_esperar()
                self.fase = 2
            else:
                self.parar()
                self.fase = 0
                self.atascos = 0
            return

        # 2. limites de software
        if self.referenciado:
            if (pos <= G_MIN and s < 0) or (pos >= G_MAX and s > 0):
                self.frenar_sin_esperar()
                self.fase = 2
                return

        # 3. freno predictivo
        if v * e > 0 and abs(e) <= abs(v) * G_CICLOS_FRENO + G_TOL:
            self.frenar_sin_esperar()
            self.fase = 2
            return

        # 4. lejos: continuo
        if abs(e) > G_CERCA:
            self.mover(duty, s)
            self.fase = 1
            return

        # 5. cerca: empuje y freno alternos
        if self.fase == 1:
            self.frenar_sin_esperar()
            self.fase = 2
        else:
            self.mover(duty, s)
            self.fase = 1

    def consigna(self, destino):
        """Fija el objetivo, recortado a los limites.  SIN REFERENCIA SE IGNORA.

        2026-11-24: antes, sin referenciar, obedecia cualquier destino
        contado desde donde estuviera al arrancar.  Tras un reinicio con las
        palas fuera (en 494), el 70 que manda el panel nada mas abrirse las
        habria sacado hasta 564, y pasado 580 se salen de la guia.
        """
        if not self.referenciado:
            return
        if self.referenciado:
            lo, hi = G_MIN + G_MARGEN, G_MAX - G_MARGEN
            if destino < lo:
                destino = lo
            elif destino > hi:
                destino = hi
        self.ref = destino

    # --------------------------------------------------------- referenciar
    def referenciar(self, duty=G_DUTY_REF, limite_t=4.0, retirar=True):
        """Busca el tope del lado de ENTRADA y pone ahi el cero.

        Solo hacia dentro: por el lado de salida no hay tope y las palas se
        saldrian de la guia.  Si no se mueve puede ser que ya este en el tope
        — son indistinguibles desde el encoder — asi que se separa y vuelve a
        buscar.  Esta funcion SI bloquea, un segundo y pico, y por eso el
        firmware pone las juntas a cero antes de llamarla.
        """
        if self._buscar(-1, duty, limite_t):
            return True
        # 2026-11-24: retirar=False (consigna -1001) NO las saca.  Tras un
        # reinicio la cuenta vale 0 esten donde esten; si estaban fuera y el
        # motor no tira, sacarlas 60 en cada intento las lleva a 580.
        if not retirar:
            self.ref = self.cuentas()   # quieto donde quedo
            return False
        c0 = self.cuentas()
        self.mover(duty, 1)
        t0 = time.ticks_ms()
        while (time.ticks_diff(time.ticks_ms(), t0) < 400
               and abs(self.cuentas() - c0) < G_RETIRADA):
            time.sleep_ms(2)
        self.frenar()
        time.sleep_ms(300)
        if self._buscar(-1, duty, limite_t):
            return True
        self.ref = self.cuentas()       # fallo: quieto donde quedo
        return False

    def _buscar(self, sentido, duty, limite_t):
        """Empuja hacia el tope, DESPACIO, hasta que se para contra el.

        Devuelve False sin insistir si al principio no se mueve: o ya esta en
        el tope o algo lo bloquea, y en los dos casos seguir empujando solo
        sirve para clavarlo.
        """
        c0 = self.cuentas()
        t0 = time.ticks_ms()
        self.mover(duty, sentido)
        encendido = True
        ultima = c0
        tv = t0
        c_vel = c0
        t_vel = t0
        tope = False
        while True:
            ahora = time.ticks_ms()
            transcurrido = time.ticks_diff(ahora, t0)
            if transcurrido > limite_t * 1000:
                break
            c = self.cuentas()
            # no arranca: ya esta en el tope o esta trabado.  No insistir.
            if (transcurrido > G_REF_SIN_MOVER_MS
                    and abs(c - c0) <= G_RECORRIDO_MIN):
                break
            # limitador de velocidad: apaga si va rapido, enciende si no
            dt = time.ticks_diff(ahora, t_vel)
            if dt >= 20:
                vel = abs(c - c_vel) * 1000 // dt
                c_vel = c
                t_vel = ahora
                if vel > G_REF_VMAX and encendido:
                    self.parar()
                    encendido = False
                elif vel <= G_REF_VMAX and not encendido:
                    self.mover(duty, sentido)
                    encendido = True
            # tope: dejo de avanzar despues de haber recorrido algo
            if time.ticks_diff(ahora, tv) >= 120:
                if abs(c - ultima) < 3 and abs(c - c0) > G_RECORRIDO_MIN:
                    tope = True
                    break
                ultima = c
                tv = ahora
            time.sleep_ms(5)
        self.frenar()
        time.sleep_ms(200)
        if not tope:
            return False
        self.cero()
        self.ref = 0
        return True


ROBOT = None


class Robot:
    def __init__(self):
        global ROBOT
        self.j = [Junta(i, *JUNTAS[i], ini=POSE_INICIAL[i]) for i in range(3)]
        ROBOT = self
        self.habilitado = False
        self.ultima = time.ticks_ms()
        try:
            self.g = Gripper()
        except Exception as x:
            print('# sin gripper:', x)
            self.g = None

    def parar(self):
        self.habilitado = False
        for j in self.j:
            j.parar()
        if self.g:
            self.g.parar()

    def consigna(self, v):
        """v1,v2,v3[,ignorado[,g_ref]].

        El cuarto campo era el servo y ya no se usa: se ignora para que los
        programas de trayectoria sigan mandando su trama sin cambios.  El
        gripper es el QUINTO, en cuentas, y con G_HOME pide referenciar.
        """
        for k in range(3):
            self.j[k].ref = v[k]
        if len(v) > 4 and self.g:
            if v[4] <= G_HOME:
                for j in self.j:          # el brazo quieto mientras busca
                    j.ref = 0.0
                    j.parar()
                ok = self.g.referenciar(retirar=v[4] > G_HOME - 0.5)
                print('# gripper referenciado' if ok
                      else '# gripper NO pudo referenciar')
            else:
                self.g.consigna(v[4])
        self.ultima = time.ticks_ms()
        self.habilitado = True

    def estado(self):
        return [(j.theta, j.vel) for j in self.j]

    def cuentas_gripper(self):
        return self.g.cuentas() if self.g else 0

    def referenciar_gripper(self):
        """Atajo para llamarlo a mano desde el REPL.  Por el serie se pide
        mandando G_HOME en el quinto campo de la trama."""
        return self.g.referenciar() if self.g else False


def run(verboso=True):
    """Bucle principal. Trama: v1,v2,v3[,ignorado[,g_ref]].

    Las tres primeras en grados/s de junta; g_ref en cuentas del
    gripper, o G_HOME para referenciarlo.
    """
    r = Robot()
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ''
    print('# control de VELOCIDAD listo')
    print('# trama: v1,v2,v3[,-,g_ref]  juntas en grados/s, gripper en cuentas')
    print('# sin consigna durante %d ms -> paro' % TIMEOUT_MS)
    sig = time.ticks_add(time.ticks_ms(), TS_MS)
    try:
        while True:
            while poll.poll(0):
                c = sys.stdin.read(1)
                if c in ('\n', '\r'):
                    if buf:
                        try:
                            v = [float(x) for x in buf.split(',')]
                            if len(v) >= 3:
                                r.consigna(v)
                        except ValueError:
                            pass
                        buf = ''
                elif len(buf) < 60:
                    buf += c

            if r.habilitado and time.ticks_diff(time.ticks_ms(),
                                                r.ultima) > TIMEOUT_MS:
                r.parar()

            if r.habilitado:
                for j in r.j:
                    j.paso()
                if r.g:
                    r.g.paso()
            else:
                for j in r.j:
                    j.leer()
                if r.g:
                    r.g.parar()

            if verboso:
                # el septimo campo son las cuentas del gripper.  Los indices
                # pares e impares que usa leer_telemetria siguen cuadrando.
                print('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d'
                      % (r.j[0].theta, r.j[0].vel,
                         r.j[1].theta, r.j[1].vel,
                         r.j[2].theta, r.j[2].vel,
                         r.cuentas_gripper()))

            f = time.ticks_diff(sig, time.ticks_ms())
            if f > 0:
                time.sleep_ms(f)
            sig = time.ticks_add(sig, TS_MS)
    except KeyboardInterrupt:
        pass
    finally:
        r.parar()
        print('# parado')
    return r


def probar(junta=0, vel=40.0, segundos=4):
    """Hace girar UNA junta a velocidad constante.

        import control_vel; control_vel.probar(0, 40)

    Mira si la velocidad medida sigue a la pedida.  Con el lazo de
    velocidad el rozamiento se vence solo: el integrador sube hasta que
    la junta arranca.
    """
    r = Robot()
    v = [0.0, 0.0, 0.0]
    v[junta] = vel
    print('%s a %.1f grados/s durante %.1f s'
          % (r.j[junta].nombre, vel, segundos))
    print('%8s %9s %9s %8s %8s' % ('t', 'pedida', 'medida', 'u', 'pos'))
    r.consigna(v)
    t0 = time.ticks_ms()
    hist = []
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < segundos * 1000:
            culpable = None
            for j in r.j:
                j.paso()
                if j.fuga and culpable is None:
                    culpable = j
            if culpable is not None:
                r.parar()
                print()
                print('### PARADA DE SEGURIDAD ###')
                print('%s se movia en SENTIDO CONTRARIO al pedido.'
                      % culpable.nombre)
                print('Su motor esta invertido respecto del encoder:')
                print('intercambia dir1 y dir2 de esa junta en JUNTAS.')
                break
            jj = r.j[junta]
            hist.append(jj.vel)
            print('%8.2f %9.1f %9.1f %8.3f %8.2f'
                  % (time.ticks_diff(time.ticks_ms(), t0) / 1000.0,
                     jj.ref, jj.vel, jj.u, jj.theta))
            time.sleep_ms(TS_MS)
    except KeyboardInterrupt:
        pass
    finally:
        r.parar()
        if len(hist) > 10:
            est = hist[len(hist) // 2:]
            media = sum(est) / len(est)
            print()
            print('  velocidad media en regimen: %.1f grados/s (pedida %.1f)'
                  % (media, vel))
            print('  error de velocidad        : %.1f %%'
                  % ((media - vel) / vel * 100 if vel else 0))


def volver_a(r, junta, destino, vel=35.0, limite=8.0, tol=0.5, pasadas=3,
             lado=0):
    """Lleva la junta a una posicion, por control de velocidad.

    tol      error en grados con el que se da por llegada.  Antes era 2.0
             fijo, y de ahi venia que el brazo quedase torcido al volver al
             home: se conformaba con 2 grados.
    pasadas  tras parar, el brazo puede quedarse corto o pasarse por el
             rozamiento.  Se vuelve a intentar hasta 'pasadas' veces,
             cada vez mas despacio.
    lado     0  llega por donde salga
            +1  llega SIEMPRE subiendo:  se pasa 'RODEO' grados por debajo
                y sube hasta el destino
            -1  llega siempre bajando
             El stick-slip hace que el error final sea de hasta 1.3 grados y
             no se puede bajar de ahi, pero llegando siempre por el mismo
             lado ese error es un SESGO CONSTANTE en vez de aleatorio, y un
             sesgo constante se compensa.  Para una referencia, repetible
             importa mas que pequeno.
    """
    j = r.j[junta]
    if lado:
        # primero rodear: ir a un punto 'RODEO' grados al lado contrario
        previo = destino - lado * RODEO
        lo, hi = j.lmin + MARGEN_TOPE, j.lmax - MARGEN_TOPE
        previo = min(max(previo, lo), hi)
        if abs(previo - j.theta) > tol:
            volver_a(r, junta, previo, vel=vel, limite=limite, tol=max(tol, 1.0),
                     pasadas=1, lado=0)
    for intento in range(pasadas):
        # cada pasada es mas lenta y mas fina que la anterior
        v_max = vel if intento == 0 else max(6.0, vel / (2 ** intento))
        v_min = 6.0 if intento == 0 else 3.0
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < limite * 1000:
            d = destino - j.theta
            if abs(d) < tol:
                break
            c = [0.0, 0.0, 0.0]
            # proporcional, saturado entre v_min y v_max
            v = abs(d) * 1.5
            if v > v_max: v = v_max
            if v < v_min: v = v_min
            c[junta] = v if d > 0 else -v
            r.consigna(c)
            for jj in r.j:
                jj.paso()
            time.sleep_ms(TS_MS)
        # parar y dejar que asiente antes de juzgar
        r.consigna([0.0, 0.0, 0.0])
        for jj in r.j:
            jj.parar()
        for _ in range(10):
            for jj in r.j:
                jj.leer()
            time.sleep_ms(TS_MS)
        if abs(destino - j.theta) < tol:
            break
    return j.theta


def barrido(junta=0, velocidades=(10, 20, 40, 60, 80, 120), t_cada=2.5,
            sentido=0):
    """Mide que PWM hace falta para cada velocidad, y hasta donde llega.

        import control_vel; control_vel.barrido(0)

    sentido:  0  alterna (vale para J1, que no carga peso)
             +1  siempre en sentido positivo
             -1  siempre en sentido negativo

    EN J2 HAY QUE MEDIR LOS DOS SENTIDOS POR SEPARADO: la gravedad ayuda
    al bajar y estorba al subir, asi que alternando saldria un promedio
    sin sentido.  Usa barrido(1, sentido=+1) y barrido(1, sentido=-1).

    Sirve ademas para saber la velocidad maxima REAL de la junta, que es
    lo que limita los tiempos de la trayectoria.
    """
    r = Robot()
    inicio = r.j[junta].theta
    lmin, lmax = r.j[junta].lmin, r.j[junta].lmax
    if inicio <= lmin + MARGEN_TOPE + 1 or inicio >= lmax - MARGEN_TOPE - 1:
        print('AVISO: %s arranca en %.1f grados, pegada a su tope [%.0f, %.0f].'
              % (r.j[junta].nombre, inicio, lmin, lmax))
        print('       Colocala a mano hacia el centro del recorrido, apaga y')
        print('       enciende la ESP32, y repite. Si no, no podra moverse.')
        return []
    print('arranca en %.1f grados; vuelve ahi entre medida y medida' % inicio)
    print('%10s %10s %8s %8s' % ('pedida', 'medida', 'u medio', 'alcanza'))
    print('-' * 40)
    resultados = []
    try:
        for v in velocidades:
            if sentido:
                signo = 1.0 if sentido > 0 else -1.0
            else:
                # alternar para no agotar el recorrido de la junta
                signo = 1.0 if len(resultados) % 2 == 0 else -1.0

            # El tiempo de medida se calcula desde el RECORRIDO DISPONIBLE.
            # Antes era fijo, y a 40 deg/s durante 2.5 s pedia 100 grados de
            # giro: J2 solo tiene 35 hacia arriba, asi que chocaba con el
            # tope, la proteccion ponia la consigna a cero, y lo que se
            # medeia era el robot parado.
            margen = ((lmax - MARGEN_TOPE) - inicio if signo > 0
                      else inicio - (lmin + MARGEN_TOPE))
            t_util = min(t_cada, 0.75 * margen / v) if v > 0 else 0.0
            if t_util < 0.8:
                print('%10.1f %10s %8s %8s   no cabe: solo %.0f grados libres'
                      % (v, '-', '-', '-', margen))
                continue

            c = [0.0, 0.0, 0.0]
            c[junta] = v * signo
            r.consigna(c)
            vs, us = [], []
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < t_util * 1000:
                for j in r.j:
                    j.paso()
                if time.ticks_diff(time.ticks_ms(), t0) > t_util * 400:
                    vs.append(abs(r.j[junta].vel))
                    us.append(abs(r.j[junta].u))
                time.sleep_ms(TS_MS)
            vm = sum(vs) / len(vs) if vs else 0.0
            um = sum(us) / len(us) if us else 0.0
            ok = 'si' if vm > v * 0.9 else 'NO'
            print('%10.1f %10.1f %8.3f %8s   %.1f s, %.0f->%.0f grados'
                  % (v, vm, um, ok, t_util, inicio, r.j[junta].theta))
            resultados.append((v, vm, um))
            # VOLVER al punto de partida: si no, la junta deriva hasta el
            # tope y el resto de medidas salen falseadas
            volver_a(r, junta, inicio)
    except KeyboardInterrupt:
        pass
    finally:
        r.parar()
    alcanzadas = [v for v, vm, um in resultados if vm > v * 0.9]
    if alcanzadas:
        print()
        print('  velocidad maxima fiable de %s: ~%.0f grados/s%s'
              % (r.j[junta].nombre, max(alcanzadas),
                 '' if not sentido else
                 ('  (sentido %+d)' % (1 if sentido > 0 else -1))))
        us = [um for v, vm, um in resultados if vm > v * 0.9]
        vs = [v for v, vm, um in resultados if vm > v * 0.9]
        if len(vs) >= 2:
            kv = (us[-1] - us[0]) / (vs[-1] - vs[0])
            u0 = us[0] - kv * vs[0]
            print('  rozamiento ajustado: u0 = %.3f   kv = %.6f' % (u0, kv))
            print('  -> ponlo en ROZAMIENTO[%r]' % r.j[junta].nombre)
        print('  (la trayectoria del ciclo pide hasta 133 grados/s en J1)')
    return resultados


def sostener(junta=1, segundos=6):
    """Mide el PWM necesario para SOSTENER la junta quieta.

        import control_vel; control_vel.sostener(1)

    Se pide velocidad cero y se deja que el integrador encuentre el valor.
    Lo que acabe entregando es la prealimentacion de gravedad de esa junta
    en esa postura: ponlo en GRAVEDAD[...].

    Ojo: depende de la POSTURA.  Con el brazo estirado el hombro pide
    mucho mas que plegado.  Mide en la postura donde vaya a trabajar, o
    en varias y quedate con la peor.
    """
    r = Robot()
    j = r.j[junta]
    ug = j.ug
    j.ug = 0.0                       # se anula para medirla de cero
    print('%s en %.1f grados: midiendo lo que cuesta sostenerla'
          % (j.nombre, j.theta))
    print('%8s %9s %9s' % ('t', 'u', 'pos'))
    r.consigna([0.0, 0.0, 0.0])
    us = []
    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < segundos * 1000:
            for jj in r.j:
                jj.paso()
            t = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
            if t > segundos * 0.5:
                us.append(j.u)
            print('%8.2f %9.3f %9.2f' % (t, j.u, j.theta))
            time.sleep_ms(TS_MS)
    except KeyboardInterrupt:
        pass
    finally:
        r.parar()
        j.ug = ug
    if us:
        m = sum(us) / len(us)
        print()
        print('  PWM para sostener: %.3f' % m)
        print('  -> ponlo en GRAVEDAD[%r]' % j.nombre)
        if abs(m) < 0.05:
            print('  (casi cero: esta junta no carga peso en esta postura)')
        return m


def arranque(junta=1, sentido=1, umax=0.95, rampa=0.25, umbral=3.0):
    """Mide el PWM al que la junta EMPIEZA a moverse (rozamiento estatico).

        import control_vel; control_vel.arranque(1, +1)   # J2 subiendo
        control_vel.arranque(1, -1)                       # J2 bajando

    Sube la salida en rampa, en LAZO ABIERTO, hasta que detecta movimiento,
    y corta.  Es la forma honesta de medir el rozamiento: sostener() no
    servia porque la junta se queda agarrada por el propio rozamiento y lo
    que mide es cuanto habia acumulado el integrador, no lo que cuesta
    moverla.

    rampa: cuanto sube la salida por segundo
    umbral: grados/s a partir de los cuales se considera que arranco
    """
    r = Robot()
    j = r.j[junta]
    p0 = j.theta
    print('%s en %.1f grados, rampa %s a %.2f por segundo'
          % (j.nombre, p0, '+' if sentido > 0 else '-', rampa))
    u = 0.0
    arranco = None
    t0 = time.ticks_ms()
    try:
        while u < umax:
            t = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
            u = rampa * t
            j.leer()
            s_ = 1 if sentido > 0 else -1
            j.d1.value(1 if s_ > 0 else 0)
            j.d2.value(0 if s_ > 0 else 1)
            d = int(min(u, umax) * 65535)
            j.pwm.duty_u16(d)
            if abs(j.vel) > umbral and arranco is None:
                arranco = u
                print('  ARRANCA con u = %.3f  (a los %.2f s)' % (u, t))
                break
            if t > 1.0 and abs(j.theta - p0) > 25.0:
                print('  se movio 25 grados, paro por seguridad')
                break
            print('  u %.3f   vel %6.1f   pos %7.2f' % (u, j.vel, j.theta))
            time.sleep_ms(TS_MS)
    except KeyboardInterrupt:
        pass
    finally:
        r.parar()
    if arranco is None:
        print('  no arranco hasta u = %.2f' % umax)
    else:
        print()
        print('  rozamiento de arranque de %s sentido %+d: %.3f'
              % (j.nombre, 1 if sentido > 0 else -1, arranco))
    return arranco


def angulos(segundos=60):
    """Muestra los tres angulos EN VIVO, con los motores apagados.

        import control_vel; control_vel.angulos()

    Sirve para explorar a mano hasta donde llega cada junta sin arriesgarse a
    empujar contra un tope con el motor.  Al final dice el recorrido minimo y
    maximo que se ha alcanzado en cada una.
    """
    r = Robot()
    for x in r.j:
        x.parar()
    print('motores APAGADOS.  mueve las juntas a mano.  Ctrl-C para salir.')
    print('home = (0, 90, -90)')
    print('  t[s] |      J1       J2       J3   |  minimos y maximos')
    mn = [999.0] * 3
    mx = [-999.0] * 3
    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < segundos * 1000:
            for x in r.j:
                x.leer()
            for i in range(3):
                v = r.j[i].theta
                if v < mn[i]: mn[i] = v
                if v > mx[i]: mx[i] = v
            print('  %4.1f | %8.2f %8.2f %8.2f   | J2 %+.1f a %+.1f'
                  % (time.ticks_diff(time.ticks_ms(), t0) / 1000.0,
                     r.j[0].theta, r.j[1].theta, r.j[2].theta, mn[1], mx[1]))
            time.sleep_ms(400)
    except KeyboardInterrupt:
        pass
    for x in r.j:
        x.parar()
    print()
    for i, n in enumerate(('J1', 'J2', 'J3')):
        print('  %s: de %+8.2f a %+8.2f grados' % (n, mn[i], mx[i]))


def medir_reduccion(junta=1, duty=0.45, sentido=1, referencia='vuelta',
                    tope=30000, segundos=90):
    """Mide la REDUCCION del motor girandolo con su propio PWM.

        import control_vel
        control_vel.medir_reduccion(1, 0.45, +1)            # motor suelto
        control_vel.medir_reduccion(1, 0.45, +1, 'grados90')  # montado con correa

    Para el motor de 60 rpm, cuyo eje no se puede girar a mano.

    referencia = 'vuelta'    el eje de SALIDA da una vuelta completa (motor
                             suelto, sin correa).  cuentas = 48 * reduccion
    referencia = 'grados90'  la JUNTA gira 90 grados (motor montado, con la
                             correa 3:1).  cuentas = 48 * reduccion * 3 / 4

    COMO SE USA: marca el eje con cinta, lanza esto, y Ctrl-C EXACTAMENTE al
    completar la referencia.

    OJO: esto es LAZO ABIERTO, los limites del firmware no protegen.  Hay
    tope de cuentas y de tiempo, pero ten la mano en el interruptor.
    """
    j = Junta(junta, *JUNTAS[junta], ini=POSE_INICIAL[junta])
    j.enc.value(0)
    time.sleep_ms(300)
    por_vuelta = referencia != 'grados90'
    f = 48.0 if por_vuelta else 36.0      # cuentas por unidad de reduccion
    print('%s  duty %.0f %%  sentido %+d  referencia: %s'
          % (j.nombre, duty * 100, sentido,
             'UNA VUELTA del eje de salida' if por_vuelta
             else '90 GRADOS de la junta'))
    print('Ctrl-C justo al completarla.   tope %d cuentas' % tope)
    print()
    print('   t[s]   cuentas   reduccion que implica')
    j.d1.value(1 if sentido > 0 else 0)
    j.d2.value(0 if sentido > 0 else 1)
    j.pwm.duty_u16(int(duty * 65535))
    c = 0
    t0 = time.ticks_ms()
    try:
        while True:
            c = j.enc.value()
            t = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
            if abs(c) > tope or t > segundos:
                print('  cortado por tope')
                break
            print('  %5.1f %9d        %6.1f : 1' % (t, c, abs(c) / f))
            time.sleep_ms(300)
    except KeyboardInterrupt:
        pass
    j.parar()
    red = abs(c) / f
    print()
    print('CUENTAS: %d   ->   REDUCCION = %.1f : 1' % (abs(c), red))
    print()
    print('comparacion con las candidatas:')
    for r in (45, 90, 98, 100, 120):
        print('   %3d:1  ->  %6d cuentas %s'
              % (r, int(r * f), '   <-- la mas cercana'
                 if abs(r - red) == min(abs(x - red) for x in (45,90,98,100,120))
                 else ''))
    print()
    print('con la correa 3:1 montada, 90 grados de junta serian %d cuentas'
          % int(round(red) * 36))
    return abs(c)


# ===========================================================================
#  REFERENCIADO CONTRA EL TOPE: PROBADO Y DESCARTADO (2026-10-28)
#
#  Se intento que J2 se referenciara sola llevandola contra su tope mecanico,
#  porque con el motor de 60 rpm (98:1) ya no se puede colocar a mano: al
#  forzar la junta PATINA LA CORREA.
#
#  NO FUNCIONA, y el motivo es estructural, no de ajuste:
#
#      motor --[ENCODER AQUI]-- caja 98:1 --[polea 20]--correa--[polea 60]-- junta
#
#  El encoder esta en el eje del MOTOR, antes de la correa.  Cuando la junta
#  topa, el motor sigue girando patinando la correa, asi que el encoder nunca
#  ve el atasco: el tope es invisible para el firmware por construccion.
#
#  Se probaron tres detectores y los tres fallaron por lo mismo:
#    1. "deja de contar"            -> saltaba en el arranque lento (a baja
#       velocidad J2 tarda en romper su rozamiento) y daba una correccion
#       falsa de 48 grados.
#    2. idem + limite software ampliado -> saltaba con el frenado del propio
#       limite, 13 grados antes del tope real, y corregia +10 de mas.
#    3. "no cuenta Y el mando esta saturado" -> ya no salta NUNCA, porque el
#       encoder sigue contando mientras la correa patina.  El brazo estuvo
#       70 grados "buscando" con la correa patinando contra el tope.
#
#  LO QUE SI FUNCIONARIA: un final de carrera en la JUNTA (un microinterruptor
#  y dos cables a un GPIO libre).  Detecta la posicion de la junta, no la del
#  motor, que es justo lo que falta.  Mientras tanto, el home se coloca a mano
#  como siempre, antes de encender.
# ===========================================================================
