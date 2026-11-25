#!/usr/bin/env python3
"""
CONFIGURACION UNICA del robot transportador 3R - montaje FISICO.

Todo lo que hay que tocar al cambiar algo del robot esta aqui y solo aqui.
Los demas archivos importan de este; si un numero aparece dos veces en el
proyecto, es un error.
"""

# =====================================================================
#  GEOMETRIA   (metros)  -  MEDIDAS REALES DEL ROBOT
#
#  Fuente unica: correciones/robot_antropomorfo_3gdl_referencia.md
#  (2026-10-22).  Tienen que coincidir con el paquete de ROS:
#  ros2_ws/src/planta_brazo/scripts/cinematica.py y el .urdf.xacro.
#
#  Lo que habia antes salia de Fisico/ControlCinematico.py y estaba mal:
#      L1 0.19   -> 0.200     medido desde la base cuadrada
#      A1 0.06   -> 0.05815   coincide con el CAD
#      A2 0.15   -> 0.145     medido
#      A3 0.105  -> 0.26285   el 0.105 era SOLO del gripper viejo; la
#                             longitud del ultimo eslabon es L3 + L4
# =====================================================================
L1 = 0.200     # suelo -> eje del hombro
A1 = 0.05815   # offset horizontal eje de base -> eje del hombro
L3 = 0.16285   # codo -> brida           (del CAD, pendiente de medir)
L4 = 0.100     # brida -> punta          (PROVISIONAL, gripper sin disenar)
A2 = 0.145     # hombro -> codo
A3 = L3 + L4   # codo -> punta del efector  (CAMBIA CON EL GRIPPER)

# Home: brazo vertical y antebrazo horizontal hacia +X.  NO se puede usar
# (0,0,0) como home: ahi sin(theta3)=0 y el jacobiano es singular.
HOME_GRADOS = (0.0, 90.0, -90.0)

# =====================================================================
#  TRANSMISION
#  Cada junta lleva motorreductor + una segunda reduccion por correa
#  dentada 200-2GT con polea motriz de 20 dientes y conducida de 60.
#  El informe lo confirma: "relacion de transmision de 3:1".
# =====================================================================
POLEA_MOTRIZ = 20          # dientes
POLEA_CONDUCIDA = 60       # dientes
N_CORREA = POLEA_CONDUCIDA / POLEA_MOTRIZ      # 3.0

# Cuantos grados gira el MOTOR por cada grado de la JUNTA.
# Es el numero que faltaba en todo el codigo anterior.
GRADOS_MOTOR_POR_JUNTA = N_CORREA

# =====================================================================
#  MOTORES   TS-25GAH370-45 / Bemonoc  (12 V, caja 45:1, 130 rpm)
#  Datos de la ficha de la tienda y de la hoja TSINY que vino en la caja.
# =====================================================================
MOTOR_RPM = 130.0                       # rpm del eje de salida de la caja
MOTOR_POTENCIA = 2.5                    # W
MOTOR_I_VACIO = 0.15                    # A

# Par NOMINAL: la ficha dice 2.6 lb.cm.
#   2.6 lb.cm x 0.45359 = 1.179 kgf.cm x 9.80665 / 100 = 0.1157 N.m
# (Antes se venia usando 0.200 N.m, un valor tipico de catalogo: el real
#  es el 58 % de aquel, asi que el margen de par es MENOR del que creiamos.)
MOTOR_PAR_NOMINAL = 0.1157              # N.m en el eje de salida de la caja
# El par de BLOQUEO no viene en la ficha. Para estos motorreductores suele
# estar entre 2.5 y 3 veces el nominal; se deja declarado como estimacion
# y hay que medirlo o pedirlo antes de dar por cerrado el dimensionamiento.
MOTOR_PAR_BLOQUEO_EST = 2.75 * MOTOR_PAR_NOMINAL
ETA_CORREA = 0.95

# Par disponible EN LA JUNTA, despues de la correa
PAR_JUNTA_NOMINAL = MOTOR_PAR_NOMINAL * N_CORREA * ETA_CORREA
PAR_JUNTA_BLOQUEO_EST = MOTOR_PAR_BLOQUEO_EST * N_CORREA * ETA_CORREA
JUNTA_RPM = MOTOR_RPM / N_CORREA        # 43.3 rpm en la junta
JUNTA_VEL_MAX = JUNTA_RPM * 360.0 / 60.0    # 260 deg/s

# Margen de seguridad: no pedir nunca mas del 60 % de la velocidad tope.
VEL_MAX = 0.60 * JUNTA_VEL_MAX          # 156 deg/s
ACEL_MAX = 150.0                        # deg/s2, valor de diseno

# =====================================================================
#  ENCODER   TSINY-8370, Hall de dos canales en cuadratura
#
#  La ficha lo dice sin lugar a dudas:
#      "12 PPR x 45 = 540 PPR"
#  o sea 12 pulsos por vuelta del EJE DEL MOTOR, y como la caja es 45:1,
#  540 pulsos por vuelta del eje de SALIDA.
#
#  El firmware anterior tenia PPR = 7100: se equivocaba por un factor de
#  13.1, asi que creia que la junta se habia movido 13 veces menos de lo
#  que realmente se movio.
#
#  DECODIFICACION: contando los dos canales y los dos flancos (cuadratura
#  completa, x4) la resolucion sube a 0.056 grados por cuenta en la junta.
#  Con x1 (solo flanco de subida de A, como estaba) serian 0.222 grados.
# =====================================================================
PPR_BASE = 540.0                        # pulsos/vuelta de salida, por canal
DECODIFICACION = 4                      # 1, 2 o 4; el firmware usa 4
PPR_MOTOR = PPR_BASE * DECODIFICACION   # cuentas por vuelta del eje de salida
PPR_JUNTA = PPR_MOTOR * N_CORREA        # cuentas por vuelta de la JUNTA
RESOLUCION = 360.0 / PPR_JUNTA          # grados de junta por cuenta

# J2 monta desde el 2026-10-28 el motor de 60 rpm, cuya caja es 98:1 y no
# 45:1, asi que da mas del doble de cuentas por el mismo angulo.  La reduccion
# se midio girando el eje de salida tres vueltas con el propio motor: 13963
# cuentas = 97,0:1, que es 98:1 con un 1 % de margen por el tiempo de reaccion.
# Tiene que coincidir con POR_VUELTA de micropython/control_vel.py.
REDUCCION = (45, 98, 45)                        # caja de cada junta
PPR_JUNTA_J = tuple(12 * r * DECODIFICACION * int(N_CORREA) for r in REDUCCION)
RESOLUCION_J = tuple(360.0 / v for v in PPR_JUNTA_J)

# =====================================================================
#  LIMITES ARTICULARES  (grados)
# =====================================================================
# El HOME es theta2 = +90, asi que el tope superior de J2 NO puede ser 90:
# el home caeria justo en el limite y el control no podria pasar de ahi.
# La trayectoria de la guia llega a ~127 grados en J2.
# PENDIENTE: confirmar en el robot fisico que J2 llega a 135 sin chocar.
LIMITES = {
    'J1': (-95.0, 95.0),    # medido -101.2/+100.5 (lo limitan los cables)
    'J2': (-68.0, 133.0),   # topes reales -73.8 / +138.3, con 5 de margen
    # 2026-11-20: igual que el firmware.  Arriba 140 (pedido por Julian, una
    # trayectoria necesita 135; a mano llego a 147.7 sin choque).  Abajo -108:
    # con el gripper J3 CHOCA en -112.6 (el -127.6 de octubre era sin gripper).
    'J3': (-108.0, 140.0),
}

# =====================================================================
#  COMUNICACION CON LA ESP32
# =====================================================================
BAUDIOS = 115200
PERIODO = 0.05            # s entre consignas; DEBE coincidir con el Ts del
                          # firmware (control_posicion.ino)

# Puertos donde buscar la ESP32. None = autodetectar.
PUERTO = None
CANDIDATOS = ('/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1',
              'COM3', 'COM4', 'COM5')

# =====================================================================
#  POSTURA DE REPOSO / REFERENCIA DE ARRANQUE
#
#  Los encoders son INCREMENTALES: cuentan cuanto se ha movido el eje,
#  no en que angulo esta.  Al encender, la ESP32 no tiene ni idea de
#  donde esta el brazo: supone que las cuentas valen cero.
#
#  Por eso hay un ritual obligatorio: ANTES de dar corriente, se coloca
#  el brazo A MANO en esta postura.  El firmware arranca creyendo que
#  esta aqui, y a partir de ahi lleva la cuenta correctamente.
#
#  Se eligio el brazo PLEGADO y no el estirado (0,0,0) porque estirado
#  es justo la postura de mayor par en el hombro: 200 % del nominal.
#  Plegado baja al 45 %, asi que el motor no sufre mientras espera.
# =====================================================================
# Tiene que coincidir con POSE_INICIAL de micropython/control_vel.py
# y con el primer punto del ciclo (P0) de trayectorias.py.
# La anterior (0, 80, -150) dejaba a J2 a 10 grados de su tope y a
# J3 justo encima del suyo: ninguna tenia recorrido para moverse.
POSE_INICIAL = HOME_GRADOS      # grados de junta J1, J2, J3

# =====================================================================
#  EFECTOR
# =====================================================================
SERVO_ABIERTO = 150       # grados
SERVO_CERRADO = 80
SERVO_NEUTRO = 90            # resto del plan del servo, que se descarto.
                             # El firmware ignora ese campo de la trama.

# ------------------------------------------------------------------ GRIPPER
# Palas con motorreductor N20 y encoder, medidas el 2026-11-13 sobre el robot:
# el cero es el tope del lado de entrada y el maximo seguro son 580 cuentas.
# Pasado ese punto NO hay tope y las palas se salen de la guia.
GRIPPER_MIN = 0
GRIPPER_MAX = 580
GRIPPER_MARGEN = 70         # el firmware recorta a MIN+margen .. MAX-margen
GRIPPER_CERRADO = 70
GRIPPER_ABIERTO = 510
GRIPPER_HOME = -1000        # consigna especial: referenciar contra el tope
GRIPPER_HOME_SIN_RETIRADA = -1001   # igual, pero SIN sacarlas si no encuentra el tope


def resumen():
    return '\n'.join([
        'Robot transportador 3R - configuracion fisica',
        '  geometria   L1=%.3f  a1=%.3f  a2=%.3f  a3=%.3f m' % (L1, A1, A2, A3),
        '  alcance desde el hombro: %.3f a %.3f m'
        % (abs(A2 - A3), A2 + A3),
        '  correa      %d/%d dientes  ->  %.1f:1' % (POLEA_CONDUCIDA,
                                                     POLEA_MOTRIZ, N_CORREA),
        '  motor       %.0f rpm  ->  junta %.1f rpm (%.0f deg/s)'
        % (MOTOR_RPM, JUNTA_RPM, JUNTA_VEL_MAX),
        '  diseno      vel %.0f deg/s   acel %.0f deg/s2' % (VEL_MAX, ACEL_MAX),
        '  encoder     %.0f x%d = %.0f cuentas/vuelta de salida, %.0f por '
        'vuelta de junta (%.4f deg/cuenta)'
        % (PPR_BASE, DECODIFICACION, PPR_MOTOR, PPR_JUNTA, RESOLUCION),
        '  par motor   %.4f N.m nominal  ->  junta %.3f N.m nominal, '
        '%.3f de bloqueo (estimado)'
        % (MOTOR_PAR_NOMINAL, PAR_JUNTA_NOMINAL, PAR_JUNTA_BLOQUEO_EST),
    ])


if __name__ == '__main__':
    print(resumen())
