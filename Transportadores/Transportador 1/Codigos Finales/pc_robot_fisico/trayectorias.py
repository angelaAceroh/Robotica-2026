#!/usr/bin/env python3
"""
Trayectorias del robot transportador, en coordenadas del ROBOT FISICO.

Los puntos de los informes anteriores estaban pensados para el modelo de
Gazebo, cuyo alcance (139 a 432 mm) no es el del robot real (45 a 255 mm
desde el hombro).  Aqui se definen dentro del alcance verdadero.
El ciclo empieza y acaba en la POSTURA DE REPOSO (brazo plegado), no en
(0,0,0).  Estirado en horizontal es la postura de MAYOR par del hombro
-el 200 % del par nominal del motor-, asi que dejar ahi el robot parado
lo hace calentar sin necesidad.  Plegado baja al 45 %.
"""
import math

import numpy as np

import cinematica as cin
import config as cfg


# ------------------------------------------------------------- ciclo 3R
# Ciclo de transporte simetrico, en GRADOS de junta.
# Mismo gesto que el de los informes: sube, gira, baja, recoge, gira al
# otro lado, suelta y vuelve.  El eje de simetria es el giro central.
CICLO = [
    # Mismos puntos que el control cinematico de Gazebo, verificados contra
    # los topes mecanicos medidos el 2026-10-23 (J1 +-95, J2 -68..133,
    # J3 -123..105).  Se pasa por el HOME al cambiar de zona porque la recta
    # directa de +Y a -Y pasa cerca del eje de la base y ahi el codo pediria
    # -138 grados, por debajo del tope fisico de -127.6.
    ('P0  home',              (   0.0,  90.00,  -90.00)),
    ('P1  zona +Y arriba',    (  30.96,  94.70, -115.77)),
    ('P2  bajar a recoger',   (  30.96,  70.51, -115.77)),
    ('P3  subir',             (  30.96,  94.70, -115.77)),
    ('P4  pasar por el home', (   0.0,   90.00,  -90.00)),
    ('P5  zona -Y',           ( -41.99, 109.21, -119.49)),
    ('P6  volver al home',    (   0.0,   90.00,  -90.00)),
]


# ===========================================================================
#  CICLO_CAJA  -  PRETRAYECTORIA, ensenada a mano el 2026-10-28
#
#  NO es la definitiva.  Julian la enseno con panel_juntas.py para ver el
#  comportamiento; faltan el gripper (palas extensibles con su propio
#  motorreductor) y las medidas reales de donde esta la caja.
#
#  La tarea real sera de DOS viajes: llevar la caja de huevos vacia al
#  ensamblador, retirarse, y cuando este cargada con los tres bombones
#  recogerla y llevarla al destino.
#
#  Las palas son theta2 + theta3 (no hay muneca): 0 = horizontales.
#
#  AVISO: en la postura de transporte el brazo VACIO ya pide 0.66 N.m y el
#  motor da 0.65, asi que con carga se pasa.  Cada 40 g de caja suman 0.15.
#  Lo que mas ayudaria es acercar la caja al eje: a 321 mm en vez de 381 el
#  par baja a la mitad.
#
#  J2 venia a -68.4 en algun punto y el limite esta en -68.0; esos puntos van
#  recortados al limite.  El tope mecanico esta en -73.8, asi que si hace
#  falta ese trozo se puede bajar el limite a -71.
# ===========================================================================
CICLO_CAJA = [
    ('C0  home',                  (  90.78,  90.00, -90.00)),
    ('C1  bajar a la caja',       (  90.78, -68.00,  59.94)),
    ('C2  levantar inclinando',   (  90.78, -64.11,  72.72)),
    ('C3  girar al ensamblador',  (  38.22, -64.11,  72.72)),
    ('C4  bajar a dejarla',       (  38.00, -68.00,  59.39)),
    ('C5  retirar las palas',     (  38.00, -68.00, 100.22)),
    ('C6  volver a bajar',        (  38.00, -68.00,  60.44)),
    ('C7  recoger cargada',       (  38.00, -57.78,  72.89)),
    ('C8  girar al destino',      ( -51.33, -67.86,  56.72)),
    ('C9  volver al home',        (  -0.22,  90.28, -89.00)),
]


#  CICLO_ENSENADO  -  ensenado a mano con panel_juntas.py el 2026-11-13
#  Siete puntos, con el gripper ya montado.  La apertura de las palas se
#  guardo tambien (lista APERTURA_ENSENADO), pero en esta toma salio ~0 en
#  todos los puntos: las palas estuvieron cerradas todo el rato.
#
#  AVISO: P1, P3 y P4 piden 0.89, 0.88 y 0.88 N.m en J2, por encima de los
#  0.65 que da el motor.  Son posturas muy estiradas con las palas arriba.
CICLO_ENSENADO_A = [
    ('P0', (  90.44,   90.00,  -90.00)),
    ('P1', (  90.28,  -14.54,   72.44)),
    ('P2', (  37.67,  -15.43,   88.44)),
    ('P3', (  37.67,  -15.43,   73.44)),
    ('P4', ( -38.39,  -15.43,   73.94)),
    ('P5', (   0.28,   91.71,  -90.00)),
    ('P6', (   0.28,   91.71,  -90.00)),
]

APERTURA_ENSENADO_A = {
    'P0': 0, 'P1': 9, 'P2': 9, 'P3': 9, 'P4': 8, 'P5': 8, 'P6': 8,
}


#  CICLO_ENSENADO  -  segunda toma, 2026-11-13.  El par ya esta bien (0.55
#  N.m maximo), pero P1..P5 piden J2 a -73.29, que esta 7 grados POR DEBAJO
#  de donde el firmware deja de empujar (-66 = lmin -71 + MARGEN_TOPE 5) y
#  por debajo del propio limite del panel (-68).  El tope mecanico medido es
#  -73.8, asi que esos puntos estan practicamente contra el tope.
#  J2 recortada de -73.29 a -68.00 el 2026-11-13.  Con MARGEN_TOPE bajado a
#  2.0 el firmware permite bajar hasta -69, asi que -68 es alcanzable y deja
#  1 grado de permiso.  Los -73.29 originales estaban a medio grado del tope
#  mecanico (-73.8) y J2 se quedaba 6.3 grados corta el 47 % del tiempo.
CICLO_ENSENADO_B = [
    ('P0', (  86.67,   90.00,  -90.00)),
    ('P1', (  89.17,  -68.00,   73.44)),
    ('P2', (  29.89,  -68.00,   87.94)),
    ('P3', (  29.89,  -68.00,   69.28)),
    ('P4', ( -48.33,  -68.00,   89.11)),
    ('P5', ( -48.33,  -68.00,   70.78)),
    ('P6', (  -3.22,   90.77,  -90.11)),
]

#  APERTURA DE LAS PALAS en cada punto, en CUENTAS del encoder del N20.
#  70 = cerradas (palas dentro), 510 = abiertas (palas fuera).
#  La orden se manda en el tramo que LLEVA a ese punto, asi que las palas
#  llegan ya colocadas.  Un punto que no aparezca aqui conserva la apertura
#  del punto anterior.
#
#  Los puntos donde J3 esta en su minimo son los de descenso -- P1 (73.44),
#  P3 (69.28) y P5 (70.78) -- y son los que mueven las palas.
APERTURA_POR_PUNTO_B = {
    0: 70,       # P0  home, palas dentro
    1: 510,      # P1  abajo: las palas SALEN bajo la caja
    2: 510,      # P2  sube y transporta con la caja encima
    3: 70,       # P3  abajo: las palas ENTRAN y la caja se queda
    4: 70,       # P4  sube vacia
    5: 510,      # P5  abajo: las palas SALEN otra vez
    6: 70,       # P6  vuelve a home con las palas dentro
}

#  Apertura tal como se ensenó (salio ~0 en todos: las palas no se movieron)
APERTURA_ENSENADO_B = {
    'P0': 0, 'P1': 1, 'P2': 1, 'P3': 1, 'P4': 1, 'P5': 1, 'P6': 1,
}


#  CICLO_ENSENADO  -  tercera toma, 2026-11-19, con panel_juntas.py.
#  P4 y P5 son el mismo punto: el plan lo convierte en una parada de 0.4 s.
#  J2 en -67.96 .. -68.09: dentro del permiso del firmware (-69 con
#  MARGEN_TOPE 2.0).  Par en J2 0.61 N.m en los puntos bajos, cerca del 0.65.
CICLO_ENSENADO_C = [
    ('P0', (  92.28,  -67.96,   59.50)),
    ('P1', (  18.00,  -67.96,   59.50)),
    ('P2', (  18.06,  -68.04,   74.22)),
    ('P3', ( -34.44,  -68.04,   74.22)),
    ('P4', ( -34.44,  -68.09,   59.28)),
    ('P5', ( -34.44,  -68.09,   59.28)),
    ('P6', (   0.44,   90.10,  -90.83)),
]

#  Apertura medida en cada punto al ensenar (cuentas del N20), TAL CUAL:
APERTURA_ENSENADO_C = {
    'P0': 52, 'P1': 52, 'P2': 48, 'P3': 48, 'P4': 48, 'P5': 48, 'P6': 48,
}


def _apertura_de_lo_ensenado(ensenado, nombres):
    """Convierte lo medido al ensenar en ordenes para las palas.

    Lo que se graba es la posicion MEDIDA, con unas cuentas de ruido: 52 y 48
    son la misma posicion.  Si se mandase tal cual, cada diferencia de ruido
    pararia el brazo para "mover" las palas 4 cuentas.  Asi que se ajusta a
    cerrado / abierto cuando esta cerca, y solo se manda cuando CAMBIA.
    """
    out = {}
    for k, n in enumerate(nombres):
        a = ensenado[n]
        if a < 150:
            a = 70          # cerrado
        elif a > 420:
            a = 510         # abierto
        out[k] = a
    return out


#  Lo que usa mover_robot_vel.py: indice de punto -> apertura en cuentas.
#  Se aplica AL LLEGAR a cada punto, y solo con el brazo ya en su sitio.
#  Pedido por Julian el 2026-11-19: extender en P1 una vez que J2 este
#  ubicado, mantenerlas fuera, y recogerlas en P5, el punto anterior al home.
#  (Lo ensenado con J4 salio cerrado en todos: ver APERTURA_ENSENADO.)
#  Cambiado el 2026-11-19 a peticion de Julian: salen en P0 (J1 en 92, ANTES
#  de girar), y el giro de P0 a P1 las mete por debajo de la caja de lado.
#  OJO: P0 y P1 tienen la misma altura e inclinacion (punta a 27 mm, -8.5
#  grados): si las palas chocan con la mesa al salir, aqui tambien.
APERTURA_POR_PUNTO_C = {
    0: 510,      # P0   al llegar: EXTENDER, antes de girar
    1: 510,      # P1   fuera: el giro las ha metido bajo la caja
    2: 510,      # P2   fuera, levantando
    3: 510,      # P3   fuera
    4: 510,      # P4   fuera
    5: 70,       # P5   al llegar: RECOGER, antes de volver al home
    6: 70,       # P6   home, cerradas
}


#  CICLO_ENSENADO  -  cuarta toma, 2026-11-20, con panel_juntas.py.
#  P0 = P1 (abajo, junto a la caja: el plan hace una parada de 0.4 s) y
#  P5 = P6 (abajo, donde la deja).  Palas horizontales en la recogida.
#
#  P7 se ENSENO con J3 a 147.72 (subida a mano: el panel no pasa de 105) y el
#  firmware no lleva J3 mas alla de 103 (lmax 105 - MARGEN_TOPE 2): se queda
#  clavada con 45 grados de error.  Recortado a 103.  En octubre se midio el
#  tope mecanico de J3 en +110 (sin gripper): antes de subir el limite hay que
#  comprobar que llega sin chocar.
#  Par en J2 0.62-0.64 N.m en los puntos bajos, al borde del 0.65.
CICLO_ENSENADO = [
    ('P0', (  94.72,  -65.87,   66.56)),
    ('P1', (  94.72,  -65.87,   66.56)),
    ('P2', (  94.44,  -65.87,   82.94)),
    ('P3', (  28.00,  -65.89,   66.00)),
    ('P4', (  28.00,  -65.92,   83.28)),
    ('P5', ( -29.67,  -65.92,   58.94)),
    ('P6', ( -29.67,  -65.92,   58.94)),
    ('P7', ( -29.67,  -65.92,  103.00)),     # ensenado 147.72, recortado
    ('P8', (   2.00,   93.47,  -86.00)),
]

#  Apertura medida al ensenar: 182 en todos (las palas no se movieron con J4)
APERTURA_ENSENADO = {
    'P0': 182, 'P1': 182, 'P2': 182, 'P3': 182, 'P4': 182, 'P5': 182,
    'P6': 182, 'P7': -1, 'P8': -1,
}

#  Pedido por Julian: como la toma anterior.  Se extienden al llegar a P1, con
#  J2 ya colocada, y se recogen en el punto donde deja la caja (P6), antes de
#  levantar el antebrazo: si siguieran fuera en P7 levantarian otra vez la
#  caja que se acaba de dejar.
APERTURA_POR_PUNTO = {
    0: 70,       # P0   cerradas
    1: 510,      # P1   al llegar: EXTENDER (mismo sitio que P0)
    2: 510,      # P2   sube con la caja
    3: 510,      # P3
    4: 510,      # P4
    5: 510,      # P5   baja a dejarla
    6: 70,       # P6   al llegar: RECOGER (mismo sitio que P5)
    7: 70,       # P7   levanta el antebrazo
    8: 70,       # P8   home
}


#  CICLO_ENSENADO_E  -  quinta toma, 2026-11-20.  NO ACTIVA: en el robot J3 se
#  quedo 21 grados corta en P0/P1 y en P5 las palas no pudieron salir (algo las
#  bloqueaba; la punta queda por debajo del plano de la base).  Julian pidio
#  volver a la cuarta toma, la que cogio la caja dentro de los limites.
#  Cargada TAL CUAL, por decision de Julian ("asi como le ensene punto por
#  punto no sucede nada").  Avisado antes de que tiene valores fuera de lo que
#  el brazo puede hacer, y que los topes del firmware lo frenaran:
#     P10  J2 -90.66  (tope mecanico -73.8; el firmware no baja de -69)
#     P0,P1  J3 -128.17  (con el gripper J3 choca en -112.6; firmware -106)
#  Probable causa: J2/J3 movidas a mano, la correa patina y el encoder (que va
#  antes de la correa) pierde la cuenta.  Para esta toma se subio el limite
#  superior de J3 de 105 a 140 (P10 pide 132.89).
CICLO_ENSENADO_E = [
    ('P0',  (  86.72,   74.41, -128.17)),
    ('P1',  (  86.72,   25.82, -128.17)),
    ('P2',  (  86.72,   25.79,  -98.39)),
    ('P3',  (  86.72,   -4.72,  -64.56)),
    ('P4',  (  86.72,  -64.95,   46.89)),
    ('P5',  (  86.72,  -64.95,   46.89)),
    ('P6',  (  86.72,  -64.95,   62.89)),
    ('P7',  (  27.44,  -64.95,   46.72)),
    ('P8',  ( -29.33,  -64.95,   54.06)),
    ('P9',  ( -29.33,  -64.95,   53.89)),
    ('P10', ( -29.33,  -90.66,  132.89)),
    ('P11', (   0.00,   91.89,  -90.61)),
]

#  Apertura medida al ensenar (sin referenciar? 797 pasa del maximo seguro 580)
APERTURA_ENSENADO_E = {
    'P0': 135, 'P1': 137, 'P2': 155, 'P3': 155, 'P4': 136, 'P5': 797,
    'P6': 797, 'P7': 797, 'P8': 797, 'P9': 132, 'P10': 132, 'P11': 135,
}

#  Lo ensenado con J4: fuera de P5 a P8, dentro el resto.  P5 repite P4 y P9
#  repite P8: ahi actuan.  Se abren a 510 (no a 797: pasado 580 se salen).
APERTURA_POR_PUNTO_E = {
    0: 70, 1: 70, 2: 70, 3: 70, 4: 70,
    5: 510,      # P5   al llegar: EXTENDER (mismo sitio que P4)
    6: 510, 7: 510, 8: 510,
    9: 70,       # P9   al llegar: RECOGER (mismo sitio que P8)
    10: 70, 11: 70,
}


def ciclo_ensenado_juntas():
    """Los puntos de CICLO_ENSENADO, en radianes."""
    q = [np.radians(p) for _, p in CICLO_ENSENADO]
    return q, [n for n, _ in CICLO_ENSENADO]


def ciclo_caja_juntas():
    """Igual que ciclo_juntas() pero con la pretrayectoria de la caja."""
    q = [np.radians(p) for _, p in CICLO_CAJA]
    return q, [n for n, _ in CICLO_CAJA]


# Donde abre y cierra el gripper, por indice de punto de paso
SERVO_POR_PUNTO = {4: cfg.SERVO_CERRADO, 9: cfg.SERVO_ABIERTO}


def ciclo_juntas():
    """Puntos de paso del ciclo, en radianes."""
    return [np.radians(q) for _, q in CICLO], [n for n, _ in CICLO]


def recta_cartesiana(A, B, n=12, codo='arriba'):
    """Subdivide el segmento A-B y resuelve la cinematica inversa.

    A, B en metros.  Devuelve los puntos de paso en radianes.
    Lanza ValueError nombrando el punto que no alcanza, en vez de dejar
    que salgan nan como pasaba antes.
    """
    hitos, nombres = [], []
    q_prev = np.zeros(3)
    for k in range(n):
        s = k / (n - 1.0)
        p = tuple(a + (b - a) * s for a, b in zip(A, B))
        try:
            q = cin.inversa(p, codo) if k == 0 else cin.inversa_cerca(p, q_prev)
        except ValueError as e:
            raise ValueError('punto %d/%d (%.1f, %.1f, %.1f) mm: %s'
                             % (k + 1, n, p[0] * 1000, p[1] * 1000,
                                p[2] * 1000, e))
        fuera = cin.en_limites(np.degrees(q))
        if fuera:
            raise ValueError('punto %d/%d: %s' % (k + 1, n, '; '.join(fuera)))
        q_prev = q
        hitos.append(q)
        nombres.append('R%d' % (k + 1))
    return hitos, nombres


def recta_por_defecto(n=12):
    """Una recta horizontal delante del robot, dentro del alcance real."""
    # recta corta cerca del home, dentro del espacio util
    A = (0.300, -0.080, 0.330)
    B = (0.300,  0.080, 0.330)
    return recta_cartesiana(A, B, n)


if __name__ == '__main__':
    print(cfg.resumen())
    print('\n--- CICLO DE TRANSPORTE ---')
    hitos, nombres = ciclo_juntas()
    print('%-22s %24s %26s' % ('punto', 'q1 q2 q3 [deg]', 'TCP x y z [mm]'))
    for n, q in zip(nombres, hitos):
        p = cin.directa(q)
        print('%-22s %8.1f%8.1f%8.1f %9.1f%9.1f%9.1f'
              % (n, *np.degrees(q), *(p * 1000)))

    print('\n--- RECTA A-B ---')
    try:
        hitos, _ = recta_por_defecto()
        for k, q in enumerate(hitos):
            p = cin.directa(q)
            print('  R%-3d %8.1f%8.1f%8.1f deg   -> %8.1f%8.1f%8.1f mm'
                  % (k + 1, *np.degrees(q), *(p * 1000)))
    except ValueError as e:
        print('  NO VIABLE:', e)
