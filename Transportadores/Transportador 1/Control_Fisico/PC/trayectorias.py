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
    ('P0  reposo',           (   0,  50, -110)),
    ('P1  levantar',         (   0,  55,  -95)),
    ('P2  girar a recogida', (  60,  55,  -95)),
    ('P3  aproximar',        (  60,  28,  -72)),
    ('P4  recoger',          (  60,  10,  -45)),
    ('P5  retirar',          (  60,  28,  -72)),
    ('P6  elevar',           (  60,  55,  -95)),
    ('P7  girar a descarga', ( -60,  55,  -95)),
    ('P8  aproximar',        ( -60,  28,  -72)),
    ('P9  soltar',           ( -60,  10,  -45)),
    ('P10 retirar',          ( -60,  28,  -72)),
    ('P11 elevar',           ( -60,  55,  -95)),
    ('P12 volver al frente', (   0,  55,  -95)),
    ('P13 reposo',           (   0,  50, -110)),
]

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
    A = (0.160, -0.070, 0.230)
    B = (0.160,  0.070, 0.230)
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
