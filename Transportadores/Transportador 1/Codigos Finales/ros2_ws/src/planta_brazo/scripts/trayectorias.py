#!/usr/bin/env python3
"""
Trayectorias articulares del robot PLANTA BRAZO.

Modulo compartido: lo usan tanto gui_articulaciones.py (boton "Trayectoria")
como trayectoria_demo.py, para que ambos recorran exactamente los mismos puntos.
"""
import math

D = math.radians

# (J1, J2, J3) en grados
PUNTOS = [
    # HOME EN L (0, 90, -90).  El home anterior era (0, 0, 0), o sea el brazo
    # totalmente estirado: ahi sin(theta3) = 0 y el jacobiano es SINGULAR, no
    # se puede invertir.  Por eso el home es en L y no estirado.
    (   0,  90,  -90),   # home en L
    (   0,  90,  -70),   # levantar
    (  60,  90,  -70),   # girar a la zona de recogida
    (  60,  80,  -90),   # aproximar
    (  60,  65, -105),   # bajar a recoger
    (  60,  80,  -90),   # subir
    ( -60,  90,  -70),   # girar a la zona de descarga
    ( -60,  65, -105),   # bajar a soltar
    ( -60,  80,  -90),   # subir
    (   0,  90,  -90),   # volver al home
]

PUNTOS_RAD = [tuple(D(a) for a in p) for p in PUNTOS]


def quintica(s):
    """Perfil suave 10s^3 - 15s^4 + 6s^5, con velocidad y aceleracion nulas
    en los extremos.  s en [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def interpolar(i, t, periodo, puntos=PUNTOS_RAD):
    """Posicion articular (rad) en el tramo i, tiempo t dentro del tramo."""
    a = puntos[i % len(puntos)]
    b = puntos[(i + 1) % len(puntos)]
    s = quintica(t / periodo if periodo > 0 else 1.0)
    return [ai + (bi - ai) * s for ai, bi in zip(a, b)]
