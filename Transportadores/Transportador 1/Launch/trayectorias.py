#!/usr/bin/env python3
# Puntos de la trayectoria de demostracion y su interpolacion.
import math

D = math.radians

PUNTOS = [
    (   0,  90,  -90),
    (   0,  90,  -70),
    (  60,  90,  -70),
    (  60,  80,  -90),
    (  60,  65, -105),
    (  60,  80,  -90),
    ( -60,  90,  -70),
    ( -60,  65, -105),
    ( -60,  80,  -90),
    (   0,  90,  -90),
]

PUNTOS_RAD = [tuple(D(a) for a in p) for p in PUNTOS]


def quintica(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def interpolar(i, t, periodo, puntos=PUNTOS_RAD):
    a = puntos[i % len(puntos)]
    b = puntos[(i + 1) % len(puntos)]
    s = quintica(t / periodo if periodo > 0 else 1.0)
    return [ai + (bi - ai) * s for ai, bi in zip(a, b)]
