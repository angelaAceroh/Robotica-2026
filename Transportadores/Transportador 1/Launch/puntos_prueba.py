#!/usr/bin/env python3
# Puntos de prueba del informe.
import math

PUNTOS_PRUEBA = [
    ('P1  home extendido',      (   0.0,   0.0,    0.0)),
    ('P2  frente arriba',       (   0.0,  40.0,  -70.0)),
    ('P3  derecha media altura',(  45.0,  25.0,  -50.0)),
    ('P4  izquierda alta',      ( -60.0,  55.0,  -95.0)),
    ('P5  atras baja',          ( 150.0, -10.0,  -30.0)),
    ('P6  lateral a 90',        (  90.0,  30.0,  -90.0)),
    ('P7  recogido',            (  30.0,  70.0, -140.0)),
    ('P8  codo abajo',          ( -30.0, -25.0,   65.0)),
]

PUNTOS_PRUEBA_RAD = [(n, tuple(math.radians(a) for a in q))
                     for n, q in PUNTOS_PRUEBA]
