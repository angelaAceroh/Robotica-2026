#!/usr/bin/env python3
"""
Puntos de prueba para comparar la cinematica DIRECTA con la INVERSA.

Cada punto se define por una terna articular de referencia q_ref (grados).
El objetivo cartesiano es entonces P_obj = CD(q_ref), un punto que por
construccion EXISTE y es alcanzable, asi que las dos vias se pueden comparar
sin discutir si el punto era alcanzable o no:

    via DIRECTA   se le mandan al robot los angulos q_ref y se mira a donde
                  llego de verdad:  P_dir = CD(q_medido)
    via INVERSA   se le da al robot solo el punto P_obj, la CI calcula unos
                  angulos q_ci y se mira a donde llego:  P_inv = CD(q_medido)

Los siete primeros puntos tienen q3 <= 0 (codo arriba), que es la postura que
elige la CI por defecto, asi que deberia recuperar q_ref clavado.  El octavo
tiene el codo abajo a proposito: sirve para ver que un mismo punto tiene mas de
una solucion articular y que la CI escoge la de la postura pedida.
"""
import math

# nombre, (J1, J2, J3) en grados
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
