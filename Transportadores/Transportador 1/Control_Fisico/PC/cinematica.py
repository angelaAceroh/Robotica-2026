#!/usr/bin/env python3
"""
CINEMATICA del robot transportador 3R - version rapida para el robot fisico.

Misma convencion Denavit-Hartenberg que ControlCinematico.py, pero con dos
diferencias que importan:

  1) Las matrices se COMPILAN una sola vez con sympy.lambdify.  El codigo
     anterior hacia T_sym.subs(...) dentro del lazo de control, a 25 Hz:
     sustituir una matriz simbolica 4x4 mas el jacobiano tarda del orden de
     decenas de milisegundos, asi que el lazo NO llegaba a tiempo real.
     Compilado tarda microsegundos.

  2) La inversa devuelve las DOS soluciones de codo (arriba y abajo) y avisa
     de los casos degenerados, en vez de quedarse con una rama fija.  Las dos
     versiones del codigo anterior usaban ramas distintas:
         ControlCinematico.py   theta2 = alpha + beta   theta3 = -pi + gamma
         final.py               theta2 = alpha - beta   theta3 =  pi - gamma
     Son las dos ramas del mismo problema; aqui estan las dos y se elige.
"""
import math

import numpy as np
import sympy as sp

import config as cfg


# ---------------------------------------------------------------- simbolico
def _construir():
    th1, th2, th3, L1, a1, a2, a3 = sp.symbols('th1 th2 th3 L1 a1 a2 a3',
                                               real=True)

    A01 = sp.Matrix([
        [sp.cos(th1), 0,  sp.sin(th1), a1 * sp.cos(th1)],
        [sp.sin(th1), 0, -sp.cos(th1), a1 * sp.sin(th1)],
        [0,           1,  0,           L1],
        [0,           0,  0,           1]])

    A12 = sp.Matrix([
        [sp.cos(th2), -sp.sin(th2), 0, a2 * sp.cos(th2)],
        [sp.sin(th2),  sp.cos(th2), 0, a2 * sp.sin(th2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    A23 = sp.Matrix([
        [sp.cos(th3), -sp.sin(th3), 0, a3 * sp.cos(th3)],
        [sp.sin(th3),  sp.cos(th3), 0, a3 * sp.sin(th3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    T = sp.simplify(A01 * A12 * A23)
    pos = T[0:3, 3]
    J = pos.jacobian([th1, th2, th3])
    args = (th1, th2, th3, L1, a1, a2, a3)
    return (sp.lambdify(args, pos, 'numpy'),
            sp.lambdify(args, J, 'numpy'))


_POS, _JAC = _construir()
_GEO = (cfg.L1, cfg.A1, cfg.A2, cfg.A3)


# ------------------------------------------------------------------ directa
def directa(q):
    """q en radianes (3,) -> posicion (x, y, z) en metros."""
    p = _POS(q[0], q[1], q[2], *_GEO)
    return np.asarray(p, dtype=float).reshape(3)


def jacobiano(q):
    """Jacobiano de velocidad lineal, 3x3."""
    return np.asarray(_JAC(q[0], q[1], q[2], *_GEO), dtype=float)


# ------------------------------------------------------------------ inversa
def inversa(p, codo='arriba'):
    """Punto (x, y, z) en metros -> (q1, q2, q3) en radianes.

    codo: 'arriba' | 'abajo' | 'cercana' (necesita q_actual via inversa_cerca)

    Lanza ValueError si el punto no es alcanzable, en vez de devolver nan
    silenciosamente como hacia el codigo anterior.
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    L1, a1, a2, a3 = _GEO

    th1 = math.atan2(y, x)

    rxy = math.hypot(x, y)
    u = rxy - a1                 # alcance horizontal desde el hombro
    v = z - L1                   # altura relativa al hombro
    R = math.hypot(u, v)

    if R > a2 + a3 + 1e-9:
        raise ValueError('punto fuera del alcance: R = %.4f m > %.4f m'
                         % (R, a2 + a3))
    if R < abs(a2 - a3) - 1e-9:
        raise ValueError('punto demasiado cerca del hombro: R = %.4f m < %.4f m'
                         % (R, abs(a2 - a3)))

    alpha = math.atan2(v, u)
    cb = (R * R + a2 * a2 - a3 * a3) / (2.0 * a2 * R)
    cg = (a2 * a2 + a3 * a3 - R * R) / (2.0 * a2 * a3)
    beta = math.acos(max(-1.0, min(1.0, cb)))
    gamma = math.acos(max(-1.0, min(1.0, cg)))

    if codo == 'abajo':
        th2 = alpha - beta
        th3 = math.pi - gamma
    else:                                   # codo arriba
        th2 = alpha + beta
        th3 = -math.pi + gamma

    return np.array([th1, th2, th3], dtype=float)


def inversa_cerca(p, q_actual):
    """Elige entre codo arriba y abajo la que menos mueva las juntas."""
    mejor, dmin = None, float('inf')
    for codo in ('arriba', 'abajo'):
        try:
            q = inversa(p, codo)
        except ValueError:
            continue
        d = float(np.sum(np.abs(q - np.asarray(q_actual))))
        if d < dmin:
            mejor, dmin = q, d
    if mejor is None:
        raise ValueError('ninguna solucion alcanzable para %s' % (p,))
    return mejor


def en_limites(q_deg):
    """Devuelve la lista de juntas fuera de rango (vacia si todo bien)."""
    fuera = []
    for v, (n, (lo, hi)) in zip(q_deg, cfg.LIMITES.items()):
        if not (lo <= v <= hi):
            fuera.append('%s = %.1f fuera de [%.0f, %.0f]' % (n, v, lo, hi))
    return fuera


# ------------------------------------------------------------------ pruebas
if __name__ == '__main__':
    import random
    import time

    print(cfg.resumen())
    print('\nComprobacion directa -> inversa -> directa, 5000 posturas')
    peor, fallos = 0.0, 0
    for _ in range(5000):
        q = np.array([random.uniform(-2.0, 2.0),
                      random.uniform(-1.4, 1.4),
                      random.uniform(-2.5, 2.5)])
        p = directa(q)
        try:
            q2 = inversa_cerca(p, q)
        except ValueError:
            fallos += 1
            continue
        peor = max(peor, float(np.linalg.norm(directa(q2) - p)))
    print('  sin solucion : %d' % fallos)
    print('  peor cierre  : %.6f mm' % (peor * 1000))

    t0 = time.perf_counter()
    for _ in range(2000):
        directa([0.1, 0.2, 0.3]); jacobiano([0.1, 0.2, 0.3])
    dt = (time.perf_counter() - t0) / 2000
    print('\nvelocidad del modelo compilado: %.1f us por evaluacion '
          '(directa + jacobiano)' % (dt * 1e6))
    print('  en un lazo de %.0f ms eso es el %.3f %% del presupuesto'
          % (cfg.PERIODO * 1000, dt / cfg.PERIODO * 100))
