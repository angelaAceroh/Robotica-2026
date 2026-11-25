#!/usr/bin/env python3
# Jacobiano del robot 3R.
import math

import numpy as np

from cinematica import CINEMATICA


def jacobiano(q, cin=CINEMATICA, extremo='tcp'):
    q1, q2, q3 = q
    L = cin.largo(extremo)
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s23, c23 = math.sin(q2 + q3), math.cos(q2 + q3)
    r = cin.a1 + cin.L2 * c2 + L * c23
    S = cin.L2 * s2 + L * s23
    C = cin.L2 * c2 + L * c23
    return np.array([
        [-r * s1, -c1 * S, -c1 * L * s23],
        [ r * c1, -s1 * S, -s1 * L * s23],
        [    0.0,       C,       L * c23]])


def determinante(q, cin=CINEMATICA, extremo='tcp'):
    _, q2, q3 = q
    L = cin.largo(extremo)
    r = cin.a1 + cin.L2 * math.cos(q2) + L * math.cos(q2 + q3)
    return -r * cin.L2 * L * math.sin(q3)


def inverso(q, amortiguacion=0.0, cin=CINEMATICA, extremo='tcp'):
    J = jacobiano(q, cin, extremo)
    if amortiguacion <= 0.0:
        return np.linalg.pinv(J)
    lam2 = float(amortiguacion) ** 2
    return J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(3))


def vel_tcp(q, qd, cin=CINEMATICA, extremo='tcp'):
    return jacobiano(q, cin, extremo) @ np.asarray(qd, float).reshape(3)


def vel_juntas(q, xd, amortiguacion=0.0, cin=CINEMATICA, extremo='tcp'):
    return inverso(q, amortiguacion, cin, extremo) @ np.asarray(xd, float).reshape(3)


def medidas(q, cin=CINEMATICA, extremo='tcp'):
    J = jacobiano(q, cin, extremo)
    s = np.linalg.svd(J, compute_uv=False)
    return {'det': float(np.linalg.det(J)),
            'det_cerrado': determinante(q, cin, extremo),
            'condicion': float(s[0] / s[-1]) if s[-1] > 1e-15 else float('inf'),
            'manipulabilidad': float(abs(np.linalg.det(J)))}

if __name__ == '__main__':
    import random
    print('Comprobacion de J contra diferencias finitas, 5000 posturas')
    peor_J = peor_d = 0.0
    h = 1e-7
    for _ in range(5000):
        q = [random.uniform(lo, hi) for lo, hi in CINEMATICA.limites.values()]
        J = jacobiano(q)
        Jn = np.zeros((3, 3))
        for j in range(3):
            qa = list(q); qa[j] += h
            qb = list(q); qb[j] -= h
            Jn[:, j] = (np.array(CINEMATICA.directa(qa))
                        - np.array(CINEMATICA.directa(qb))) / (2 * h)
        peor_J = max(peor_J, float(np.max(np.abs(J - Jn))))
        peor_d = max(peor_d, abs(np.linalg.det(J) - determinante(q)))
    print('  peor error de J            : %.3e m/rad' % peor_J)
    print('  peor error del determinante: %.3e' % peor_d)
