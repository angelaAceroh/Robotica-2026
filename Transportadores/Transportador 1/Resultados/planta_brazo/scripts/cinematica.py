#!/usr/bin/env python3
# Cinematica directa e inversa (fuente unica de la geometria).
import math

D1 = 0.200
A1 = 0.05815
L2 = 0.145
L3 = 0.16285
LG = 0.100

LIMITES = {
    'J1': (math.radians(-95),  math.radians(95)),
    'J2': (math.radians(-68),  math.radians(133)),
    'J3': (math.radians(-123), math.radians(105)),
}

HOME = (0.0, math.pi / 2, -math.pi / 2)

TOL = 1e-9


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


class Cinematica:
    def __init__(self, d1=D1, a1=A1, l2=L2, l3=L3, lg=LG, limites=None):
        self.d1, self.a1, self.L2, self.L3, self.Lg = d1, a1, l2, l3, lg
        self.limites = dict(limites or LIMITES)

    def largo(self, extremo='tcp'):
        return self.L3 + self.Lg if str(extremo).lower() == 'tcp' else self.L3

    def alcance(self, extremo='tcp'):
        L = self.largo(extremo)
        return abs(self.L2 - L), self.L2 + L

    def en_limites(self, q):
        return all(lo - TOL <= v <= hi + TOL
                   for v, (lo, hi) in zip(q, self.limites.values()))

    def fuera(self, q):
        return [n for v, (n, (lo, hi)) in zip(q, self.limites.items())
                if not (lo - TOL <= v <= hi + TOL)]

    def directa(self, q, extremo='tcp'):
        q1, q2, q3 = q
        L = self.largo(extremo)
        r = self.a1 + self.L2 * math.cos(q2) + L * math.cos(q2 + q3)
        return (r * math.cos(q1),
                r * math.sin(q1),
                self.d1 + self.L2 * math.sin(q2) + L * math.sin(q2 + q3))

    def inversa(self, punto, extremo='tcp', q_actual=None):
        x, y, z = punto
        L = self.largo(extremo)
        rxy = math.hypot(x, y)
        v = z - self.d1
        eje = rxy < 1e-6

        base = (q_actual[0] if (eje and q_actual is not None)
                else math.atan2(y, x))
        ramas = ((+1.0, envolver(base), 'hombro_frente'),
                 (-1.0, envolver(base + math.pi), 'hombro_atras'))

        soluciones = []
        for signo, q1, nombre_base in ramas:
            u = signo * rxy - self.a1
            D = (u * u + v * v - self.L2 ** 2 - L ** 2) / (2.0 * self.L2 * L)
            if abs(D) > 1.0 + 1e-6:
                continue
            singular = abs(abs(D) - 1.0) < 1e-6
            D = max(-1.0, min(1.0, D))
            for s3, nombre_codo in ((-1.0, 'codo_arriba'), (+1.0, 'codo_abajo')):
                q3 = s3 * math.acos(D)
                q2 = envolver(math.atan2(v, u)
                              - math.atan2(L * math.sin(q3),
                                           self.L2 + L * math.cos(q3)))
                q = (q1, q2, envolver(q3))
                p = self.directa(q, extremo)
                soluciones.append({
                    'q': q,
                    'config': f'{nombre_base}/{nombre_codo}',
                    'valida': self.en_limites(q),
                    'fuera': self.fuera(q),
                    'singular': singular,
                    'eje': eje,
                    'residuo': math.dist(p, punto),
                })
                if singular:
                    break
        return soluciones

    def resolver(self, punto, extremo='tcp', preferencia='codo_arriba',
                 q_actual=None):
        todas = self.inversa(punto, extremo, q_actual)
        validas = [s for s in todas if s['valida']]
        if not validas:
            return None, todas

        if preferencia == 'cercana' and q_actual is not None:
            clave = lambda s: sum(abs(envolver(a - b))
                                  for a, b in zip(s['q'], q_actual))
        else:
            clave = lambda s: (0 if preferencia in s['config'] else 1,
                               0 if 'hombro_frente' in s['config'] else 1)
        return min(validas, key=clave), todas

CINEMATICA = Cinematica()

if __name__ == '__main__':
    import random
    c = CINEMATICA
    print('Comprobacion CD -> CI -> CD con 20000 posturas aleatorias validas')
    peor = 0.0
    fallos = 0
    for _ in range(20000):
        q = tuple(random.uniform(lo, hi) for lo, hi in c.limites.values())
        p = c.directa(q)
        sol, todas = c.resolver(p, q_actual=q, preferencia='cercana')
        if sol is None:
            fallos += 1
            continue
        peor = max(peor, math.dist(c.directa(sol['q']), p))
    print(f'  soluciones no encontradas : {fallos}')
    print(f'  peor error de cierre      : {peor * 1e6:.3f} micras')
