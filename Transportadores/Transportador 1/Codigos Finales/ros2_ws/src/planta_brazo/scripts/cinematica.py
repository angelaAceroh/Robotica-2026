#!/usr/bin/env python3
"""
Cinematica del robot antropomorfico 3R "PLANTA BRAZO": DIRECTA e INVERSA.

Modulo puro (sin ROS) para que lo usen por igual el nodo de cinematica directa,
el panel de control y el banco de comparacion.

--------------------------------------------------------------------- geometria
Parametros Denavit-Hartenberg (los mismos de CinematicaDirecta.pdf, mas el
offset real de hombro a1 que tiene el CAD y la longitud del gripper Lg):

    i | theta_i | d_i | a_i    | alpha_i
    --+---------+-----+--------+---------
    1 |   q1    | d1  | a1     |  +pi/2
    2 |   q2    |  0  | L2     |    0
    3 |   q3    |  0  | L      |    0      L = L3        -> brida
                                           L = L3 + Lg   -> punta de los dedos

--------------------------------------------------------------- cinematica DIRECTA
Dados los ANGULOS, ¿donde queda la herramienta?

    r = a1 + L2*cos(q2) + L*cos(q2+q3)
    x = r*cos(q1)
    y = r*sin(q1)
    z = d1 + L2*sin(q2) + L*sin(q2+q3)

Solucion unica: una terna de angulos da exactamente un punto.

--------------------------------------------------------------- cinematica INVERSA
Dado el PUNTO, ¿que angulos hay que poner?  Se resuelve en forma cerrada
(no iterativa) separando el giro de la base del brazo plano de 2 eslabones:

  1) giro de base          q1 = atan2(y, x)          (o q1 + pi, "hombro atras")

  2) el problema queda plano en (u, v), con el hombro en el origen:
         u = +-sqrt(x^2 + y^2) - a1      (+ hombro al frente, - hombro atras)
         v = z - d1

  3) ley del coseno para el codo:
         cos(q3) = (u^2 + v^2 - L2^2 - L^2) / (2*L2*L)  =  D
         q3 = -acos(D)   codo ARRIBA        q3 = +acos(D)   codo ABAJO

  4) hombro, restando el aporte del antebrazo:
         q2 = atan2(v, u) - atan2(L*sin(q3), L2 + L*cos(q3))

Por tanto hay hasta CUATRO soluciones para un mismo punto (2 de base x 2 de
codo).  El punto cartesiano es el mismo en todas; cambia la postura.  De ahi
que la inversa no sea "la directa al reves": hay que ELEGIR.

Casos que hay que vigilar y que este modulo marca explicitamente:
  - |D| > 1        el punto esta fuera del alcance (o demasiado cerca del eje)
  - |D| = 1        brazo estirado o completamente plegado: singularidad de codo,
                   las dos soluciones de codo se funden en una
  - x = y = 0      el punto cae sobre el eje de la base: q1 es indeterminado
                   (infinitas soluciones); se conserva el q1 actual
  - fuera de rango la solucion existe pero el robot no puede adoptarla
"""
import math

# ------------------------------------------- MEDIDAS REALES DEL ROBOT FISICO
# Fuente unica de la geometria.  Valores medidos sobre el robot y recogidos en
# correciones/robot_antropomorfo_3gdl_referencia.md (2026-10-22).
#
# Lo que cambio respecto a los valores del CAD que habia antes:
#   D1  0.34265 -> 0.200    el CAD usaba una base mucho mas alta que la real
#   L2  0.14636 -> 0.145    se unifica con la medida sobre el robot
#   LG  0.12300 -> 0.100    gripper sin disenar; longitud provisional L4
D1 = 0.200       # base -> eje del hombro          [m]   (L1 en la guia)
A1 = 0.05815     # offset horizontal del hombro    [m]
L2 = 0.145       # Brazo2 (hombro -> codo)         [m]
L3 = 0.16285     # Brazo3 (codo -> brida)          [m]   (del CAD, por medir)
LG = 0.100       # brida -> punta del gripper      [m]   (L4, PROVISIONAL)

# ------------------------------------------------------------------- limites
# OJO: el home es (0, 90, -90) grados, o sea que J2 ARRANCA en +90.  Con el
# limite anterior de +-90 el home caia justo en el tope: el control pedia pasar
# de ahi, la saturacion lo cortaba, el error no se cerraba nunca y el brazo
# entraba en ciclo limite (el "baile").  La trayectoria de prueba de la guia
# llega ademas a theta2 ~ 127 y theta3 ~ -142 grados.
#
# PENDIENTE DE CONFIRMAR EN EL ROBOT FISICO que J2 llega a 135 sin chocar.
# TOPES MECANICOS MEDIDOS en el robot el 2026-10-23, moviendo cada junta a
# mano con control_vel.angulos().  Se dejan unos 5 grados de margen:
#     J1  -101.2 / +100.5   (lo limitan los cables, no un tope)
#     J2   -73.8 / +138.3   (arriba choca con la cola del motor)
#     J3  -127.6 / +110.0
LIMITES = {
    'J1': (math.radians(-95),  math.radians(95)),
    'J2': (math.radians(-68),  math.radians(133)),
    'J3': (math.radians(-123), math.radians(105)),
}

# postura de reposo: home en L (brazo vertical, antebrazo horizontal a +X)
HOME = (0.0, math.pi / 2, -math.pi / 2)

TOL = 1e-9


def envolver(a):
    """Angulo equivalente dentro de (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Cinematica:
    def __init__(self, d1=D1, a1=A1, l2=L2, l3=L3, lg=LG, limites=None):
        self.d1, self.a1, self.L2, self.L3, self.Lg = d1, a1, l2, l3, lg
        self.limites = dict(limites or LIMITES)

    # ------------------------------------------------------------------ ayudas
    def largo(self, extremo='tcp'):
        """Longitud efectiva del ultimo eslabon segun el punto de interes."""
        return self.L3 + self.Lg if str(extremo).lower() == 'tcp' else self.L3

    def alcance(self, extremo='tcp'):
        """(radio minimo, radio maximo) medidos desde el hombro."""
        L = self.largo(extremo)
        return abs(self.L2 - L), self.L2 + L

    def en_limites(self, q):
        return all(lo - TOL <= v <= hi + TOL
                   for v, (lo, hi) in zip(q, self.limites.values()))

    def fuera(self, q):
        """Lista de juntas que se salen de rango (vacia si todo bien)."""
        return [n for v, (n, (lo, hi)) in zip(q, self.limites.items())
                if not (lo - TOL <= v <= hi + TOL)]

    # -------------------------------------------------------------- CD directa
    def directa(self, q, extremo='tcp'):
        """Angulos (rad) -> punto (x, y, z) en metros."""
        q1, q2, q3 = q
        L = self.largo(extremo)
        r = self.a1 + self.L2 * math.cos(q2) + L * math.cos(q2 + q3)
        return (r * math.cos(q1),
                r * math.sin(q1),
                self.d1 + self.L2 * math.sin(q2) + L * math.sin(q2 + q3))

    # -------------------------------------------------------------- CI inversa
    def inversa(self, punto, extremo='tcp', q_actual=None):
        """Punto (x, y, z) -> lista de soluciones, de la mejor a la peor.

        Cada solucion es un dict:
            q          terna (q1, q2, q3) en rad
            config     'hombro_frente/codo_arriba', etc.
            valida     True si respeta los limites articulares
            fuera      juntas que se salen de rango
            singular   True si |D| ~ 1 (brazo estirado o plegado)
            residuo    |directa(q) - punto| en metros (control de calidad)
        """
        x, y, z = punto
        L = self.largo(extremo)
        rxy = math.hypot(x, y)
        v = z - self.d1
        eje = rxy < 1e-6          # punto sobre el eje de la base

        base = (q_actual[0] if (eje and q_actual is not None)
                else math.atan2(y, x))
        ramas = ((+1.0, envolver(base), 'hombro_frente'),
                 (-1.0, envolver(base + math.pi), 'hombro_atras'))

        soluciones = []
        for signo, q1, nombre_base in ramas:
            u = signo * rxy - self.a1
            D = (u * u + v * v - self.L2 ** 2 - L ** 2) / (2.0 * self.L2 * L)
            if abs(D) > 1.0 + 1e-6:
                continue                      # fuera del alcance por esta rama
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
                    break                     # las dos de codo son la misma
        return soluciones

    def resolver(self, punto, extremo='tcp', preferencia='codo_arriba',
                 q_actual=None):
        """Inversa + eleccion de UNA solucion.

        preferencia:
            'codo_arriba' / 'codo_abajo'  postura pedida (por defecto arriba)
            'cercana'                     la que menos mueva las juntas desde
                                          q_actual (criterio de continuidad,
                                          el que usan los robots reales)

        Devuelve (solucion, todas).  solucion es None si ninguna es alcanzable.
        """
        todas = self.inversa(punto, extremo, q_actual)
        validas = [s for s in todas if s['valida']]
        if not validas:
            return None, todas

        if preferencia == 'cercana' and q_actual is not None:
            clave = lambda s: sum(abs(envolver(a - b))
                                  for a, b in zip(s['q'], q_actual))
        else:
            # primero la postura pedida, y dentro de ella la de hombro al frente
            clave = lambda s: (0 if preferencia in s['config'] else 1,
                               0 if 'hombro_frente' in s['config'] else 1)
        return min(validas, key=clave), todas


# instancia lista para usar con los valores del CAD
CINEMATICA = Cinematica()


# ------------------------------------------------------------------ autoprueba
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
