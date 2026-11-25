#!/usr/bin/env python3
"""
PARES ARTICULARES del robot PLANTA BRAZO y comprobacion contra los motores.

Modelo dinamico reducido, suficiente para dimensionar: el brazo es un 3R con
J1 vertical y J2/J3 horizontales, asi que el peso NO carga sobre J1 y todo el
esfuerzo de gravedad se lo reparten hombro y codo en el plano r-z.

    tau_grav(j) = g * SUM_i  m_i * (r_i - r_j)      [N.m]

donde r es la coordenada RADIAL (horizontal) del centro de masa de cada
eslabon i que cuelga de la junta j, y r_j la de la propia junta.

Al par de gravedad se le suma el termino inercial, estimado con el momento de
inercia instantaneo del conjunto que cuelga de la junta (teorema de Steiner):

    tau_ine(j) = I_eff(j) * qdd_j          I_eff = SUM_i [ I_i + m_i * d_i^2 ]

y el rozamiento viscoso/seco que declara el URDF:

    tau_fric(j) = b * qd_j + c * signo(qd_j)

Es una estimacion conservadora (desprecia acoplamientos de Coriolis, que a
estas velocidades valen decimas de mN.m) y sirve para lo que hace falta:
saber si el motor da o no da.

--------------------------------------------------------------- los motores
TS-25GAH370-45: motorreductor de 25 mm, motor 370, reduccion 45:1, 12 V,
130 rpm en vacio a la salida (5850 rpm del motor / 45 = 130, cuadra).

Los pares NOMINAL y de BLOQUEO son parametros de este modulo porque hay
variantes del catalogo: si la hoja de datos del laboratorio da otros, se
cambian aqui y todas las cuentas se rehacen solas.
"""
import math

G = 9.80665

# ---------------------------------------------------- masas tomadas del URDF
# (m [kg], distancia del centro de masa medida desde el eje de SU junta,
#  a lo largo del eslabon)
M2, C2 = 0.32, 0.073      # Brazo2, cm a 73 mm de J2
M3, C3 = 0.26, 0.068      # Brazo3, cm a 68 mm de J3
MG, CG = 0.22, 0.061      # gripper, cm a 61 mm de la brida
# cajas de inercia del URDF: ancho x alto del prisma equivalente
I2 = M2 * (0.040 ** 2 + 0.040 ** 2) / 12.0
I3 = M3 * (0.040 ** 2 + 0.040 ** 2) / 12.0
IG = MG * (0.072 ** 2 + 0.045 ** 2) / 12.0
B_VISC, C_SECO = 0.30, 0.20    # <dynamics damping/friction> del URDF

# ------------------------------------------------------------- los motores
MOTOR = {
    'nombre': 'TS-25GAH370-45',
    'tension': 12.0,          # V
    'rpm_vacio': 130.0,       # rpm en el eje de salida
    'reduccion': 45.0,
    'par_nominal': 0.20,      # N.m  (~2.0 kgf.cm)  <- confirmar con la hoja
    'par_bloqueo': 0.55,      # N.m  (~5.6 kgf.cm)  <- confirmar con la hoja
}


def kgfcm(nm):
    """N.m -> kgf.cm, que es como vienen los catalogos de estos motores."""
    return nm / G * 100.0


class Pares:
    def __init__(self, cin, motor=None):
        self.c = cin
        self.motor = dict(MOTOR, **(motor or {}))
        self.L3g = cin.L3 + CG     # J3 -> centro de masa del gripper

    # ------------------------------------------------------------- geometria
    def _radios(self, q):
        """Coordenada radial (horizontal) y altura de cada centro de masa,
        medidas desde el eje de la base."""
        q1, q2, q3 = q
        r2 = self.c.a1 + C2 * math.cos(q2)
        r3 = self.c.a1 + self.c.L2 * math.cos(q2) + C3 * math.cos(q2 + q3)
        rg = self.c.a1 + self.c.L2 * math.cos(q2) + self.L3g * math.cos(q2 + q3)
        rj2 = self.c.a1
        rj3 = self.c.a1 + self.c.L2 * math.cos(q2)
        return r2, r3, rg, rj2, rj3

    # --------------------------------------------------------------- estatica
    def gravedad(self, q):
        """Par de gravedad en cada junta, N.m.  J1 no ve el peso."""
        r2, r3, rg, rj2, rj3 = self._radios(q)
        t2 = G * (M2 * (r2 - rj2) + M3 * (r3 - rj2) + MG * (rg - rj2))
        t3 = G * (M3 * (r3 - rj3) + MG * (rg - rj3))
        return [0.0, t2, t3]

    # --------------------------------------------------------------- inercias
    def inercia(self, q):
        """Momento de inercia equivalente visto por cada junta, kg.m^2."""
        q1, q2, q3 = q
        r2, r3, rg, rj2, rj3 = self._radios(q)
        z2 = C2 * math.sin(q2)
        z3 = self.c.L2 * math.sin(q2) + C3 * math.sin(q2 + q3)
        zg = self.c.L2 * math.sin(q2) + self.L3g * math.sin(q2 + q3)
        # J1 gira alrededor de la vertical: solo cuenta la distancia radial
        i1 = M2 * r2 ** 2 + M3 * r3 ** 2 + MG * rg ** 2
        # J2 y J3: distancia en el plano r-z al eje de la junta
        d2 = lambda r, z, rj, zj: (r - rj) ** 2 + (z - zj) ** 2
        i2 = (I2 + M2 * d2(r2, z2, rj2, 0.0)
              + I3 + M3 * d2(r3, z3, rj2, 0.0)
              + IG + MG * d2(rg, zg, rj2, 0.0))
        zj3 = self.c.L2 * math.sin(q2)
        i3 = (I3 + M3 * d2(r3, z3, rj3, zj3)
              + IG + MG * d2(rg, zg, rj3, zj3))
        return [i1, i2, i3]

    # ----------------------------------------------------------------- total
    def total(self, q, qd=(0, 0, 0), qdd=(0, 0, 0)):
        """Par que debe entregar cada motor, N.m, con el desglose."""
        g = self.gravedad(q)
        i = self.inercia(q)
        ine = [ii * a for ii, a in zip(i, qdd)]
        fri = [B_VISC * v + C_SECO * (1.0 if v > 1e-6 else
                                      -1.0 if v < -1e-6 else 0.0) for v in qd]
        tot = [a + b + c for a, b, c in zip(g, ine, fri)]
        return {'total': tot, 'gravedad': g, 'inercia': ine,
                'friccion': fri, 'I_eff': i}

    # ------------------------------------------------- curva par - velocidad
    def w_vacio(self):
        """Velocidad de vacio del eje de salida, rad/s."""
        return self.motor['rpm_vacio'] * 2.0 * math.pi / 60.0

    def carga(self, tau, w):
        """Punto de trabajo sobre la recta par-velocidad del motor CC.

        Un motor de continua no entrega su par de bloqueo y su velocidad de
        vacio a la vez: la caracteristica es aproximadamente una recta entre
        (0, w_vacio) y (tau_bloqueo, 0).  El punto pedido es realizable si

            tau / tau_bloqueo  +  |w| / w_vacio  <=  1

        El valor devuelto es ese indice: <= 1 el motor lo da, > 1 no."""
        return (abs(tau) / self.motor['par_bloqueo']
                + abs(w) / self.w_vacio())

    # ----------------------------------------------------------- comprobacion
    def veredicto(self, tau_max):
        """Compara el par maximo de cada junta con el motor."""
        nom = self.motor['par_nominal']
        blo = self.motor['par_bloqueo']
        out = []
        for j, t in zip(('J1', 'J2', 'J3'), tau_max):
            t = abs(t)
            if t <= nom:
                estado, margen = 'OK', nom / t if t > 1e-9 else float('inf')
            elif t <= blo:
                estado, margen = 'JUSTO', nom / t
            else:
                estado, margen = 'NO ALCANZA', nom / t
            out.append({'junta': j, 'tau': t, 'kgfcm': kgfcm(t),
                        'uso_nominal': t / nom, 'uso_bloqueo': t / blo,
                        'estado': estado, 'margen': margen})
        return out


if __name__ == '__main__':
    from cinematica import CINEMATICA
    p = Pares(CINEMATICA)
    print('Motor %s: nominal %.3f N.m (%.2f kgf.cm), bloqueo %.3f N.m (%.2f kgf.cm)'
          % (p.motor['nombre'], p.motor['par_nominal'],
             kgfcm(p.motor['par_nominal']), p.motor['par_bloqueo'],
             kgfcm(p.motor['par_bloqueo'])))
    print('\nPeor caso estatico: brazo horizontal completamente estirado '
          '(q2 = q3 = 0)')
    t = p.total((0.0, 0.0, 0.0))
    for j, v in zip(('J1', 'J2', 'J3'), t['total']):
        print('  %s  %7.4f N.m  = %6.3f kgf.cm' % (j, v, kgfcm(v)))
    for v in p.veredicto(t['total']):
        print('  %s  %s  usa el %.0f%% del par nominal' %
              (v['junta'], v['estado'], v['uso_nominal'] * 100))
