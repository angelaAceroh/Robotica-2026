#!/usr/bin/env python3
"""
PERFILES DE VELOCIDAD para la planeacion de trayectorias del PLANTA BRAZO.

Modulo puro (sin ROS) con los dos perfiles de los apuntes de clase, escritos
de forma NORMALIZADA: cada perfil entrega s(t), s'(t) y s''(t) con s(0) = 0 y
s(tf) = 1, de modo que la posicion de cada junta en un segmento es

    q_j(t)    = q_i,j + s(t) * dq_j
    q'_j(t)   = s'(t) * dq_j
    q''_j(t)  = s''(t) * dq_j

Escribirlo asi es lo que SINCRONIZA las juntas: las tres comparten el mismo
s(t), asi que arrancan y paran a la vez y el camino en el espacio de juntas es
un segmento recto.

--------------------------------------------------------------- perfil CUBICO
Polinomio de tercer grado con velocidad nula en los extremos:

    s(t)   = 3*u^2 - 2*u^3          u = t/tf
    s'(t)  = (6*u - 6*u^2) / tf
    s''(t) = (6 - 12*u) / tf^2

Pico de velocidad  s'_max = 1.5/tf  en t = tf/2
Pico de aceleracion s''_max = 6/tf^2 en los extremos

----------------------------------------------------------- perfil TRAPEZOIDAL
Tres tramos: rampa de subida, crucero y rampa de bajada, simetricos respecto
de tf/2.  Queda definido por el tiempo de mezcla tc:

    s''_c = 1 / (tc*(tf - tc))          s'_c = 1 / (tf - tc)

y la restriccion de existencia es  0 < tc <= tf/2.

Los apuntes lo plantean de dos maneras, que son la misma ecuacion despejada
respecto de variables distintas:

    tipo 1   se da la aceleracion de crucero
             tc = tf/2 - (1/2)*sqrt( (tf^2*qddc - 4*dq) / qddc )
             restriccion   |qddc| >= 4*|dq| / tf^2

    tipo 2   se da la velocidad de crucero
             tc = (qi - qf + qdc*tf) / qdc = tf - dq/qdc
             restriccion   |qdc| >= |dq|/tf   (y <= 2|dq|/tf para que haya
                                               crucero de verdad)

Las dos salen de la misma condicion de area:

    qddc * tc * (tf - tc) = dq
"""
import math

TOL = 1e-12


# ------------------------------------------------------------------- utilidades
def signo(v):
    return 1.0 if v >= 0 else -1.0


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------- CUBICO
class Cubico:
    """Perfil cuadratico de los apuntes: polinomio cubico en el tiempo."""

    nombre = 'cubico'

    def __init__(self, tf):
        if tf <= 0:
            raise ValueError('tf debe ser positivo')
        self.tf = float(tf)
        self.tc = None                      # no tiene tramo de mezcla

    def s(self, t):
        u = limitar(t / self.tf, 0.0, 1.0)
        return u * u * (3.0 - 2.0 * u)

    def sd(self, t):
        u = limitar(t / self.tf, 0.0, 1.0)
        return (6.0 * u - 6.0 * u * u) / self.tf

    def sdd(self, t):
        u = limitar(t / self.tf, 0.0, 1.0)
        return (6.0 - 12.0 * u) / (self.tf * self.tf)

    @property
    def sd_max(self):
        return 1.5 / self.tf

    @property
    def sdd_max(self):
        return 6.0 / (self.tf * self.tf)

    def descripcion(self):
        return ('cubico   tf = %.2f s   pico de velocidad en t = tf/2'
                % self.tf)


# ---------------------------------------------------------------- TRAPEZOIDAL
class Trapezoidal:
    """Perfil trapezoidal normalizado, definido por tf y el tiempo de mezcla."""

    nombre = 'trapezoidal'

    def __init__(self, tf, tc):
        if tf <= 0:
            raise ValueError('tf debe ser positivo')
        if tc <= 0 or tc > tf / 2.0 + TOL:
            raise ValueError('tc debe cumplir 0 < tc <= tf/2 (tc = %.4f, '
                             'tf/2 = %.4f)' % (tc, tf / 2.0))
        self.tf = float(tf)
        self.tc = float(min(tc, tf / 2.0))
        self.a = 1.0 / (self.tc * (self.tf - self.tc))   # s''_c
        self.v = self.a * self.tc                        # s'_c

    # ------------------------------------------------------------ evaluacion
    def s(self, t):
        t = limitar(t, 0.0, self.tf)
        if t <= self.tc:
            return 0.5 * self.a * t * t
        if t <= self.tf - self.tc:
            return self.a * self.tc * (t - self.tc / 2.0)
        r = self.tf - t
        return 1.0 - 0.5 * self.a * r * r

    def sd(self, t):
        t = limitar(t, 0.0, self.tf)
        if t <= self.tc:
            return self.a * t
        if t <= self.tf - self.tc:
            return self.v
        return self.a * (self.tf - t)

    def sdd(self, t):
        t = limitar(t, 0.0, self.tf)
        if t < self.tc:
            return self.a
        if t <= self.tf - self.tc:
            return 0.0
        return -self.a

    @property
    def sd_max(self):
        return self.v

    @property
    def sdd_max(self):
        return self.a

    @property
    def t_crucero(self):
        return self.tf - 2.0 * self.tc

    def descripcion(self):
        return ('trapezoidal   tf = %.2f s   tc = %.3f s   '
                'crucero = %.3f s (%.0f %% del tramo)'
                % (self.tf, self.tc, self.t_crucero,
                   self.t_crucero / self.tf * 100.0))

    # ------------------------------------------------- constructores de clase
    @classmethod
    def por_aceleracion(cls, dq, tf, qddc):
        """TIPO 1 de los apuntes: se define la aceleracion de crucero.

        tc = tf/2 - (1/2)*sqrt( (tf^2*qddc - 4*dq) / qddc )
        """
        dq, tf, qddc = float(dq), float(tf), float(qddc)
        if abs(dq) < TOL:
            return cls(tf, tf / 2.0)
        qddc = abs(qddc) * signo(dq)          # el signo lo manda el recorrido
        cota = 4.0 * abs(dq) / (tf * tf)
        if abs(qddc) < cota - 1e-9:
            raise ValueError(
                'aceleracion demasiado BAJA: |qddc| = %.4f < 4|dq|/tf^2 = '
                '%.4f, el tramo no cabe en tf' % (abs(qddc), cota))
        rad = (tf * tf * qddc - 4.0 * dq) / qddc
        tc = tf / 2.0 - 0.5 * math.sqrt(max(rad, 0.0))
        return cls(tf, tc)

    @classmethod
    def por_velocidad(cls, dq, tf, qdc):
        """TIPO 2 de los apuntes: se define la velocidad de crucero.

        tc = tf - dq/qdc     (equivale a (qi - qf + qdc*tf)/qdc)
        """
        dq, tf, qdc = float(dq), float(tf), float(qdc)
        if abs(dq) < TOL:
            return cls(tf, tf / 2.0)
        qdc = abs(qdc) * signo(dq)
        # con tf FIJO la velocidad de crucero no es libre: como
        # qdc = dq/(tf - tc) y 0 < tc <= tf/2, solo caben
        #     |dq|/tf  <=  |qdc|  <=  2|dq|/tf
        lo, hi = abs(dq) / tf, 2.0 * abs(dq) / tf
        if abs(qdc) < lo - 1e-12:
            raise ValueError(
                'velocidad demasiado BAJA: |qdc| = %.4f < |dq|/tf = %.4f, '
                'no da tiempo a recorrer el tramo' % (abs(qdc), lo))
        if abs(qdc) > hi + 1e-9:
            raise ValueError(
                'velocidad demasiado ALTA: |qdc| = %.4f > 2|dq|/tf = %.4f, '
                'el tramo se acabaria antes de tf' % (abs(qdc), hi))
        tc = tf - dq / qdc
        return cls(tf, min(tc, tf / 2.0))

    @classmethod
    def tiempo_minimo(cls, dq, qdc, qddc):
        """tf mas corto que respeta a la vez velocidad y aceleracion maximas.

        Util para responder '¿cuanto puede tardar como poco este tramo?'."""
        dq = abs(float(dq))
        if dq < TOL:
            return 0.0, None
        qdc, qddc = abs(float(qdc)), abs(float(qddc))
        tc = qdc / qddc                       # rampa hasta la velocidad tope
        if dq >= qdc * tc:                    # llega a crucero: trapecio
            tf = dq / qdc + tc
        else:                                 # no llega: triangulo
            tc = math.sqrt(dq / qddc)
            tf = 2.0 * tc
        return tf, tc


def tf_cubico(dq, qdc, qddc):
    """Duracion minima de un tramo cubico que respeta velocidad y aceleracion.

    El cubico tiene sus picos cerrados:  qd_max = 1.5*dq/tf  y
    qdd_max = 6*dq/tf^2, asi que basta despejar tf de cada uno y quedarse
    con el mas restrictivo."""
    dq = abs(float(dq))
    if dq < TOL:
        return 0.0
    tf_v = 1.5 * dq / abs(float(qdc))
    tf_a = math.sqrt(6.0 * dq / abs(float(qddc)))
    return max(tf_v, tf_a)


# --------------------------------------------------------------------- fabrica
def construir(tipo, dq_dominante, tf, valor=None, modo='velocidad'):
    """Crea el perfil que pide el panel.

    tipo   'trapezoidal' | 'cubico'
    modo   (solo trapezoidal) 'velocidad' -> tipo 2 ;  'aceleracion' -> tipo 1
    valor  velocidad o aceleracion de crucero de la junta DOMINANTE, en las
           mismas unidades que dq_dominante (rad y segundos)

    La junta dominante es la de mayor |dq| del segmento: es la que marca el
    ritmo, y las demas se escalan con el mismo s(t).
    """
    tipo = str(tipo).lower()
    if tipo.startswith('cub'):
        return Cubico(tf)
    if valor is None:
        raise ValueError('el perfil trapezoidal necesita un valor de crucero')
    if str(modo).lower().startswith('acel'):
        return Trapezoidal.por_aceleracion(dq_dominante, tf, valor)
    return Trapezoidal.por_velocidad(dq_dominante, tf, valor)


# ------------------------------------------------------------------ autoprueba
if __name__ == '__main__':
    print('Comprobacion de los perfiles\n')
    dq, tf = math.radians(-150.0), 3.0

    p1 = Trapezoidal.por_aceleracion(dq, tf, math.radians(-79.37))
    print('tipo 1 (aceleracion):', p1.descripcion())
    print('   tc = %.6f s  (esperado 0.900000)' % p1.tc)
    print('   qdc = %.3f deg/s' % math.degrees(p1.sd_max * dq))

    p2 = Trapezoidal.por_velocidad(dq, tf, math.radians(-71.43))
    print('tipo 2 (velocidad)  :', p2.descripcion())
    print('   tc = %.6f s  (esperado 0.900000)' % p2.tc)

    p3 = Cubico(tf)
    print('cubico              :', p3.descripcion())
    print('   pico de velocidad %.2f deg/s (trapecio: %.2f)'
          % (math.degrees(p3.sd_max * dq), math.degrees(p1.sd_max * dq)))

    print('\ncierre de los perfiles (s(tf) debe valer 1):')
    for p in (p1, p2, p3):
        print('   %-12s s(0) = %.9f   s(tf) = %.9f'
              % (p.nombre, p.s(0.0), p.s(p.tf)))

    tfm, tcm = Trapezoidal.tiempo_minimo(dq, math.radians(200.0),
                                         math.radians(400.0))
    print('\ntiempo minimo del tramo con 200 deg/s y 400 deg/s2: %.3f s' % tfm)
