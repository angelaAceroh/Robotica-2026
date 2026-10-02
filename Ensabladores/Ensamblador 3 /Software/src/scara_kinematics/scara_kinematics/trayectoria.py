"""Planificacion de trayectorias del SCARA: los caminos y los perfiles.

Biblioteca pura -sin ROS, sin numpy- que usa el nodo `tray`. No toca ninguno de
los seis nodos: solo importa `model.py` para la cinematica.

Una trayectoria son SIEMPRE dos cosas independientes, y aqui estan separadas
a proposito:

    CAMINO   la forma: por donde pasa la punta.      u in [0,1] -> q
    PERFIL   el tiempo: como se recorre esa forma.   t in [0,T] -> u

    q(t) = camino( perfil(t/T) )

Separarlas es lo que permite combinar los cinco caminos con los cinco perfiles
-25 metodos- sin escribir 25 funciones. El perfil no sabe si el camino es una
recta o un circulo, y el camino no sabe si se recorre en 2 o en 20 segundos.

LOS CINCO PERFILES (espacio del tiempo)

    cubico     3er grado.  Velocidad continua, aceleracion a escalones.
               s = 3u^2 - 2u^3            kv = 1.5      ka = 6
    quintico   5o grado.   Aceleracion continua (arranca y para sin tiron).
               s = 10u^3 - 15u^4 + 6u^5   kv = 1.875    ka = 5.774
    septico    7o grado.   Jerk continuo, el mas suave de los polinomicos.
               s = 35u^4 - 84u^5 + 70u^6 - 20u^7   kv = 2.1875
    trapecio   LSPB: rampa - crucero - rampa. El mas rapido para un techo de
               velocidad dado (kv = 1/(1-beta)), pero la aceleracion salta.
    scurve     Doble S: el trapecio con el jerk acotado. Siete tramos.

    kv es el pico de velocidad normalizada: con duracion T y recorrido D, la
    velocidad maxima es kv*D/T. Es lo que usa `duracion_auto` para respetar el
    max_vel del YAML sin pasarse.

LOS CINCO CAMINOS (espacio de la forma)

    junta      interpolacion en el espacio articular. No hay IK: es el unico
               que nunca falla y nunca pasa por una singularidad, pero la punta
               describe una curva rara.
    recta      linea recta en cartesiano. IK en cada muestra.
    arco       arco de circunferencia (o circulo completo, o helice si z varia).
               Tambien por tres puntos: `CaminoArco.por_tres`.
    spline     spline cubica natural por N puntos via. Pasa por todos.
    poligonal  segmentos rectos encadenados por N puntos via.

Los cuatro ultimos resuelven la cinematica inversa; `model.py` ya la traia
escrita (`ik`, `ik_all`) pero ningun nodo la usaba.

Uso:

    from scara_kinematics.trayectoria import CaminoRecta, PERFILES, Trayectoria

    cam = CaminoRecta(modelo, (0.30, 0.00, 0.15), (0.20, 0.18, 0.11))
    tr  = Trayectoria(cam, PERFILES['quintico'](), 3.0, q_actual)
    for t, q, qd in tr.recorrer(0.02):
        ...
"""
from __future__ import annotations

import math
from typing import Iterator, Sequence

from scara_kinematics.model import IKError, ScaraModel

TAU = 2.0 * math.pi


def _lim(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return lo if v < lo else (hi if v > hi else v)


# =========================================================================
#  PERFILES: el tiempo.  s(u): [0,1] -> [0,1], con s(0)=0 y s(1)=1
# =========================================================================
class Perfil:
    """Ley horaria normalizada. Las derivadas son respecto de u, no de t."""

    nombre = 'perfil'
    KV: float | None = None      # pico de |ds/du|, si se conoce exacto
    KA: float | None = None      # pico de |d2s/du2|

    def s(self, u: float) -> float:
        raise NotImplementedError

    def ds(self, u: float) -> float:
        raise NotImplementedError

    def dds(self, u: float) -> float:
        raise NotImplementedError

    # Los picos son lo que convierte una duracion en una velocidad: con un
    # recorrido D y una duracion T, v_max = kv*D/T y a_max = ka*D/T^2.
    def _picos(self) -> tuple[float, float]:
        if getattr(self, '_cache', None) is None:
            kv, ka = self.KV, self.KA
            if kv is None or ka is None:
                n = 2000
                m_v = max(abs(self.ds(i / n)) for i in range(n + 1))
                m_a = max(abs(self.dds(i / n)) for i in range(n + 1))
                kv = m_v if kv is None else kv
                ka = m_a if ka is None else ka
            self._cache = (kv, ka)
        return self._cache

    @property
    def kv(self) -> float:
        return self._picos()[0]

    @property
    def ka(self) -> float:
        return self._picos()[1]

    def __repr__(self) -> str:
        return f'{self.nombre}(kv={self.kv:.3f}, ka={self.ka:.3f})'


class PerfilCubico(Perfil):
    """Polinomio de 3er grado con velocidad nula en los extremos.

    s = 3u^2 - 2u^3. La aceleracion salta de +6 a -6 en el punto medio y de 0 a
    +6 al arrancar: es el escalon de par que se oye en el brazo real.
    """

    nombre = 'cubico'
    KV = 1.5
    KA = 6.0

    def s(self, u):
        u = _lim(u)
        return u * u * (3.0 - 2.0 * u)

    def ds(self, u):
        u = _lim(u)
        return 6.0 * u * (1.0 - u)

    def dds(self, u):
        u = _lim(u)
        return 6.0 - 12.0 * u


class PerfilQuintico(Perfil):
    """Polinomio de 5o grado: ademas de la velocidad, anula la aceleracion.

    s = 10u^3 - 15u^4 + 6u^5. Es el que se usa por defecto: arranca y para sin
    escalon de par, a cambio de un pico de velocidad 25% mayor que el cubico.
    """

    nombre = 'quintico'
    KV = 1.875
    KA = 10.0 / math.sqrt(3.0)

    def s(self, u):
        u = _lim(u)
        return u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))

    def ds(self, u):
        u = _lim(u)
        return 30.0 * u * u * (1.0 - u) ** 2

    def dds(self, u):
        u = _lim(u)
        return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)


class PerfilSeptico(Perfil):
    """Polinomio de 7o grado: anula tambien el jerk en los extremos.

    s = 35u^4 - 84u^5 + 70u^6 - 20u^7. El mas suave de los polinomicos y el que
    menos excita la estructura; paga con kv = 2.1875.
    """

    nombre = 'septico'
    KV = 2.1875

    def s(self, u):
        u = _lim(u)
        return u ** 4 * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)))

    def ds(self, u):
        u = _lim(u)
        return 140.0 * u ** 3 * (1.0 - u) ** 3

    def dds(self, u):
        u = _lim(u)
        return 420.0 * u * u * (1.0 - u) ** 2 * (1.0 - 2.0 * u)


class PerfilTrapecio(Perfil):
    """LSPB: lineal con mezclas parabolicas. Rampa - crucero - rampa.

    beta es la fraccion de la duracion que dura cada rampa (0 < beta <= 0.5).
    Con beta = 0.5 no hay crucero y el perfil es triangular.

        v = 1/(1-beta)      a = v/beta

    Es el perfil optimo en tiempo para un techo de velocidad dado, y por eso es
    el clasico de los controladores industriales. Su defecto es que la
    aceleracion salta: 0 -> a en el instante 0.
    """

    nombre = 'trapecio'

    def __init__(self, beta: float = 0.25):
        if not (0.0 < beta <= 0.5):
            raise ValueError(f'beta del trapecio fuera de (0, 0.5]: {beta}')
        self.beta = float(beta)
        self.v = 1.0 / (1.0 - self.beta)
        self.a = self.v / self.beta
        self.KV, self.KA = self.v, self.a
        self._cache = None

    def s(self, u):
        u, b = _lim(u), self.beta
        if u < b:
            return 0.5 * self.a * u * u
        if u <= 1.0 - b:
            return 0.5 * self.a * b * b + self.v * (u - b)
        return 1.0 - 0.5 * self.a * (1.0 - u) ** 2

    def ds(self, u):
        u, b = _lim(u), self.beta
        if u < b:
            return self.a * u
        if u <= 1.0 - b:
            return self.v
        return self.a * (1.0 - u)

    def dds(self, u):
        u, b = _lim(u), self.beta
        if u < b:
            return self.a
        if u <= 1.0 - b:
            return 0.0
        return -self.a

    def __repr__(self):
        return f'trapecio(beta={self.beta:.2f}, kv={self.kv:.3f}, ka={self.ka:.3f})'


class PerfilScurve(Perfil):
    """Doble S: el trapecio con el jerk acotado. Siete tramos.

    tj es la fraccion de tiempo de cada rampa de jerk y ta la fase de
    aceleracion entera (incluye las dos rampas de jerk):

        2*tj <= ta <= 0.5      v = 1/(1-ta)    a = v/(ta-tj)    j = a/tj

    El perfil es simetrico, asi que la mitad de vuelta se calcula con
    s(u) = 1 - s(1-u) y no hay que escribir los tramos de frenada.

    Es lo que se pide cuando el brazo lleva algo que no puede dar un tiron: el
    par no salta nunca, sube en rampa.
    """

    nombre = 'scurve'

    def __init__(self, tj: float = 0.15, ta: float = 0.35):
        if not (0.0 < tj <= ta / 2.0):
            raise ValueError(f'la rampa de jerk pide 0 < tj <= ta/2: tj={tj}, ta={ta}')
        if not (ta <= 0.5):
            raise ValueError(f'la fase de aceleracion no cabe: ta={ta} > 0.5')
        self.tj, self.ta = float(tj), float(ta)
        self.v = 1.0 / (1.0 - self.ta)
        self.a = self.v / (self.ta - self.tj)
        self.j = self.a / self.tj
        self.KV, self.KA = self.v, self.a
        self._cache = None
        self._s_ta = self._acel(self.ta)[0]

    def _acel(self, u: float) -> tuple[float, float, float]:
        """(s, ds, dds) dentro de la fase de aceleracion, 0 <= u <= ta."""
        tj, a, j = self.tj, self.a, self.j
        if u <= tj:                                  # 1. el jerk sube
            return j * u ** 3 / 6.0, 0.5 * j * u * u, j * u
        s1, v1 = j * tj ** 3 / 6.0, 0.5 * j * tj * tj
        w = u - tj
        if u <= self.ta - tj:                        # 2. aceleracion constante
            return s1 + v1 * w + 0.5 * a * w * w, v1 + a * w, a
        w2 = self.ta - 2.0 * tj
        s2 = s1 + v1 * w2 + 0.5 * a * w2 * w2
        v2 = v1 + a * w2
        w = u - (self.ta - tj)                       # 3. el jerk baja
        return (s2 + v2 * w + 0.5 * a * w * w - j * w ** 3 / 6.0,
                v2 + a * w - 0.5 * j * w * w,
                a - j * w)

    def _medio(self, u: float) -> tuple[float, float, float]:
        if u <= self.ta:
            return self._acel(u)
        return self._s_ta + self.v * (u - self.ta), self.v, 0.0   # 4. crucero

    def s(self, u):
        u = _lim(u)
        return self._medio(u)[0] if u <= 0.5 else 1.0 - self._medio(1.0 - u)[0]

    def ds(self, u):
        u = _lim(u)
        return self._medio(u)[1] if u <= 0.5 else self._medio(1.0 - u)[1]

    def dds(self, u):
        u = _lim(u)
        return self._medio(u)[2] if u <= 0.5 else -self._medio(1.0 - u)[2]

    def __repr__(self):
        return (f'scurve(tj={self.tj:.2f}, ta={self.ta:.2f}, '
                f'kv={self.kv:.3f}, ka={self.ka:.3f}, j={self.j:.1f})')


PERFILES = {
    'cubico': PerfilCubico,
    'quintico': PerfilQuintico,
    'septico': PerfilSeptico,
    'trapecio': PerfilTrapecio,
    'scurve': PerfilScurve,
}


def perfil(nombre: str, **kw) -> Perfil:
    """Fabrica por nombre; lo que usa el nodo para leer `perfil=quintico`."""
    try:
        return PERFILES[nombre](**kw)
    except KeyError:
        raise ValueError(
            f'perfil desconocido: {nombre!r}. Los hay: {", ".join(PERFILES)}') from None


# =========================================================================
#  SPLINE CUBICA NATURAL (1D). Sin numpy: Thomas sobre la tridiagonal.
# =========================================================================
class _Spline1D:
    """Spline cubica natural por (t_i, y_i). Segunda derivada nula en los extremos."""

    def __init__(self, t: Sequence[float], y: Sequence[float]):
        self.t, self.y = list(t), list(y)
        n = len(t)
        self.M = [0.0] * n
        if n < 3:
            return
        h = [t[i + 1] - t[i] for i in range(n - 1)]
        # Sistema tridiagonal en las segundas derivadas M[1..n-2].
        a = [0.0] * n
        b = [0.0] * n
        c = [0.0] * n
        d = [0.0] * n
        for i in range(1, n - 1):
            a[i] = h[i - 1]
            b[i] = 2.0 * (h[i - 1] + h[i])
            c[i] = h[i]
            d[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
        for i in range(2, n - 1):                       # eliminacion hacia delante
            m = a[i] / b[i - 1]
            b[i] -= m * c[i - 1]
            d[i] -= m * d[i - 1]
        for i in range(n - 2, 0, -1):                   # sustitucion hacia atras
            self.M[i] = (d[i] - c[i] * self.M[i + 1]) / b[i]

    def __call__(self, x: float) -> float:
        t, y, M = self.t, self.y, self.M
        n = len(t)
        if n == 1:
            return y[0]
        i = 0
        hi = n - 2
        while i < hi and x > t[i + 1]:
            i += 1
        h = t[i + 1] - t[i]
        if h <= 0.0:
            return y[i]
        A, B = t[i + 1] - x, x - t[i]
        return (M[i] * A ** 3 + M[i + 1] * B ** 3) / (6.0 * h) \
            + (y[i] / h - M[i] * h / 6.0) * A \
            + (y[i + 1] / h - M[i + 1] * h / 6.0) * B


def _parametrizar(pts: Sequence[Sequence[float]]) -> list[float]:
    """Parametro por longitud de cuerda, normalizado a [0,1].

    Es lo que evita que un tramo largo y uno corto se recorran en el mismo
    tiempo: el parametro avanza con la distancia, no con el indice del punto.
    """
    d = [0.0]
    for a, b in zip(pts, pts[1:]):
        d.append(d[-1] + math.dist(a, b))
    total = d[-1]
    if total <= 1e-12:
        n = len(pts) - 1
        return [i / n if n else 0.0 for i in range(len(pts))]
    return [v / total for v in d]


# =========================================================================
#  CAMINOS: la forma.  q(u): [0,1] -> vector de juntas
# =========================================================================
class Camino:
    """Forma geometrica. `q_ref` es la muestra anterior: fija la rama de la IK."""

    nombre = 'camino'

    def q(self, u: float, q_ref: Sequence[float]) -> list[float]:
        raise NotImplementedError

    def via(self) -> list[tuple[float, float, float]]:
        """Puntos que hay que marcar en RViz. Vacio si no los hay."""
        return []

    def __repr__(self) -> str:
        return self.nombre


class CaminoJunta(Camino):
    """Interpolacion en el espacio articular: cada junta va de q0_i a q1_i.

    El unico camino sin cinematica inversa. Nunca falla, nunca cruza una
    singularidad y es el mas barato; a cambio la punta no describe nada
    reconocible. Es el que se usa para ir a `home` o para recolocar el brazo.
    """

    nombre = 'junta'

    def __init__(self, q0: Sequence[float], q1: Sequence[float]):
        self.q0, self.q1 = [float(v) for v in q0], [float(v) for v in q1]

    def q(self, u, q_ref):
        u = _lim(u)
        return [a + u * (b - a) for a, b in zip(self.q0, self.q1)]


class _CaminoCartesiano(Camino):
    """Base de los caminos que resuelven IK: guarda modelo y rama del codo."""

    def __init__(self, modelo: ScaraModel, codo: str = 'nearest'):
        self.modelo = modelo
        self.codo = codo

    def p(self, u: float) -> tuple[float, float, float]:
        raise NotImplementedError

    def q(self, u, q_ref):
        x, y, z = self.p(_lim(u))
        try:
            return self.modelo.ik(x, y, z, elbow=self.codo, q_ref=list(q_ref))
        except IKError as e:
            raise IKError(f'{self.nombre}: en u={u:.3f} el punto '
                          f'({x:.3f}, {y:.3f}, {z:.3f}) no vale -> {e}') from None


class CaminoRecta(_CaminoCartesiano):
    """Linea recta en cartesiano entre p0 y p1.

    La punta va recta; las juntas, no. Es el camino que hace falta cuando la
    herramienta tiene que seguir un borde y el que puede fallar a mitad si la
    recta se sale del anillo alcanzable: por eso la trayectoria se genera
    entera antes de mover nada.
    """

    nombre = 'recta'

    def __init__(self, modelo, p0, p1, codo='nearest'):
        super().__init__(modelo, codo)
        self.p0, self.p1 = tuple(map(float, p0)), tuple(map(float, p1))

    def p(self, u):
        return tuple(a + u * (b - a) for a, b in zip(self.p0, self.p1))

    def via(self):
        return [self.p0, self.p1]


class CaminoArco(_CaminoCartesiano):
    """Arco de circunferencia en XY; si z0 != z1 es una helice.

    Con a1 - a0 = +-2*pi*n son n vueltas completas. Ojo: un circulo alrededor
    del origen no cabe, porque q1 solo llega a +-120 grados; hay que centrarlo
    lejos del eje.
    """

    nombre = 'arco'

    def __init__(self, modelo, centro, radio, a0, a1, z0, z1=None, codo='nearest'):
        super().__init__(modelo, codo)
        self.cx, self.cy = float(centro[0]), float(centro[1])
        self.radio = float(radio)
        self.a0, self.a1 = float(a0), float(a1)
        self.z0 = float(z0)
        self.z1 = self.z0 if z1 is None else float(z1)

    def p(self, u):
        a = self.a0 + u * (self.a1 - self.a0)
        return (self.cx + self.radio * math.cos(a),
                self.cy + self.radio * math.sin(a),
                self.z0 + u * (self.z1 - self.z0))

    def via(self):
        return [self.p(0.0), self.p(0.5), self.p(1.0)]

    @classmethod
    def circulo(cls, modelo, centro, radio, z, vueltas=1.0, sentido=1.0, codo='nearest'):
        a0 = 0.0
        return cls(modelo, centro, radio, a0, a0 + sentido * TAU * vueltas, z, z, codo)

    @classmethod
    def por_tres(cls, modelo, p0, pm, p1, codo='nearest'):
        """El arco que pasa por tres puntos: la interpolacion circular clasica.

        Se saca el circuncentro de los tres en XY y se elige el sentido de giro
        que deja pm dentro del arco. z se interpola linealmente de p0 a p1.
        """
        (x1, y1), (x2, y2), (x3, y3) = p0[:2], pm[:2], p1[:2]
        d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        if abs(d) < 1e-12:
            raise IKError('los tres puntos del arco estan alineados: usa una recta')
        s1, s2, s3 = x1 * x1 + y1 * y1, x2 * x2 + y2 * y2, x3 * x3 + y3 * y3
        cx = (s1 * (y2 - y3) + s2 * (y3 - y1) + s3 * (y1 - y2)) / d
        cy = (s1 * (x3 - x2) + s2 * (x1 - x3) + s3 * (x2 - x1)) / d
        r = math.hypot(x1 - cx, y1 - cy)
        a0 = math.atan2(y1 - cy, x1 - cx)
        am = (math.atan2(y2 - cy, x2 - cx) - a0) % TAU
        a3 = (math.atan2(y3 - cy, x3 - cx) - a0) % TAU
        barrido = a3 if am < a3 else a3 - TAU      # el sentido que contiene a pm
        z0 = float(p0[2]) if len(p0) > 2 else 0.0
        z1 = float(p1[2]) if len(p1) > 2 else z0
        return cls(modelo, (cx, cy), r, a0, a0 + barrido, z0, z1, codo)


class CaminoPoligonal(_CaminoCartesiano):
    """Segmentos rectos encadenados por N puntos via, por longitud de cuerda.

    Pasa exactamente por cada punto, pero con una esquina viva en cada uno: la
    velocidad cambia de direccion de golpe. Es el camino honesto cuando los
    puntos son posiciones que hay que tocar (soldar, taladrar).
    """

    nombre = 'poligonal'

    def __init__(self, modelo, puntos, codo='nearest'):
        if len(puntos) < 2:
            raise ValueError('la poligonal necesita al menos dos puntos')
        self.pts = [tuple(map(float, p)) for p in puntos]
        self.u = _parametrizar(self.pts)
        super().__init__(modelo, codo)

    def p(self, u):
        i = 0
        while i < len(self.u) - 2 and u > self.u[i + 1]:
            i += 1
        tramo = self.u[i + 1] - self.u[i]
        f = 0.0 if tramo <= 0.0 else (u - self.u[i]) / tramo
        return tuple(a + f * (b - a) for a, b in zip(self.pts[i], self.pts[i + 1]))

    def via(self):
        return list(self.pts)


class CaminoSpline(_CaminoCartesiano):
    """Spline cubica natural por N puntos via, en cartesiano.

    Pasa por todos los puntos como la poligonal, pero sin esquinas: la
    velocidad y la aceleracion de la punta son continuas. Es la que se usa para
    un contorno suave. A cambio se sale del segmento recto entre puntos, asi
    que no sirve si hay un obstaculo pegado a la linea.
    """

    nombre = 'spline'

    def __init__(self, modelo, puntos, codo='nearest'):
        if len(puntos) < 2:
            raise ValueError('la spline necesita al menos dos puntos')
        self.pts = [tuple(map(float, p)) for p in puntos]
        self.u = _parametrizar(self.pts)
        self.sx = _Spline1D(self.u, [p[0] for p in self.pts])
        self.sy = _Spline1D(self.u, [p[1] for p in self.pts])
        self.sz = _Spline1D(self.u, [p[2] for p in self.pts])
        super().__init__(modelo, codo)

    def p(self, u):
        return (self.sx(u), self.sy(u), self.sz(u))

    def via(self):
        return list(self.pts)


class CaminoSplineJunta(Camino):
    """La misma spline cubica natural, pero sobre las juntas y no sobre la punta.

    Sin IK: se interpolan directamente los N vectores articulares. Es el
    equivalente de lo que hace un `joint_trajectory_controller` con varios
    puntos, y no puede fallar por alcance.
    """

    nombre = 'spline_junta'

    def __init__(self, qs: Sequence[Sequence[float]]):
        if len(qs) < 2:
            raise ValueError('la spline articular necesita al menos dos poses')
        self.qs = [[float(v) for v in q] for q in qs]
        self.u = _parametrizar(self.qs)
        self.sp = [_Spline1D(self.u, [q[i] for q in self.qs])
                   for i in range(len(self.qs[0]))]

    def q(self, u, q_ref):
        u = _lim(u)
        return [s(u) for s in self.sp]


CAMINOS = {
    'junta': CaminoJunta,
    'recta': CaminoRecta,
    'arco': CaminoArco,
    'poligonal': CaminoPoligonal,
    'spline': CaminoSpline,
    'spline_junta': CaminoSplineJunta,
}


# =========================================================================
#  TRAYECTORIA: camino + perfil + duracion
# =========================================================================
class Trayectoria:
    """Un camino recorrido con un perfil en T segundos.

    La geometria se tabula ENTERA en el constructor, antes de mover nada. Dos
    razones:

    1. Si la IK falla en el punto 300 de 400, se sabe aqui y el brazo no se ha
       movido todavia. Una recta que se sale del alcance a mitad seria, si no,
       un robot parado a medio camino con un error por detras.
    2. La tabla se recorre con q_ref encadenado, asi que la rama del codo no
       salta de una muestra a la siguiente.

    Despues el muestreo es solo interpolar: el perfil dice que u toca en cada t.
    """

    def __init__(self, camino: Camino, perf: Perfil, duracion: float,
                 q_ref: Sequence[float], muestras: int = 400,
                 salto_max: float = 0.30):
        self.camino = camino
        self.perfil = perf
        self.duracion = max(float(duracion), 1e-3)
        self.muestras = int(muestras)

        qr = list(q_ref)
        self.tabla: list[list[float]] = []
        for i in range(self.muestras + 1):
            qr = camino.q(i / self.muestras, qr)
            self.tabla.append(list(qr))
        self._sin_saltos(salto_max)

        # dq/du por diferencias centradas: es lo que da la velocidad articular
        # sin depender de la forma del camino.
        n, tab = self.muestras, self.tabla
        self.dtabla = []
        for i in range(n + 1):
            a = tab[max(i - 1, 0)]
            b = tab[min(i + 1, n)]
            k = n / float(min(i + 1, n) - max(i - 1, 0))
            self.dtabla.append([(vb - va) * k for va, vb in zip(a, b)])

    def _sin_saltos(self, salto_max: float) -> None:
        """Se planta si la IK ha cambiado de rama a mitad del camino.

        Con dos soluciones posibles (codo arriba y codo abajo), `model.ik` se
        queda con las que caben en los limites. Si la rama que se venia
        siguiendo se sale -por ejemplo el codo topa con q2_max a mitad de un
        circulo-, la IK devuelve la otra sin avisar, y entre dos muestras
        seguidas el codo se da la vuelta entera: varios radianes en 2.5 ms.

        Gazebo intentaria seguirlo y el brazo real tambien. Asi que eso no se
        manda: se levanta IKError con el punto exacto y quien llama decide
        -`planificar` prueba con la otra rama, el nodo lo dice y no se mueve-.
        """
        peor, donde, junta = 0.0, 0, 0
        for i in range(self.muestras):
            for j, (a, b) in enumerate(zip(self.tabla[i], self.tabla[i + 1])):
                d = abs(b - a)
                if d > peor:
                    peor, donde, junta = d, i, j
        if peor > salto_max:
            u = donde / self.muestras
            raise IKError(
                f'{self.camino.nombre}: cambio de rama en u={u:.3f} '
                f'(la junta {junta + 1} salta {peor:.2f} de golpe). El codo '
                f'llega a un tope y la IK se pasa a la otra solucion: prueba '
                f'con codo=up o codo=down, o acerca el camino al centro del '
                f'anillo alcanzable')

    # ------------------------------------------------------------- geometria
    def picos(self) -> list[float]:
        """max |dq_j/du| por junta: el recorrido que exige el camino."""
        return [max(abs(f[j]) for f in self.dtabla) for j in range(len(self.tabla[0]))]

    def _en(self, u: float) -> tuple[list[float], list[float]]:
        x = _lim(u) * self.muestras
        i = min(int(x), self.muestras - 1)
        f = x - i
        qa, qb = self.tabla[i], self.tabla[i + 1]
        da, db = self.dtabla[i], self.dtabla[i + 1]
        return ([a + f * (b - a) for a, b in zip(qa, qb)],
                [a + f * (b - a) for a, b in zip(da, db)])

    # --------------------------------------------------------------- muestreo
    def muestra(self, t: float) -> tuple[list[float], list[float]]:
        """(q, qd) en el instante t. Fuera de [0,T] devuelve los extremos."""
        tn = _lim(t / self.duracion)
        u = self.perfil.s(tn)
        du = self.perfil.ds(tn) / self.duracion
        q, dq = self._en(u)
        return q, [v * du for v in dq]

    def recorrer(self, dt: float) -> Iterator[tuple[float, list[float], list[float]]]:
        """Genera (t, q, qd) cada dt segundos, incluido el instante final."""
        n = max(int(round(self.duracion / dt)), 1)
        for i in range(n + 1):
            t = min(i * dt, self.duracion)
            q, qd = self.muestra(t)
            yield t, q, qd

    def fin(self) -> list[float]:
        return list(self.tabla[-1])

    def __repr__(self):
        return (f'{self.camino.nombre}+{self.perfil.nombre} '
                f'T={self.duracion:.2f}s')


def duracion_auto(camino: Camino, perf: Perfil, q_ref: Sequence[float],
                  max_vel: Sequence[float], t_min: float = 0.5,
                  muestras: int = 400) -> tuple[float, Trayectoria]:
    """La duracion mas corta que no pasa del max_vel de ninguna junta.

    La velocidad de la junta j es  qd_j(t) = (dq_j/du) * ds/dt, y su pico vale
    pico_j * kv / T. Igualando al techo y quedandose con la junta mas exigente:

        T = max_j ( pico_j * kv / max_vel_j )

    Devuelve tambien la trayectoria ya construida, porque tabular la geometria
    es justo lo que hace falta para conocer los picos y no merece hacerse dos
    veces.
    """
    tr = Trayectoria(camino, perf, 1.0, q_ref, muestras)
    picos = tr.picos()
    t = t_min
    for p, vmax in zip(picos, max_vel):
        if vmax > 1e-9:
            t = max(t, p * perf.kv / vmax)
    tr.duracion = t
    return t, tr


def planificar(camino: Camino, perf: Perfil, q_ref: Sequence[float],
               max_vel: Sequence[float], duracion: float | None = None,
               codos: Sequence[str] = ('nearest', 'up', 'down'),
               t_min: float = 0.5, muestras: int = 400) -> Trayectoria:
    """Construye la trayectoria eligiendo la rama del codo que aguanta entera.

    Es el unico sitio donde se decide entre codo arriba y codo abajo, y se
    decide mirando el camino COMPLETO, no punto a punto: una rama que vale al
    principio puede topar a mitad ([`Trayectoria._sin_saltos`]). Se prueban por
    orden las de `codos` y se devuelve la primera que llega al final sin saltos.

    Con `duracion=None` la duracion la pone `duracion_auto` a partir del
    max_vel del YAML; con un numero, se respeta tal cual y solo se avisa si se
    pasa de velocidad.

    Levanta IKError -con el motivo de cada intento- si ninguna rama vale.
    """
    fijo = getattr(camino, 'codo', None) is None       # junta y spline_junta
    intentos = ['-'] if fijo else list(codos)
    fallos = []
    for c in intentos:
        if not fijo:
            camino.codo = c
        try:
            tr = Trayectoria(camino, perf, duracion or 1.0, q_ref, muestras)
        except IKError as e:
            fallos.append(f'codo={c}: {e}')
            continue
        if duracion is None:
            t = t_min
            for pico, vmax in zip(tr.picos(), max_vel):
                if vmax > 1e-9:
                    t = max(t, pico * perf.kv / vmax)
            tr.duracion = t
        return tr
    raise IKError(f'ninguna rama del codo sirve para este camino. ' + ' | '.join(fallos))
