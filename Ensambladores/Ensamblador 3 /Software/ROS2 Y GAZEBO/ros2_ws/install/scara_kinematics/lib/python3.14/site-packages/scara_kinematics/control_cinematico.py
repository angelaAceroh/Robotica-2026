"""Control cinematico del SCARA de la mesa: el lazo del Jacobiano, cerrado con
lo que MIDE el robot.

    qd = J(q)^-1 * (xd_dot + K * (xd - FK(q)))        q = lo medido, ahora

`ruta.py` hace la misma cuenta, pero sobre el modelo: integra la trayectoria
entera antes de mover nada y luego se reproduce como posiciones. Aqui la cuenta
se hace en cada tic con las juntas medidas, y lo que sale es una VELOCIDAD de
junta, que es lo que se le manda a Gazebo (juntas por velocidad, ver
launch/control.launch.py). Si las juntas siguen la velocidad pedida,

    e_dot = -K * e     ->     e(t) = e0 * exp(-K * t)

el error cae al 37 % en tau = 1/K segundos, y en linea recta hacia la consigna.
Es lo que se mide en Gazebo (CONTROL.md).

Este modulo no sabe nada de ROS: la ley, las referencias (xd(t), xd_dot(t)),
las comprobaciones de antes de mover y un simulador de planta ideal para los
tests. El nodo es `control_nodo.py`.

Unidades, como en ruta.py: mm | s | rad. d3 en mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from scara_kinematics import ruta as R
from scara_kinematics import trayectoria as T

# =========================================================
# 1. PARAMETROS DEL CONTROL
# =========================================================
K_DEF = 5.0              # ganancia [1/s] -> tau = 0.2 s
QDOT_MAX = R.QDOT_MAX    # [1.5 rad/s, 1.5 rad/s, 8 mm/s]: los del brazo real

# Cerca del brazo estirado el Jacobiano no se invierte bien. Por debajo de este
# |sin codo| (5.7 deg) se usan minimos cuadrados amortiguados: el brazo deja de
# seguir la consigna exacta en vez de pedir velocidades enormes. El amortiguamiento
# crece de 0 (en SIN_DLS) a LAMBDA_MAX (brazo estirado del todo).
SIN_DLS = 0.10
LAMBDA_MAX = 25.0        # mm/rad, ~0.1 * a2

# Las referencias se planifican al 80 % del techo de cada junta: el resto queda
# para que K*e pueda corregir sin que la saturacion se coma el feed-forward.
MARGEN_VEL = 0.8

# Una junta a menos de esto de su tope no se deja seguir hacia el
TOL_TOPE = (1e-3, 1e-3, 0.05)    # rad, rad, mm


class ControlError(ValueError):
    """La tarea no se puede hacer con este robot. Se sabe antes de mover nada."""


# =========================================================
# 2. LA LEY DE CONTROL
# =========================================================

@dataclass
class Salida:
    qd: np.ndarray           # lo que se manda [rad/s, rad/s, mm/s]
    p: np.ndarray            # la punta medida, FK(q) [mm]
    e: np.ndarray            # xd - p [mm]
    sat: float               # max |qd_j| / qd_max_j ANTES de recortar (> 1: se recorto)
    amort: float             # lambda de los minimos cuadrados (0: inversa exacta)
    topes: tuple             # juntas frenadas contra su tope


class Ley:
    """qd = J^-1 (xd_dot + K e), con las tres protecciones de antes de mandar.

    1. Singularidad: minimos cuadrados amortiguados en el plano (el husillo
       no entra: su fila del Jacobiano es [0, 0, z_dir] y nunca es singular).
    2. Topes: una junta en su tope no se deja seguir hacia el.
    3. Saturacion: si alguna junta pasa de su techo, se escalan LAS TRES por el
       mismo factor. La punta va mas despacio pero en la misma direccion; si se
       recortara cada junta por su lado, la punta se torceria.
    """

    def __init__(self, robot: R.Robot = R.ROBOT, k=K_DEF, qdot_max=QDOT_MAX,
                 sin_dls: float = SIN_DLS, lambda_max: float = LAMBDA_MAX):
        self.robot = robot
        self.k = k
        self.qdot_max = np.asarray(qdot_max, dtype=float)
        self.sin_dls = float(sin_dls)
        self.lambda_max = float(lambda_max)

    @property
    def k(self) -> np.ndarray:
        return self._k

    @k.setter
    def k(self, k):
        k = np.broadcast_to(np.asarray(k, dtype=float), (3,)).copy()
        if np.any(k < 0.0):
            raise ValueError(f'la ganancia no puede ser negativa: {k}')
        self._k = k

    def inversa(self, q, u) -> tuple[np.ndarray, float]:
        """qd tal que J(q) qd = u (o lo mas cerca, si esta casi estirado)."""
        r = self.robot
        J2 = R.jacobiano_scara_rrp(q, r)[:2, :2]
        s = r.sin_codo(q)
        if s >= self.sin_dls:
            lam = 0.0
            q12 = np.linalg.solve(J2, u[:2])
        else:
            lam = self.lambda_max * math.sqrt(1.0 - (s / self.sin_dls) ** 2)
            q12 = J2.T @ np.linalg.solve(J2 @ J2.T + lam * lam * np.eye(2), u[:2])
        return np.array([q12[0], q12[1], u[2] / r.z_dir]), lam

    def paso(self, q, pd, vd) -> Salida:
        q = np.asarray(q, dtype=float)
        p = R.fk_scara_rrp(q, self.robot)
        e = np.asarray(pd, dtype=float) - p
        qd, lam = self.inversa(q, np.asarray(vd, dtype=float) + self.k * e)

        topes = []
        for i, (v, (lo, hi), tol) in enumerate(zip(q, self.robot.lim, TOL_TOPE)):
            if (v <= lo + tol and qd[i] < 0.0) or (v >= hi - tol and qd[i] > 0.0):
                qd[i] = 0.0
                topes.append(R.NOMBRES[i])

        sat = float(np.max(np.abs(qd) / self.qdot_max))
        if sat > 1.0:
            qd = qd / sat
        return Salida(qd, p, e, sat, lam, tuple(topes))


def rama(q, robot: R.Robot = R.ROBOT) -> str:
    """En que rama del codo esta el brazo: 'down' (la del reposo) o 'up'."""
    return 'down' if q[1] - robot.b1 + robot.b2 < 0.0 else 'up'


def alcance_plano(X, Y, robot: R.Robot = R.ROBOT) -> dict:
    """Donde llega la punta en planta, punto a punto (X, Y en mm, arrays).

    Devuelve {'down': bool, 'up': bool}: con esa rama del codo la IK sale, q1 y
    q2 caben en sus topes y el brazo no va casi estirado. Es lo mismo que mira
    `revisar` con ik_scara_rrp, sin z y para una rejilla entera de una vez.
    """
    X, Y = np.asarray(X, dtype=float), np.asarray(Y, dtype=float)
    c = (X * X + Y * Y - robot.a1 ** 2 - robot.a2 ** 2) / (2.0 * robot.a1 * robot.a2)
    dentro = np.abs(c) <= 1.0
    g0 = np.arccos(np.clip(c, -1.0, 1.0))
    (l1, h1), (l2, h2) = robot.lim[0], robot.lim[1]
    out = {}
    for codo, g in (('up', g0), ('down', -g0)):
        fi = np.arctan2(Y, X) - np.arctan2(robot.a2 * np.sin(g), robot.a1 + robot.a2 * np.cos(g))
        q1 = np.angle(np.exp(1j * (fi - robot.b1)))
        q2 = np.angle(np.exp(1j * (g + robot.b1 - robot.b2)))
        out[codo] = (dentro & (q1 >= l1) & (q1 <= h1) & (q2 >= l2) & (q2 <= h2)
                     & (np.abs(np.sin(g)) >= R.SIN_CODO_MIN))
    return out


# =========================================================
# 3. LAS REFERENCIAS: xd(t) y xd_dot(t)
# =========================================================

class Referencia:
    """Lo que tiene que hacer la punta. Pasado `duracion`, se queda en el final."""

    nombre = 'referencia'
    tipo = 'referencia'
    duracion = 0.0

    def en(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """(xd, xd_dot) en el instante t, en mm y mm/s."""
        raise NotImplementedError

    def inicio(self) -> np.ndarray:
        return self.en(0.0)[0]

    def fin(self) -> np.ndarray:
        return self.en(self.duracion)[0]

    def estirar(self, f: float):
        """Mas lenta por un factor f (> 1): mismo camino, velocidades / f."""
        self.duracion *= f

    def muestras(self, n: int = 400) -> np.ndarray:
        return np.array([self.en(t)[0] for t in np.linspace(0.0, self.duracion, n + 1)])

    def __repr__(self) -> str:
        return f'{self.nombre} ({self.duracion:.2f} s)'


class Fija(Referencia):
    """Un punto quieto. Desde otro sitio es un escalon: la regulacion pura."""

    nombre = 'el punto'
    tipo = 'salto'

    def __init__(self, p):
        self.p = np.asarray(p, dtype=float).copy()

    def en(self, t):
        return self.p.copy(), np.zeros(3)


class Recta(Referencia):
    """De pi a pf en linea recta, con la ley horaria de un perfil."""

    nombre = 'recta'

    def __init__(self, pi, pf, v: float, perf: T.Perfil | None = None, tipo: str = 'recta'):
        self.pi = np.asarray(pi, dtype=float).copy()
        self.pf = np.asarray(pf, dtype=float).copy()
        self.d = self.pf - self.pi
        self.perfil = perf or T.perfil('trapecio')
        self.tipo = tipo
        D = float(np.linalg.norm(self.d))
        if v <= 0.0:
            raise ControlError(f'la velocidad tiene que ser positiva: {v}')
        # v es el PICO de la velocidad de la punta: con un recorrido D en T
        # segundos el pico vale kv*D/T.
        self.duracion = self.perfil.kv * D / v if D > 1e-9 else 0.0

    def en(self, t):
        if self.duracion <= 0.0:
            return self.pf.copy(), np.zeros(3)
        u = min(max(t / self.duracion, 0.0), 1.0)
        return (self.pi + self.perfil.s(u) * self.d,
                self.perfil.ds(u) / self.duracion * self.d)


class Circulo(Referencia):
    """Un circulo horizontal. El angulo lleva la ley horaria: sale y llega parado."""

    nombre = 'circulo'
    tipo = 'circulo'

    def __init__(self, centro, radio: float, z: float, v: float, vueltas: float = 1.0,
                 perf: T.Perfil | None = None, ang0: float = 0.0):
        if radio <= 0.0 or v <= 0.0 or vueltas <= 0.0:
            raise ControlError('radio, velocidad y vueltas tienen que ser positivos')
        self.c = np.array([float(centro[0]), float(centro[1])])
        self.r, self.z = float(radio), float(z)
        self.vueltas, self.ang0 = float(vueltas), float(ang0)
        self.perfil = perf or T.perfil('trapecio')
        self.duracion = self.perfil.kv * 2.0 * math.pi * self.r * self.vueltas / v

    def en(self, t):
        u = min(max(t / self.duracion, 0.0), 1.0)
        giro = 2.0 * math.pi * self.vueltas
        th = self.ang0 + giro * self.perfil.s(u)
        w = giro * self.perfil.ds(u) / self.duracion
        c, s = math.cos(th), math.sin(th)
        return (np.array([self.c[0] + self.r * c, self.c[1] + self.r * s, self.z]),
                np.array([-self.r * s * w, self.r * c * w, 0.0]))


class Articular(Referencia):
    """La aproximacion: las juntas de q0 a q1 en linea recta (como `tray` y
    `ruta_nodo`), pero dada como referencia de la punta, xd = FK(q(t)) y
    xd_dot = J(q) qd(t), para que la siga el mismo control.

    No necesita IK, asi que no puede cambiar de rama. La duracion sale del
    techo de la junta mas exigente.
    """

    nombre = 'aproximacion'
    tipo = 'aproximacion'

    def __init__(self, q0, q1, robot: R.Robot = R.ROBOT, qdot_max=QDOT_MAX,
                 margen: float = MARGEN_VEL, t_min: float = 0.5):
        self.q0 = np.asarray(q0, dtype=float).copy()
        self.dq = np.asarray(q1, dtype=float) - self.q0
        self.robot = robot
        self.perfil = T.perfil('quintico')
        techo = np.asarray(qdot_max, dtype=float) * margen
        self.duracion = max(t_min, float(np.max(self.perfil.kv * np.abs(self.dq) / techo)))

    def q(self, t) -> tuple[np.ndarray, np.ndarray]:
        u = min(max(t / self.duracion, 0.0), 1.0)
        return (self.q0 + self.perfil.s(u) * self.dq,
                self.perfil.ds(u) / self.duracion * self.dq)

    def en(self, t):
        q, qd = self.q(t)
        return (R.fk_scara_rrp(q, self.robot),
                R.jacobiano_scara_rrp(q, self.robot) @ qd)


class Secuencia(Referencia):
    """Varias referencias, una detras de otra. Es lo que ejecuta el nodo."""

    def __init__(self, nombre: str, partes: list[Referencia]):
        self.nombre = nombre
        self.tipo = nombre
        self.partes = [p for p in partes if p.duracion > 0.0] or partes[-1:]

    @property
    def duracion(self) -> float:
        return sum(p.duracion for p in self.partes)

    def estirar(self, f: float):
        for p in self.partes:
            p.estirar(f)

    def donde(self, t: float) -> tuple[int, float]:
        """(indice de la parte, t dentro de ella)."""
        for i, p in enumerate(self.partes):
            if t < p.duracion or i == len(self.partes) - 1:
                return i, t
            t -= p.duracion
        return len(self.partes) - 1, t

    def en(self, t):
        i, tl = self.donde(t)
        return self.partes[i].en(tl)

    def muestras(self, n: int = 400) -> np.ndarray:
        cada = max(20, n // max(len(self.partes), 1))
        return np.vstack([p.muestras(cada) for p in self.partes])


# =========================================================
# 4. ANTES DE MOVER: que quepa y a que velocidad
# =========================================================

def revisar(ref: Referencia, robot: R.Robot, codo: str, qdot_max=QDOT_MAX,
            n: int = 400) -> float:
    """Recorre la referencia con la IK en la rama `codo`.

    Levanta ControlError si algun punto no se alcanza, se sale de los topes o
    pasa casi estirado. Devuelve el pico de max_j |qd_j| / qd_max_j que pide el
    feed-forward: > 1 = alguna junta no da esa velocidad.
    """
    qdot_max = np.asarray(qdot_max, dtype=float)
    peor = 0.0
    for t in np.linspace(0.0, ref.duracion, n + 1 if ref.duracion > 0.0 else 1):
        pd, vd = ref.en(t)
        donde = f'{ref.nombre}, t = {t:.2f} s' if ref.duracion > 0.0 else ref.nombre
        try:
            q = R.ik_scara_rrp(pd, robot, codo)
        except R.RutaError as e:
            raise ControlError(f'{donde}: {e}') from None
        if robot.sin_codo(q) < R.SIN_CODO_MIN:
            raise ControlError(
                f'{donde}: el brazo pasa casi estirado en '
                f'[{pd[0]:.0f}, {pd[1]:.0f}, {pd[2]:.0f}] mm (|sin codo| = '
                f'{robot.sin_codo(q):.3f})')
        qd = np.linalg.solve(R.jacobiano_scara_rrp(q, robot), vd)
        peor = max(peor, float(np.max(np.abs(qd) / qdot_max)))
    return peor


def a_tiempo(ref: Referencia, robot: R.Robot, codo: str, qdot_max=QDOT_MAX,
             margen: float = MARGEN_VEL) -> float:
    """Revisa la referencia y, si pide mas del margen a alguna junta, la estira.

    Devuelve el factor por el que se ha estirado (1.0 = nada).
    """
    pico = revisar(ref, robot, codo, qdot_max)
    if pico <= margen:
        return 1.0
    f = pico / margen
    ref.estirar(f)
    return f


# =========================================================
# 5. MEDIR: la constante de tiempo y un simulador ideal
# =========================================================

def tau(t, e) -> float | None:
    """Lo que tarda |e| en caer al 37 % (1/e) de lo que valia en t[0].

    Interpola entre muestras. None si no llega a caer tanto.
    """
    t, e = np.asarray(t, dtype=float), np.asarray(e, dtype=float)
    if len(e) < 2 or e[0] <= 0.0:
        return None
    objetivo = e[0] / math.e
    for i in range(1, len(e)):
        if e[i] <= objetivo:
            w = (e[i - 1] - objetivo) / (e[i - 1] - e[i]) if e[i - 1] != e[i] else 1.0
            return float(t[i - 1] + w * (t[i] - t[i - 1]) - t[0])
    return None


def simular(ref: Referencia, q0, ley: Ley, dt: float = 0.01, retraso: int = 0,
            t_extra: float = 0.0) -> dict:
    """La planta ideal del control cinematico: cada junta hace exactamente la
    velocidad que se le manda (q += qd*dt), con `retraso` tics de retraso.

    Es lo que supone la teoria; Gazebo con las juntas por velocidad se le
    parece mucho, y el brazo real menos. Devuelve arrays: t, q, p, pd, e, qd, sat.
    """
    q = np.asarray(q0, dtype=float).copy()
    cola = [np.zeros(3)] * retraso
    n = int(math.ceil((ref.duracion + t_extra) / dt))
    h = {k: [] for k in ('t', 'q', 'p', 'pd', 'e', 'qd', 'sat')}
    for i in range(n + 1):
        t = i * dt
        pd, vd = ref.en(t)
        s = ley.paso(q, pd, vd)
        cola.append(s.qd)
        qd = cola.pop(0)
        for k, v in (('t', t), ('q', q.copy()), ('p', s.p), ('pd', pd), ('e', s.e),
                     ('qd', qd), ('sat', s.sat)):
            h[k].append(v)
        q = q + qd * dt
    return {k: np.array(v) for k, v in h.items()}
