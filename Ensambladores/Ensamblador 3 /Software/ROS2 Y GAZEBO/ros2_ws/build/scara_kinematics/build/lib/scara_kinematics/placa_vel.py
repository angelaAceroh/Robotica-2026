"""Lo que hablan el PC y main_control.py (la placa), y las cuentas que faltan
para llevar el control cinematico al brazo de verdad.

Biblioteca sin ROS. La usan el nodo `puente_placa`, la ventana
`control_placa_gui`, launch/control_placa.launch.py y los tests.

«Motor» es lo que cuenta main_control.py, en las unidades de los programas de
Sergio: Alvin y Simon en grados de su junta, Teodoro en mm. Las POSICIONES se
pasan de junta a motor con alcance.Calibracion (a_motor / a_junta), con los
numeros de config/micropython.yaml. Aqui va lo de las VELOCIDADES:

    motor/s = signo * reduccion * grados/s de la junta        Alvin, Simon
            = signo * (mm/s) * 360 / paso_husillo_mm          Teodoro

(sin el `cero`: una velocidad no depende de donde este el cero). Y los limites
de junta que salen de los recorridos de los motores, que son los que tiene que
respetar control_nodo en el brazo de verdad.
"""
from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass

import numpy as np

from scara_kinematics import alcance as A
from scara_kinematics import ruta as R

MOTORES = A.MOTORES
UNIDADES = ('°', '°', 'mm')          # las de main_control.py: Alvin, Simon, Teodoro
# time.ticks_ms() de MicroPython da la vuelta aqui (2^30 ms, unos 12.4 dias)
TICKS_PERIODO = 1 << 30


class ChecksumMalo(ValueError):
    """La linea ha llegado estropeada por el USB."""


def checksum(texto: str) -> str:
    """Suma de los bytes, modulo 256, en hexadecimal: la misma cuenta que la placa."""
    return '%02X' % (sum(texto.encode()) & 0xFF)


def con_checksum(texto: str) -> str:
    return f'{texto} *{checksum(texto)}'


def linea_v(vel) -> str:
    """Las tres velocidades (grados/s, grados/s, mm/s) como las lee main_control.py."""
    return con_checksum('V ' + ' '.join(f'{v:.3f}' for v in vel))


@dataclass
class Muestra:
    """Una P: el reloj de la placa, donde esta cada motor y si esta activo."""
    ms: int
    pos: list
    activo: list


@dataclass
class Diagnostico:
    """Una D: lo que va cada motor por detras de su consigna (grados o mm) y su PWM."""
    ms: int
    seguimiento: list
    pwm: list


def leer(linea: str) -> Muestra | Diagnostico | None:
    """Una P o una D de la placa. None si es otra cosa; ChecksumMalo si no cuadra."""
    if not linea or linea[0] not in 'PD' or len(linea) < 2 or linea[1] != ' ':
        return None
    if ' *' not in linea:
        raise ChecksumMalo(f'sin checksum: {linea!r}')
    cuerpo, llegado = linea.rsplit(' *', 1)
    if checksum(cuerpo) != llegado.strip().upper():
        raise ChecksumMalo(f'no cuadra: {linea!r}')
    partes = cuerpo.split()
    try:
        if partes[0] == 'P' and len(partes) == 6 and len(partes[5]) == 3:
            return Muestra(int(partes[1]), [float(v) for v in partes[2:5]],
                           [c == 'A' for c in partes[5]])
        if partes[0] == 'D' and len(partes) == 8:
            return Diagnostico(int(partes[1]), [float(v) for v in partes[2:5]],
                               [float(v) for v in partes[5:8]])
    except ValueError:
        pass
    raise ChecksumMalo(f'el checksum cuadra pero no se entiende: {linea!r}')


def leer_limites(linea: str) -> list | None:
    """'LIMITES Alvin 0 100 Simon -200 0 Teodoro -100 0' -> los seis numeros."""
    nums = re.findall(r'-?\d+(?:\.\d+)?', linea)
    return [float(v) for v in nums] if len(nums) == 6 else None


# =========================================================
# JUNTA <-> MOTOR, PARA LAS VELOCIDADES Y LOS LIMITES
# =========================================================

def vel_a_motor(qd, cal: A.Calibracion) -> list:
    """[rad/s, rad/s, mm/s] -> lo que se manda a Alvin, Simon y Teodoro.

    Teodoro sale 0 si falta el paso del husillo: no se sabe cuanto girarlo.
    """
    a = cal.signo[0] * cal.reduccion[0] * math.degrees(qd[0])
    s = cal.signo[1] * cal.reduccion[1] * math.degrees(qd[1])
    t = (cal.signo[2] * qd[2] * 360.0 / cal.paso_husillo_mm
         if cal.husillo_medido else 0.0)
    return [a, s, t]


def limites_junta(cal: A.Calibracion, robot: R.Robot = R.ROBOT, margen: float = 0.5):
    """Los recorridos de los motores (menos `margen` de motor por cada lado),
    pasados a junta: ((q1 min, max) rad, (q2 min, max) rad, (d3 min, max) mm).

    Sin el paso del husillo, d3 se queda con los topes del CAD (la z del control
    va de mentira, ver puente_placa). Nunca se sale de los topes del CAD; si el
    recorrido de un motor no los cruza (la calibracion no cuadra), ValueError.
    """
    out = []
    for i in range(3):
        lo, hi = cal.limites[i]
        lo, hi = lo + margen, hi - margen
        if i == 2 and not cal.husillo_medido:
            out.append(tuple(robot.lim[2]))
            continue
        m_lo, m_hi = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        m_lo[i], m_hi[i] = lo, hi
        a, b = cal.a_junta(m_lo)[i], cal.a_junta(m_hi)[i]
        a, b = min(a, b), max(a, b)
        cad = robot.lim[i]
        lo, hi = max(a, cad[0]), min(b, cad[1])
        if lo >= hi:
            u = (lambda v: f'{v:.1f} mm') if i == 2 else (lambda v: f'{math.degrees(v):.1f}°')
            raise ValueError(
                f'{MOTORES[i]}: su recorrido pasado a junta, [{u(a)}, {u(b)}], no se cruza '
                f'con los topes del CAD, [{u(cad[0])}, {u(cad[1])}]. Revisa cero, signo, '
                f'reduccion y paso_husillo_mm en config/micropython.yaml')
        out.append((lo, hi))
    return tuple(out)


def robot_con_limites(robot: R.Robot, lim) -> R.Robot:
    return R.Robot(robot.a1, robot.b1, robot.a2, robot.b2, robot.z_ref, robot.z_dir,
                   tuple(tuple(v) for v in lim))


def zona_trabajo(robot: R.Robot, z: float = R.Z_SEGURO, n: int = 41) -> dict:
    """Donde llega la punta en planta con los limites de `robot` (los del brazo de
    verdad, de limites_junta): la caja que la contiene y el punto mas adentro.
    Si el codo puede pasar por recto, solo en la rama down (A.lado_down): el
    control no cambia de rama, y el punto mas adentro no puede ser el brazo
    estirado.

    {'caja': (x0, x1, y0, y1), 'centro': (x, y, z), 'holgura': mm hasta el borde}
    """
    (a0, a1), (b0, b1) = robot.lim[0], A.lado_down(robot, *robot.lim[1])
    d3 = (z - robot.z_ref) / robot.z_dir

    def punta(q1, q2):
        return R.fk_scara_rrp([q1, q2, d3], robot)

    Q1, Q2 = np.meshgrid(np.linspace(a0, a1, n), np.linspace(b0, b1, n))
    P = np.array([punta(u, v) for u, v in zip(Q1.ravel(), Q2.ravel())])
    t = np.linspace(0.0, 1.0, 4 * n)
    borde = np.array([punta(a0 + (a1 - a0) * s, b) for b in (b0, b1) for s in t]
                     + [punta(a, b0 + (b1 - b0) * s) for a in (a0, a1) for s in t])[:, :2]
    dist = np.array([np.min(np.linalg.norm(borde - p[:2], axis=1)) for p in P])
    k = int(np.argmax(dist))
    return {'caja': (float(P[:, 0].min()), float(P[:, 0].max()),
                     float(P[:, 1].min()), float(P[:, 1].max())),
            'centro': tuple(float(v) for v in P[k]),
            'holgura': float(dist[k])}


# =========================================================
# EL RELOJ DE LA PLACA
# =========================================================

class Reloj:
    """Pasa el reloj de la placa (ms de ticks_ms) al del PC (s).

    control_nodo calcula el error contra donde tenia que estar la punta CUANDO
    SE MIDIO (CONTROL.md §6.2), asi que /joint_states tiene que llevar la hora de
    la medida, no la de llegada. La diferencia llegada - reloj de la placa es el
    desfase mas lo que tarda la linea en cruzar el USB; su minimo en los ultimos
    `ventana` puntos es el desfase con el retraso mas corto. Si el reloj de la
    placa va hacia atras (se ha reiniciado), se empieza de cero.
    """

    def __init__(self, ventana: int = 500):
        self.ventana = ventana
        self._minimos: deque = deque()       # (n, diferencia), diferencias crecientes
        self._n = 0
        self._base = 0
        self._ultimo: int | None = None

    def __call__(self, ms: int, llegada: float) -> float:
        if self._ultimo is not None and ms < self._ultimo:
            if self._ultimo > 0.9 * TICKS_PERIODO and ms < 0.1 * TICKS_PERIODO:
                self._base += TICKS_PERIODO              # ticks_ms ha dado la vuelta
            else:
                self.__init__(self.ventana)              # la placa se ha reiniciado
        self._ultimo = ms
        t = (self._base + ms) * 1e-3
        d = llegada - t
        self._n += 1
        while self._minimos and self._minimos[-1][1] >= d:
            self._minimos.pop()
        self._minimos.append((self._n, d))
        while self._minimos[0][0] <= self._n - self.ventana:
            self._minimos.popleft()
        return t + self._minimos[0][1]
