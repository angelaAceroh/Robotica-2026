"""Modelo cinematico de un SCARA (RRP, con muneca opcional).

Los dos SCARA del workspace (UrdfR3 y scara_urdf) salen de SolidWorks, asi que
sus ejes 1 y 2 no arrancan alineados con el eje X de la base: cada junta trae un
pequeno giro fijo en el <origin> y, en scara_urdf, el codo ademas esta desplazado
lateralmente. Por eso el modelo no es el 2R de libro "L1*cos(q1)+..." sino:

    x = l1*cos(q1 + b1) + l2*cos(q1 + q2 + b2)
    y = l1*sin(q1 + b1) + l2*sin(q1 + q2 + b2)
    z = z_ref + z_dir * d3
    yaw = q1 + q2 + wrist_sign*q4 + yaw_offset        (solo si hay muneca)

donde l1/b1 son el modulo y el argumento del vector hombro->codo medidos sobre la
FK real del URDF, y l2/b2 lo mismo para codo->husillo. Con esto el error frente a
la cadena completa del URDF es < 0.15 mm (ver test/test_kinematics.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def wrap(a: float) -> float:
    """Angulo normalizado a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class IKError(Exception):
    """El punto pedido no tiene solucion valida."""


@dataclass
class ScaraModel:
    # --- nombres de las juntas, en el orden [q1, q2, d3, (q4)] ---
    joint_names: list[str]

    # --- brazo planar ---
    l1: float
    b1: float
    l2: float
    b2: float

    # --- husillo vertical: z_tcp = z_ref + z_dir * d3 ---
    z_ref: float
    z_dir: float = 1.0

    # --- limites ---
    q1_min: float = -math.pi
    q1_max: float = math.pi
    q2_min: float = -math.pi
    q2_max: float = math.pi
    d3_min: float = 0.0
    d3_max: float = 0.1

    # --- muneca (4o GDL) ---
    has_wrist: bool = False
    wrist_sign: float = -1.0
    yaw_offset: float = 0.0
    q4_min: float = -math.pi
    q4_max: float = math.pi

    # --- velocidades maximas, para interpolar ordenes suaves (rad/s, m/s) ---
    max_vel: list[float] = field(default_factory=lambda: [1.5, 1.5, 0.15, 2.0])

    # ------------------------------------------------------------------ util
    @property
    def ndof(self) -> int:
        return 4 if self.has_wrist else 3

    @property
    def r_max(self) -> float:
        return self.l1 + self.l2

    @property
    def r_min(self) -> float:
        return abs(self.l1 - self.l2)

    @property
    def z_min(self) -> float:
        return min(self.z_ref + self.z_dir * self.d3_min,
                   self.z_ref + self.z_dir * self.d3_max)

    @property
    def z_max(self) -> float:
        return max(self.z_ref + self.z_dir * self.d3_min,
                   self.z_ref + self.z_dir * self.d3_max)

    def reach(self) -> tuple[float, float]:
        """Radio alcanzable teniendo en cuenta el limite de la junta 2.

        r_min/r_max son los del 2R ideal; si q2 no llega a plegarse del todo, el
        agujero central es mayor de lo que dice |l1-l2|. Es lo que hay que usar
        para elegir puntos que la IK vaya a aceptar de verdad.
        """
        g_lo = self.q2_min + self.b2 - self.b1
        g_hi = self.q2_max + self.b2 - self.b1

        def radio(g: float) -> float:
            g = min(max(g, -math.pi), math.pi)
            return math.sqrt(self.l1 ** 2 + self.l2 ** 2
                             + 2.0 * self.l1 * self.l2 * math.cos(g))

        g_cerca = 0.0 if g_lo <= 0.0 <= g_hi else (g_lo if abs(g_lo) < abs(g_hi) else g_hi)
        g_lejos = g_lo if abs(g_lo) > abs(g_hi) else g_hi
        return radio(g_lejos), radio(g_cerca)

    def limits(self) -> list[tuple[float, float]]:
        lim = [(self.q1_min, self.q1_max), (self.q2_min, self.q2_max),
               (self.d3_min, self.d3_max)]
        if self.has_wrist:
            lim.append((self.q4_min, self.q4_max))
        return lim

    def in_limits(self, q: list[float], tol: float = 1e-6) -> bool:
        return all(lo - tol <= v <= hi + tol for v, (lo, hi) in zip(q, self.limits()))

    def clamp(self, q: list[float]) -> list[float]:
        return [min(max(v, lo), hi) for v, (lo, hi) in zip(q, self.limits())]

    # -------------------------------------------------------------- cinematica
    def fk(self, q: list[float]) -> tuple[float, float, float, float]:
        """Directa: [q1, q2, d3, (q4)] -> (x, y, z, yaw) del TCP."""
        q1, q2, d3 = q[0], q[1], q[2]
        a1 = q1 + self.b1
        a2 = q1 + q2 + self.b2
        x = self.l1 * math.cos(a1) + self.l2 * math.cos(a2)
        y = self.l1 * math.sin(a1) + self.l2 * math.sin(a2)
        z = self.z_ref + self.z_dir * d3
        if self.has_wrist:
            q4 = q[3] if len(q) > 3 else 0.0
            yaw = wrap(q1 + q2 + self.wrist_sign * q4 + self.yaw_offset)
        else:
            # sin muneca la orientacion de la herramienta no es libre: la fija el brazo
            yaw = wrap(q1 + q2 + self.yaw_offset)
        return x, y, z, yaw

    def ik_all(self, x: float, y: float, z: float,
               yaw: float | None = None) -> list[list[float]]:
        """Inversa: devuelve las (hasta 2) soluciones, codo+ primero.

        Levanta IKError si el punto queda fuera del anillo alcanzable o fuera
        del recorrido del husillo.
        """
        r = math.hypot(x, y)
        if r > self.r_max + 1e-9:
            raise IKError(f'fuera de alcance: r={r:.4f} m > r_max={self.r_max:.4f} m')
        if r < self.r_min - 1e-9:
            raise IKError(f'dentro del agujero central: r={r:.4f} m < r_min={self.r_min:.4f} m')

        d3 = self.z_dir * (z - self.z_ref)
        if not (self.d3_min - 1e-9 <= d3 <= self.d3_max + 1e-9):
            raise IKError(f'z={z:.4f} m fuera del recorrido del husillo '
                          f'[{self.z_min:.4f}, {self.z_max:.4f}] m')
        d3 = min(max(d3, self.d3_min), self.d3_max)

        cg = (r * r - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        cg = min(max(cg, -1.0), 1.0)
        gamma = math.acos(cg)                     # angulo interno del codo

        sols: list[list[float]] = []
        for g in (gamma, -gamma):
            a1 = math.atan2(y, x) - math.atan2(self.l2 * math.sin(g),
                                               self.l1 + self.l2 * math.cos(g))
            q1 = wrap(a1 - self.b1)
            q2 = wrap(g + self.b1 - self.b2)
            q = [q1, q2, d3]
            if self.has_wrist:
                target_yaw = 0.0 if yaw is None else yaw
                q4 = wrap((target_yaw - q1 - q2 - self.yaw_offset) / self.wrist_sign)
                q.append(q4)
            sols.append(q)
            if abs(gamma) < 1e-9:                 # brazo estirado: solucion unica
                break
        return sols

    def ik(self, x: float, y: float, z: float, yaw: float | None = None,
           elbow: str = 'nearest', q_ref: list[float] | None = None) -> list[float]:
        """Inversa con seleccion de rama.

        elbow: 'up' (codo+, gamma>0), 'down' (codo-), o 'nearest' (la mas cercana
        a q_ref, evitando saltos bruscos de configuracion).
        """
        sols = self.ik_all(x, y, z, yaw)
        valid = [s for s in sols if self.in_limits(s)]
        if not valid:
            det = ', '.join('[' + ', '.join(f'{v:.3f}' for v in s) + ']' for s in sols)
            raise IKError(f'las soluciones violan los limites de junta: {det}')

        if elbow == 'up':
            pick = valid[0]
        elif elbow == 'down':
            pick = valid[-1]
        else:
            if q_ref is None:
                pick = valid[0]
            else:
                def cost(s):
                    return sum(abs(wrap(a - b)) if i != 2 else abs(a - b)
                               for i, (a, b) in enumerate(zip(s, q_ref)))
                pick = min(valid, key=cost)
        return pick

    def jacobian(self, q: list[float]) -> list[list[float]]:
        """Jacobiano de la tarea (x, y, z, yaw) respecto de las juntas."""
        q1, q2 = q[0], q[1]
        a1 = q1 + self.b1
        a2 = q1 + q2 + self.b2
        s1, c1 = math.sin(a1), math.cos(a1)
        s2, c2 = math.sin(a2), math.cos(a2)
        j = [
            [-self.l1 * s1 - self.l2 * s2, -self.l2 * s2, 0.0],
            [self.l1 * c1 + self.l2 * c2, self.l2 * c2, 0.0],
            [0.0, 0.0, self.z_dir],
            [1.0, 1.0, 0.0],
        ]
        if self.has_wrist:
            for row, extra in zip(j, (0.0, 0.0, 0.0, self.wrist_sign)):
                row.append(extra)
        return j

    def manipulability(self, q: list[float]) -> float:
        """det del jacobiano planar = l1*l2*sin(gamma). ~0 => singularidad."""
        gamma = q[1] - self.b1 + self.b2
        return abs(self.l1 * self.l2 * math.sin(gamma))

    # ------------------------------------------------------------ constructores
    @staticmethod
    def from_params(p: dict) -> 'ScaraModel':
        """Construye el modelo desde un diccionario de parametros ROS."""
        return ScaraModel(
            joint_names=list(p['joint_names']),
            l1=float(p['l1']), b1=float(p['b1']),
            l2=float(p['l2']), b2=float(p['b2']),
            z_ref=float(p['z_ref']), z_dir=float(p['z_dir']),
            q1_min=float(p['q1_min']), q1_max=float(p['q1_max']),
            q2_min=float(p['q2_min']), q2_max=float(p['q2_max']),
            d3_min=float(p['d3_min']), d3_max=float(p['d3_max']),
            has_wrist=bool(p['has_wrist']), wrist_sign=float(p['wrist_sign']),
            yaw_offset=float(p['yaw_offset']),
            q4_min=float(p['q4_min']), q4_max=float(p['q4_max']),
            max_vel=[float(v) for v in p['max_vel']],
        )


# Parametros por defecto: se declaran en los nodos y se sobreescriben con el YAML.
DEFAULT_PARAMS = {
    'joint_names': ['Simon_joint', 'Teodoro_joint', 'Dave_joint'],
    'l1': 0.165, 'b1': -0.056701,
    'l2': 0.135, 'b2': 0.007065,
    'z_ref': 0.22884, 'z_dir': -1.0,
    'q1_min': -3.0, 'q1_max': 3.0,
    'q2_min': -2.5, 'q2_max': 2.5,
    'd3_min': 0.0, 'd3_max': 0.045,
    'has_wrist': False, 'wrist_sign': -1.0, 'yaw_offset': 0.0,
    'q4_min': -3.1416, 'q4_max': 3.1416,
    'max_vel': [1.5, 1.5, 0.15, 2.0],
    'z_arm': 0.24045,
}


def declare_model_params(node) -> ScaraModel:
    """Declara DEFAULT_PARAMS en el nodo y devuelve el modelo resultante."""
    vals = {}
    for k, v in DEFAULT_PARAMS.items():
        vals[k] = node.declare_parameter(k, v).value
    return ScaraModel.from_params(vals)
