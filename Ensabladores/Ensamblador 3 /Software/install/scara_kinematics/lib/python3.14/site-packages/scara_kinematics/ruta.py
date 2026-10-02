"""Trayectorias reales del SCARA de la mesa, por Jacobiano (control de velocidad).

Es el codigo guia (la perforacion P1..P6 con feed-forward + Jacobiano inverso)
llevado al robot montado en el laboratorio. La estructura es la misma, seccion a
seccion, para que se reconozca:

    1. parametros del robot      4. velocidades y control
    2. puntos de trabajo         funciones cinematicas (FK, J, IK)
    3. rutas                     seguimiento + autocorreccion, informe, CSV, graficas

    ros2 run scara_kinematics ruta                        # perforado: informe + CSV + graficas
    ros2 run scara_kinematics ruta --ruta contorno
    ros2 run scara_kinematics ruta --perfil lineal        # la ley horaria de la guia, tal cual
    ros2 run scara_kinematics ruta --png ~/graficas --sin-ventanas

Esto solo CALCULA, como la guia. Para mandarlo a Gazebo o al brazo de la mesa:

    ros2 launch scara_kinematics ruta.launch.py

Unidades, como en la guia: longitud = mm | tiempo = s | angulos = rad.
El paso a las unidades de ROS (d3 en metros) esta en un solo sitio: `a_ros`.

LO QUE CAMBIA RESPECTO DE LA GUIA, Y POR QUE

 1. Las medidas. a1 = 250.0032 y a2 = 249.9997 mm (la guia: 350 y 250). Salen
    de config/scara_urdf3.yaml, medidas sobre la cadena del URDF.
 2. Los giros fijos b1 y b2. Al salir de SolidWorks los eslabones no arrancan
    alineados con X: x = a1*cos(q1 + b1) + a2*cos(q1 + q2 + b2). Sin ellos la
    punta se va ~13 deg de donde dice la formula. Entran en FK, J e IK.
 3. El husillo SUBE con d3: z = z_ref + d3 (la guia: z = d1 - d3). La tercera
    fila del Jacobiano es +1, no -1.
 4. Limites de junta. La guia normaliza los angulos a [-pi, pi]; este brazo no
    puede dar la vuelta (topes a +-120 y +-150 deg, cables). Aqui no se
    normaliza nada: si una junta se sale, la ruta se rechaza ANTES de mover.
    Lo mismo cerca de la singularidad (brazo estirado).
 5. Velocidades. El husillo real da 8 mm/s (max_vel_hw de esp.py), asi que la
    bajada va a 5 y la subida a 7 mm/s (la guia: 15 y 40). En el plano, 40 mm/s:
    las juntas no pasan de ~0.3 rad/s, que Gazebo sigue sin retraso apreciable.
 6. Ley horaria. La guia arranca cada tramo ya a la velocidad de crucero
    (s = t/T: aceleracion infinita en cada punto). Un motor no puede, asi que
    por defecto es 'trapecio' (rampa - crucero - rampa: la misma velocidad
    constante de avance mientras taladra, pero arrancando y parando). Con
    --perfil lineal sale exactamente la ley de la guia.
 7. Codo 'down' (el c2_sign = -1 de la guia). El brazo en reposo (q = 0) ya
    esta en esa rama; la otra obliga a pasar por el brazo estirado.
 8. La ruta sale de HOME y vuelve a HOME, la misma pose de reposo que `tray`.
 9. El error del lazo se mide contra donde se deberia estar AHORA. La guia lo
    mide contra el punto siguiente, que el feed-forward ya va a cubrir, y el
    robot acaba un paso por delante: un error fijo de v*dt (0.6 mm a sus
    120 mm/s; 0.2 mm aqui). Corregido, el error baja a ~5 micras y la
    autocorreccion del final de cada tramo ya no tiene nada que corregir.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass

import numpy as np

from scara_kinematics import trayectoria as T

# =========================================================
# 1. PARAMETROS GEOMETRICOS DEL ROBOT (el de la mesa)
# =========================================================
# Copiados de config/scara_urdf3.yaml (alli en metros). test/test_ruta.py
# comprueba que no se separan de el. El nodo `ruta_nodo` no usa estos: lee el
# YAML como los demas nodos (Robot.desde_modelo).
A1 = 250.0032        # hombro -> codo [mm]                     (la guia: 350)
B1 = 0.2268643       # giro fijo del eslabon 1 [rad] = +13.0 deg (la guia: 0)
A2 = 249.9997        # codo -> husillo [mm]                    (la guia: 250)
B2 = -0.2029018      # giro fijo del eslabon 2 [rad] = -11.6 deg (la guia: 0)
Z_REF = 65.51        # z de la punta de la pinza con d3 = 0 [mm]
Z_DIR = 1.0          # +1: el husillo SUBE con d3 (la guia: -1, bajaba)

Q1_LIM = (-2.0944, 2.0944)     # +-120 deg
Q2_LIM = (-2.618, 2.618)       # +-150 deg
D3_LIM = (20.0, 170.0)         # mm  ->  z entre 85.51 y 235.51 mm

NOMBRES = ('theta1', 'theta2', 'd3')


@dataclass(frozen=True)
class Robot:
    a1: float = A1
    b1: float = B1
    a2: float = A2
    b2: float = B2
    z_ref: float = Z_REF
    z_dir: float = Z_DIR
    lim: tuple = (Q1_LIM, Q2_LIM, D3_LIM)

    @staticmethod
    def desde_modelo(m) -> 'Robot':
        """El mismo robot, desde el ScaraModel de los nodos (metros -> mm)."""
        return Robot(m.l1 * 1e3, m.b1, m.l2 * 1e3, m.b2, m.z_ref * 1e3, m.z_dir,
                     ((m.q1_min, m.q1_max), (m.q2_min, m.q2_max),
                      (m.d3_min * 1e3, m.d3_max * 1e3)))

    def fuera(self, q, tol: float = 1e-9) -> str | None:
        """Que junta se sale de su recorrido, o None si ninguna."""
        for nombre, v, (lo, hi), u in zip(NOMBRES, q, self.lim, ('rad', 'rad', 'mm')):
            if not (lo - tol <= v <= hi + tol):
                if u == 'rad':
                    return (f'{nombre} = {math.degrees(v):.1f} deg fuera de '
                            f'[{math.degrees(lo):.0f}, {math.degrees(hi):.0f}] deg')
                return f'{nombre} = {v:.1f} mm fuera de [{lo:.0f}, {hi:.0f}] mm'
        return None

    def sin_codo(self, q) -> float:
        """|sin| del angulo entre eslabones. 0 = brazo estirado (singular)."""
        return abs(math.sin(q[1] - self.b1 + self.b2))


ROBOT = Robot()


def a_ros(q) -> list[float]:
    """[theta1 rad, theta2 rad, d3 mm] -> lo que esperan gz y la aduana (d3 en m)."""
    return [float(q[0]), float(q[1]), float(q[2]) * 1e-3]


# =========================================================
# 2. PUNTOS DE TRABAJO [X, Y, Z] en mm
# =========================================================
# Dentro de lo que alcanza ESTE brazo: anillo de 23 a 500 mm, hombro +-120 deg,
# z entre 85.5 y 235.5 mm. Se quedan entre 280 y 380 mm de radio, lejos del
# brazo estirado (500) y con 25 mm de margen en z por arriba y por abajo.
Z_SEGURO = 150.0     # altura de paso, en el aire [mm]
Z_TRABAJO = 110.0    # fondo del agujero / altura de contacto [mm] -> 40 mm de carrera

HOME = np.array([300.0, 0.0, Z_SEGURO])     # la pose de reposo de `tray`

# Perforado: tres agujeros
P1 = np.array([280.0, 120.0, Z_SEGURO])
P2 = np.array([280.0, 120.0, Z_TRABAJO])    # perforacion / bajada
P3 = np.array([360.0, 50.0, Z_SEGURO])
P4 = np.array([360.0, 50.0, Z_TRABAJO])     # perforacion / bajada
P5 = np.array([330.0, -100.0, Z_SEGURO])
P6 = np.array([330.0, -100.0, Z_TRABAJO])   # perforacion / bajada

# Contorno: un cuadrado de 80 mm recorrido en contacto (marcar, desbarbar)
C0 = np.array([290.0, -20.0, Z_SEGURO])     # encima de la primera esquina
C1 = np.array([290.0, -20.0, Z_TRABAJO])
C2 = np.array([370.0, -20.0, Z_TRABAJO])
C3 = np.array([370.0, 60.0, Z_TRABAJO])
C4 = np.array([290.0, 60.0, Z_TRABAJO])

# =========================================================
# 3. RUTAS COMPLETAS (una columna por punto, como en la guia)
# =========================================================
RUTAS = {
    'perforado': np.column_stack([HOME, P1, P2, P1, P3, P4, P3, P5, P6, P5, HOME]),
    'contorno': np.column_stack([HOME, C0, C1, C2, C3, C4, C1, C0, HOME]),
}
# Solo para rotular las graficas
PUNTOS = {
    'perforado': {'HOME': HOME, 'P1': P1, 'P2': P2, 'P3': P3,
                  'P4': P4, 'P5': P5, 'P6': P6},
    'contorno': {'HOME': HOME, 'C0': C0, 'C1': C1, 'C2': C2, 'C3': C3, 'C4': C4},
}

# =========================================================
# 4. VELOCIDADES Y PARAMETROS DE CONTROL
# =========================================================
# Velocidad de crucero de la punta en cada tipo de tramo [mm/s]. Con un perfil
# que no sea 'lineal', es el PICO: el tramo arranca y para en cero.
VELOCIDADES = {
    'bajada': 5.0,           # la "perforacion" de la guia (alli 15); husillo real: 8
    'subida': 7.0,           # la "salida" (alli 40)
    'desplazamiento': 40.0,  # horizontal a Z_SEGURO, en el aire (alli 120)
    'trazado': 15.0,         # horizontal por debajo de Z_SEGURO: en contacto
}

DT = 0.005                 # paso de integracion [s]
KP_POS = 3.0               # ganancia proporcional de posicion [1/s]

# Limites de velocidad de los motores: max_vel_hw de esp.py / brazo.launch.py
QDOT_MAX = np.array([1.5, 1.5, 8.0])    # [rad/s, rad/s, mm/s]

# Autocorreccion al final de cada tramo (igual que la guia)
TOL_POS = 0.01             # [mm]
KP_CORR = 10.0
MAX_ITER_CORR = 400

# Por debajo de esto el brazo esta casi estirado (|sin| = 0.05 -> 2.9 deg) y
# el Jacobiano pide velocidades enormes: se rechaza la ruta.
SIN_CODO_MIN = 0.05


class RutaError(ValueError):
    """La ruta no se puede hacer con este robot. Se sabe antes de mover nada."""


# =========================================================
# FUNCIONES CINEMATICAS DEL SCARA RRP (el de la mesa)
# =========================================================

def fk_scara_rrp(q, r: Robot = ROBOT) -> np.ndarray:
    """Cinematica directa: q = [theta1, theta2, d3] -> P = [x, y, z] (mm)."""
    a1, a2 = q[0] + r.b1, q[0] + q[1] + r.b2
    return np.array([r.a1 * math.cos(a1) + r.a2 * math.cos(a2),
                     r.a1 * math.sin(a1) + r.a2 * math.sin(a2),
                     r.z_ref + r.z_dir * q[2]])


def jacobiano_scara_rrp(q, r: Robot = ROBOT) -> np.ndarray:
    """Jacobiano 3x3: [vx, vy, vz] = J @ [theta1_dot, theta2_dot, d3_dot]."""
    a1, a2 = q[0] + r.b1, q[0] + q[1] + r.b2
    s1, c1, s12, c12 = math.sin(a1), math.cos(a1), math.sin(a2), math.cos(a2)
    return np.array([
        [-r.a1 * s1 - r.a2 * s12, -r.a2 * s12, 0.0],     # vx
        [r.a1 * c1 + r.a2 * c12, r.a2 * c12, 0.0],       # vy
        [0.0, 0.0, r.z_dir],                             # vz (+1: d3 sube)
    ])


def ik_scara_rrp(P, r: Robot = ROBOT, codo: str = 'down') -> np.ndarray:
    """Cinematica inversa analitica. codo: 'up' (c2_sign=+1 de la guia) o 'down' (-1).

    Es la misma que model.ik_all, escrita como en la guia. Levanta RutaError si
    el punto no se alcanza o la solucion se sale de los limites de junta.
    """
    x, y, z = float(P[0]), float(P[1]), float(P[2])
    d3 = (z - r.z_ref) / r.z_dir

    c = (x * x + y * y - r.a1 ** 2 - r.a2 ** 2) / (2.0 * r.a1 * r.a2)
    if abs(c) > 1.0 + 1e-12:
        raise RutaError(f'P = [{x:.1f}, {y:.1f}, {z:.1f}] fuera del alcance del SCARA '
                        f'(radio {math.hypot(x, y):.1f} mm)')
    g = math.acos(min(max(c, -1.0), 1.0))            # angulo entre eslabones
    if codo == 'down':
        g = -g
    elif codo != 'up':
        raise ValueError(f"codo tiene que ser 'up' o 'down', no {codo!r}")
    fi = math.atan2(y, x) - math.atan2(r.a2 * math.sin(g), r.a1 + r.a2 * math.cos(g))
    q = np.array([math.atan2(math.sin(fi - r.b1), math.cos(fi - r.b1)),
                  math.atan2(math.sin(g + r.b1 - r.b2), math.cos(g + r.b1 - r.b2)),
                  d3])
    malo = r.fuera(q)
    if malo:
        raise RutaError(f'P = [{x:.1f}, {y:.1f}, {z:.1f}] con codo={codo}: {malo}')
    return q


# =========================================================
# LEY HORARIA: la de la guia y las cinco de trayectoria.py
# =========================================================

class PerfilLineal(T.Perfil):
    """La ley de la guia: s = u. Velocidad constante desde el primer instante."""

    nombre = 'lineal'
    KV = 1.0
    KA = math.inf

    def s(self, u):
        return min(max(u, 0.0), 1.0)

    def ds(self, u):
        return 1.0

    def dds(self, u):
        return 0.0


PERFILES = ('lineal',) + tuple(T.PERFILES)


def perfil(nombre: str) -> T.Perfil:
    return PerfilLineal() if nombre == 'lineal' else T.perfil(nombre)


def tipo_de_tramo(Pi, Pf, z_seguro: float = Z_SEGURO) -> str:
    """La guia elige la velocidad por el signo de dZ; aqui ademas se distingue
    lo horizontal en el aire (desplazamiento) de lo horizontal en contacto."""
    dz = Pf[2] - Pi[2]
    if dz < -1e-9:
        return 'bajada'
    if dz > 1e-9:
        return 'subida'
    return 'trazado' if Pf[2] < z_seguro - 1e-6 else 'desplazamiento'


# =========================================================
# EL SEGUIMIENTO: feed-forward + Jacobiano inverso, y autocorreccion
# =========================================================

@dataclass
class Tramo:
    n: int                   # 1, 2, ... como en el informe de la guia
    tipo: str
    pi: np.ndarray
    pf: np.ndarray
    distancia: float         # mm
    velocidad: float         # mm/s, la de crucero
    duracion: float          # s, sin la autocorreccion
    iter_corr: int = 0
    error_final: float = 0.0


@dataclass
class Plan:
    """La trayectoria entera, muestra a muestra. Todo en mm / rad / s."""
    t: np.ndarray            # (N,)
    q: np.ndarray            # (N, 3)  [theta1, theta2, d3]
    qd: np.ndarray           # (N, 3)
    p: np.ndarray            # (N, 3)  la punta: FK(q)
    pd: np.ndarray           # (N, 3)  la punta deseada
    err: np.ndarray          # (N,)    |pd - p|
    tramo: np.ndarray        # (N,)    indice en `tramos` (-1 = el punto de salida)
    tramos: list
    ruta: np.ndarray         # (3, M)
    codo: str
    perfil: str

    @property
    def duracion(self) -> float:
        return float(self.t[-1])

    def en(self, t: float):
        """(q, qd, pd, tramo) en el instante t, interpolando entre muestras."""
        t = min(max(t, 0.0), self.duracion)
        k = int(np.searchsorted(self.t, t, side='right'))
        k = min(max(k, 1), len(self.t) - 1)
        t0, t1 = self.t[k - 1], self.t[k]
        w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        q = (1.0 - w) * self.q[k - 1] + w * self.q[k]
        pd = (1.0 - w) * self.pd[k - 1] + w * self.pd[k]
        return q, self.qd[k], pd, int(self.tramo[k])


def _qdot(q, v, r: Robot, qdot_max, donde: str) -> np.ndarray:
    """Jacobiano inverso + saturacion, como la guia. Se para cerca de la singularidad."""
    if r.sin_codo(q) < SIN_CODO_MIN:
        raise RutaError(f'{donde}: el brazo pasa casi estirado '
                        f'(|sin codo| = {r.sin_codo(q):.3f}); el Jacobiano no se invierte')
    qdot = np.linalg.pinv(jacobiano_scara_rrp(q, r)) @ v
    sat = float(np.max(np.abs(qdot) / qdot_max))
    return qdot / sat if sat > 1.0 else qdot


def planificar(ruta: np.ndarray, r: Robot = ROBOT, nombre_perfil: str = 'trapecio',
               codo: str = 'down', velocidades: dict | None = None, dt: float = DT,
               kp: float = KP_POS, qdot_max=QDOT_MAX, tol: float = TOL_POS,
               kp_corr: float = KP_CORR, max_iter: int = MAX_ITER_CORR,
               z_seguro: float = Z_SEGURO) -> Plan:
    """Recorre la ruta punto a punto. Levanta RutaError si algo no cabe."""
    vel = {**VELOCIDADES, **(velocidades or {})}
    pf_ = perfil(nombre_perfil)
    qdot_max = np.asarray(qdot_max, dtype=float)

    # Cinematica inversa del primer punto: de ahi sale el brazo
    q = ik_scara_rrp(ruta[:, 0], r, codo)
    P0 = fk_scara_rrp(q, r)
    t = 0.0
    t_h, q_h, qd_h, p_h = [t], [q.copy()], [np.zeros(3)], [P0]
    pd_h, e_h, tr_h = [ruta[:, 0].copy()], [float(np.linalg.norm(ruta[:, 0] - P0))], [-1]
    tramos: list[Tramo] = []

    def anotar(qdot, Pd, i):
        malo = r.fuera(q)
        if malo:
            raise RutaError(f'tramo {tramos[-1].n} ({tramos[-1].tipo}), '
                            f't = {t:.2f} s: {malo}')
        Pe = fk_scara_rrp(q, r)
        t_h.append(t)
        q_h.append(q.copy())
        qd_h.append(qdot.copy())
        p_h.append(Pe)
        pd_h.append(Pd.copy())
        e_h.append(float(np.linalg.norm(Pd - Pe)))
        tr_h.append(len(tramos) - 1)

    for i in range(ruta.shape[1] - 1):
        Pi, Pf = ruta[:, i], ruta[:, i + 1]
        dP = Pf - Pi
        D = float(np.linalg.norm(dP))
        if D < 1e-12:
            continue
        tipo = tipo_de_tramo(Pi, Pf, z_seguro)
        v = vel[tipo]
        Tf = pf_.kv * D / v            # con 'lineal' (kv = 1): D/v, la de la guia
        N = max(1, int(math.ceil(Tf / dt)))
        h = Tf / N
        tramos.append(Tramo(len(tramos) + 1, tipo, Pi.copy(), Pf.copy(), D, v, Tf))
        donde = f'tramo {len(tramos)} ({tipo})'

        # --- trayectoria principal: feed-forward + Jacobiano inverso ---
        for k in range(1, N + 1):
            Pd = Pi + pf_.s(k / N) * dP                  # a donde hay que llegar
            v_ff = pf_.ds((k - 0.5) / N) / Tf * dP       # dPd/dt en mitad del paso
            # El error, contra donde se deberia estar AHORA. La guia lo mide
            # contra Pd (el punto siguiente), y como el feed-forward ya da ese
            # paso, el robot acaba corriendo un paso por delante: v*dt de error
            # fijo (0.6 mm a sus 120 mm/s, 0.2 mm aqui a 40).
            ep = (Pi + pf_.s((k - 1) / N) * dP) - fk_scara_rrp(q, r)
            qdot = _qdot(q, v_ff + kp * ep, r, qdot_max, donde)
            q = q + qdot * h                             # Euler, sin normalizar
            t += h
            anotar(qdot, Pd, i)

        # --- autocorreccion al final del tramo ---
        it = 0
        while it < max_iter:
            ep = Pf - fk_scara_rrp(q, r)
            if np.linalg.norm(ep) < tol:
                break
            it += 1
            qdot = _qdot(q, kp_corr * ep, r, qdot_max, donde)
            q = q + qdot * dt
            t += dt
            anotar(qdot, Pf, i)
        tramos[-1].iter_corr = it
        tramos[-1].error_final = float(np.linalg.norm(Pf - fk_scara_rrp(q, r)))

    return Plan(np.array(t_h), np.array(q_h), np.array(qd_h), np.array(p_h),
                np.array(pd_h), np.array(e_h), np.array(tr_h, dtype=int),
                tramos, ruta, codo, pf_.nombre)


# =========================================================
# REPORTES, CSV Y GRAFICAS (lo que hace la guia al final)
# =========================================================

def informe(plan: Plan, nombre: str, qdot_max=QDOT_MAX, out=print):
    q0 = plan.q[0]
    out('=' * 56)
    out(f' SCARA DE LA MESA | ruta {nombre} | perfil {plan.perfil} | codo {plan.codo}')
    out('=' * 56)
    out(f'Theta 1 = {math.degrees(q0[0]):10.4f} deg')
    out(f'Theta 2 = {math.degrees(q0[1]):10.4f} deg')
    out(f'd3      = {q0[2]:10.4f} mm')
    out(f'P inicial = {np.round(plan.p[0], 4)}  error {plan.err[0]:.3e} mm')
    for tr in plan.tramos:
        out(f'\nTramo {tr.n:2d} : {tr.tipo.upper():<15s} v = {tr.velocidad:5.1f} mm/s')
        out(f'  Pi = [{tr.pi[0]:7.1f} {tr.pi[1]:7.1f} {tr.pi[2]:7.1f}]'
            f'  Pf = [{tr.pf[0]:7.1f} {tr.pf[1]:7.1f} {tr.pf[2]:7.1f}]')
        out(f'  distancia {tr.distancia:7.2f} mm | tiempo {tr.duracion:6.2f} s | '
            f'autocorreccion {tr.iter_corr} it | error final {tr.error_final:.5f} mm')

    pico = np.max(np.abs(plan.qd), axis=0)
    unid = ('rad/s', 'rad/s', 'mm/s')
    manip = min(ROBOT.sin_codo(q) for q in plan.q)
    out('\n' + '=' * 56)
    out(' ESTADISTICAS GLOBALES')
    out('=' * 56)
    out(f'Punto final alcanzado:  {np.round(plan.p[-1], 4)}')
    out(f'Error final posicion:   {np.linalg.norm(plan.ruta[:, -1] - plan.p[-1]):.6f} mm')
    out(f'Tiempo total:           {plan.duracion:.2f} s  ({len(plan.t)} muestras)')
    out(f'Error maximo posicion:  {plan.err.max():.6f} mm')
    out(f'Error medio posicion:   {plan.err.mean():.6f} mm')
    out(f'Error RMS posicion:     {math.sqrt(np.mean(plan.err ** 2)):.6f} mm')
    for j in range(3):
        out(f'{NOMBRES[j]}_dot pico: {pico[j]:8.4f} {unid[j]} '
            f'(limite {qdot_max[j]:.3f}, {100 * pico[j] / qdot_max[j]:5.1f} %)')
    out(f'|sin codo| minimo:      {manip:.3f} (0 = brazo estirado)')
    for j in range(3):
        lo, hi = plan.q[:, j].min(), plan.q[:, j].max()
        if j < 2:
            lo, hi = math.degrees(lo), math.degrees(hi)
        out(f'{NOMBRES[j]} recorre:  [{lo:8.2f}, {hi:8.2f}] {"deg" if j < 2 else "mm"}')


def exportar_csv(plan: Plan, ruta_csv: str):
    """Las cuatro columnas de la guia (tiempo, q1_rad, q2_rad, d3_mm) y detras
    lo que hace falta para el informe: tramo, punta real y deseada, error."""
    with open(ruta_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tiempo', 'q1_rad', 'q2_rad', 'd3_mm', 'tramo', 'tipo',
                    'q1_dot', 'q2_dot', 'd3_dot', 'x_mm', 'y_mm', 'z_mm',
                    'xd_mm', 'yd_mm', 'zd_mm', 'error_mm'])
        for k in range(len(plan.t)):
            i = plan.tramo[k]
            w.writerow([f'{plan.t[k]:.4f}'] + [f'{v:.7f}' for v in plan.q[k]]
                       + [i + 1, plan.tramos[i].tipo if i >= 0 else 'inicio']
                       + [f'{v:.6f}' for v in plan.qd[k]]
                       + [f'{v:.4f}' for v in plan.p[k]] + [f'{v:.4f}' for v in plan.pd[k]]
                       + [f'{plan.err[k]:.6f}'])


# Colores: azul = lo que hace el robot, naranja = lo que se pide.
AZUL, NARANJA = '#2a78d6', '#eb6834'
TINTA, TINTA2, REJILLA, GRIS = '#1f1f1d', '#52514e', '#e4e3df', '#a3a29c'


def graficas(plan: Plan, nombre: str, qdot_max=QDOT_MAX, carpeta: str | None = None,
             ventanas: bool = True):
    import matplotlib
    if not ventanas:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        'font.size': 9, 'axes.edgecolor': GRIS, 'axes.labelcolor': TINTA2,
        'xtick.color': TINTA2, 'ytick.color': TINTA2, 'text.color': TINTA,
        'axes.grid': True, 'grid.color': REJILLA, 'grid.linewidth': 0.7,
        'grid.linestyle': '-', 'axes.titlesize': 10, 'lines.solid_capstyle': 'round'})
    figs = {}

    # 1. Trayectoria cartesiana: en 3D (como la guia) y en planta
    fig = plt.figure(figsize=(11.5, 5.4))
    gs = fig.add_gridspec(1, 2, width_ratios=(1.3, 1.0), wspace=0.18,
                          left=0.0, right=0.97, bottom=0.1, top=0.9)
    ax = fig.add_subplot(gs[0], projection='3d')
    q0, p0 = plan.q[0], plan.p[0]
    codo = [ROBOT.a1 * math.cos(q0[0] + ROBOT.b1), ROBOT.a1 * math.sin(q0[0] + ROBOT.b1)]
    puntos = PUNTOS.get(nombre, {})
    # el brazo en la pose de salida, en gris: columna -> codo -> punta
    ax.plot([0, 0], [0, 0], [0, p0[2]], color=GRIS, lw=2.0)
    ax.plot([0, codo[0], p0[0]], [0, codo[1], p0[1]], [p0[2]] * 3,
            color=GRIS, lw=2.0, label='Brazo en la salida')
    ax.plot(*plan.p.T, color=AZUL, lw=2.0, label='Trayectoria del robot')
    ax.plot(*plan.ruta, '--', color=NARANJA, lw=1.2, label='Trayectoria deseada')
    for et, P in puntos.items():
        ax.scatter(*P, color=TINTA, s=24, depthshade=False)
        abajo = P[2] < Z_SEGURO - 1e-6           # los del fondo, rotulados por debajo
        ax.text(P[0], P[1], P[2] - (14 if abajo else -6), f' {et}', color=TINTA,
                fontsize=8, va='top' if abajo else 'bottom')
    # proporciones de verdad: 40 mm de bajada se ven como 40 mm
    xs = np.r_[plan.p[:, 0], 0.0, codo[0]]
    ys = np.r_[plan.p[:, 1], 0.0, codo[1]]
    lims = [(xs.min() - 20, xs.max() + 20), (ys.min() - 20, ys.max() + 20),
            (0.0, plan.p[:, 2].max() + 30)]
    ax.set_xlim(*lims[0])
    ax.set_ylim(*lims[1])
    ax.set_zlim(*lims[2])
    ax.set_box_aspect([hi - lo for lo, hi in lims])
    ax.view_init(elev=24, azim=-58)
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]', labelpad=2)
    ax.legend(loc='upper left', frameon=False, fontsize=8)

    ax = fig.add_subplot(gs[1])
    ax.plot([0, codo[0], p0[0]], [0, codo[1], p0[1]], color=GRIS, lw=2.0)
    ax.scatter([0], [0], color=GRIS, s=40, zorder=3)
    ax.text(0, -12, 'base', color=TINTA2, fontsize=8, ha='center', va='top')
    ax.plot(plan.p[:, 0], plan.p[:, 1], color=AZUL, lw=2.0)
    ax.plot(plan.ruta[0], plan.ruta[1], '--', color=NARANJA, lw=1.2)
    vistos = set()
    for et, P in puntos.items():
        ax.scatter(P[0], P[1], color=TINTA, s=24, zorder=3)
        clave = (round(P[0]), round(P[1]))
        et_xy = ' / '.join(e for e, Q in puntos.items()
                           if (round(Q[0]), round(Q[1])) == clave)
        if clave not in vistos:              # P1 y P2 caen en el mismo sitio
            vistos.add(clave)
            ax.text(P[0] + 6, P[1] + 6, et_xy, color=TINTA, fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_title('Planta (vista desde arriba)')
    fig.suptitle(f'Trayectoria cartesiana, SCARA de la mesa: {nombre}')
    figs['cartesiana'] = fig

    # 2. Juntas: posicion (izquierda) y velocidad (derecha) frente al tiempo
    fig, axs = plt.subplots(3, 2, figsize=(10.0, 6.0), sharex=True)   # cabe en 1366x768
    esc = (math.degrees(1), math.degrees(1), 1.0)
    et_q = (r'$\theta_1$ [deg]', r'$\theta_2$ [deg]', r'$d_3$ [mm]')
    et_v = (r'$\dot\theta_1$ [deg/s]', r'$\dot\theta_2$ [deg/s]', r'$\dot d_3$ [mm/s]')
    for j in range(3):
        axs[j, 0].plot(plan.t, plan.q[:, j] * esc[j], color=AZUL, lw=2.0)
        axs[j, 0].set_ylabel(et_q[j])
        axs[j, 1].plot(plan.t, plan.qd[:, j] * esc[j], color=AZUL, lw=2.0)
        axs[j, 1].set_ylabel(et_v[j])
        lim, pico = qdot_max[j] * esc[j], np.max(np.abs(plan.qd[:, j])) * esc[j]
        if lim < 1.6 * max(pico, 1e-9):      # el limite, solo si esta a la vista
            for s in (1, -1):
                axs[j, 1].axhline(s * lim, color=GRIS, lw=1.0)
            axs[j, 1].text(plan.t[-1], lim, 'limite del motor', ha='right',
                           va='bottom', color=TINTA2, fontsize=8)
    axs[2, 0].set_xlabel('Tiempo [s]')
    axs[2, 1].set_xlabel('Tiempo [s]')
    fig.suptitle(f'Variables articulares ({nombre}, perfil {plan.perfil})')
    fig.tight_layout()
    figs['articulares'] = fig

    # 3. Error de seguimiento
    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    ax.plot(plan.t, plan.err * 1e3, color=AZUL, lw=2.0)
    ax.set_xlabel('Tiempo [s]')
    ax.set_ylabel('Error de posicion [µm]')
    ax.set_title(f'Error de seguimiento cartesiano: max {plan.err.max() * 1e3:.2f} µm, '
                 f'RMS {math.sqrt(np.mean(plan.err ** 2)) * 1e3:.2f} µm')
    fig.tight_layout()
    figs['error'] = fig

    if carpeta:
        carpeta = os.path.expanduser(carpeta)
        os.makedirs(carpeta, exist_ok=True)
        for k, f in figs.items():
            destino = os.path.join(carpeta, f'{nombre}_{k}.png')
            f.savefig(destino, dpi=110)
            print(f'grafica: {destino}')
    if ventanas:
        plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='ruta', description='Trayectorias reales del SCARA de la mesa, por Jacobiano.')
    ap.add_argument('--ruta', choices=tuple(RUTAS), default='perforado')
    ap.add_argument('--perfil', choices=PERFILES, default='trapecio',
                    help="'lineal' = la ley de la guia (velocidad constante)")
    ap.add_argument('--codo', choices=('down', 'up'), default='down')
    ap.add_argument('--csv', default=None,
                    help='por defecto ~/trayectoria_scara_<ruta>.csv')
    ap.add_argument('--png', default=None, help='carpeta donde guardar las graficas')
    ap.add_argument('--sin-ventanas', action='store_true')
    ap.add_argument('--sin-graficas', action='store_true')
    args, _ = ap.parse_known_args(argv)      # tolera un --ros-args de ros2 run

    try:
        plan = planificar(RUTAS[args.ruta], nombre_perfil=args.perfil, codo=args.codo)
    except RutaError as e:
        print(f'la ruta {args.ruta} no sale: {e}', file=sys.stderr)
        return 1
    informe(plan, args.ruta)

    ruta_csv = os.path.expanduser(args.csv or f'~/trayectoria_scara_{args.ruta}.csv')
    exportar_csv(plan, ruta_csv)
    print(f'\nCSV: {ruta_csv}')
    if not args.sin_graficas:
        graficas(plan, args.ruta, carpeta=args.png, ventanas=not args.sin_ventanas)
    return 0


if __name__ == '__main__':
    sys.exit(main())
