"""El ir «por juntas»: cada junta a SU velocidad, fija, como en los programas de
Sergio, sin corregir nada por el camino; al llegar se mide y se corrige lo que
falte. Lo pidio el 23/10: «solo corrige el error cuando llega al punto, hazlo
con las velocidades para cada juntura que ya te dije».

Las velocidades son `velocidades` de config/micropython.yaml, las del vector de
main.py: Alvin 14 RPM (84 grados/s), Simon 30 RPM (180 grados/s) y Teodoro
60 mm/min (1 mm/s).

Sin ROS: lo usan juntas_nodo y juntas_gui, y los tests con un motor de
mentira. Todo en grados de junta (Alvin, Simon) y mm (Teodoro).

Un ir, por fases:

    ir        cada junta, a +-V hacia su objetivo (la IK del punto), hasta que
              le falta ANTICIPO: lo que aun anda despues de mandarle 0 (la
              consigna de la placa va por delante, y la orden tarda en llegar).
              Las juntas no se esperan: cada una para cuando llega, asi que la
              punta no va en recta.
    asentar   todo a 0 hasta que las tres se paran
    corregir  si alguna esta a mas de TOL de su objetivo, su consigna en la
              placa (la posicion medida mas el seguimiento que manda la placa)
              se lleva al objetivo: v = falta / T durante T. Y otra vez a
              asentar. CORRECCIONES veces como mucho
    hecho     lo que queda, en la punta y en cada junta
"""
from __future__ import annotations

import dataclasses
import math
from collections import deque

import numpy as np

from scara_kinematics import alcance as A
from scara_kinematics import ruta as R
from scara_kinematics import traycho as T

NOMBRES = ('Alvin', 'Simon', 'Teodoro')
UNIDADES = ('°', '°', ' mm')
# Lo que la consigna de la placa va por delante del motor como mucho
# (ADELANTO_MAX de main_control.py, 1.5 grados del eje del motor), en grados de
# junta y mm: Alvin 1:1, Simon con su 18/21 y Teodoro con sus 41.4 mm por vuelta
ADELANTO = (1.5, 1.5 * 18 / 21, 1.5 * (3.14159265 * 10 + 10) / 360)
RETRASO_S = 0.04          # de la medida al motor: la P, el tic y la V. SUPUESTO
# Llegado. Menos no: la placa deja de empujar a BANDA (0.3 grados de motor) de
# su consigna, y parada no vuelve hasta el doble
TOL = (0.4, 0.4, 0.25)                 # grados, grados, mm
CORRECCIONES = 3
T_CORREGIR_S = 0.4        # una correccion dura esto (si a las V no le da)...
# ... salvo que asi vaya mas despacio que esto: la placa la tomaria por «quieta»
# (VEL_QUIETA, 0.5 grados/s de motor) y no empujaria hasta el doble de la banda
V_CORREGIR_MIN = (0.6, 0.6, 0.07)      # grados/s, grados/s, mm/s
# Y tampoco mas deprisa que ADELANTO en PEGADO_S: lo que tarda un motor parado en
# arrancar (la placa sube el PWM de ARRANQUE a TOPE en 0.2 s; a los 0.25 s sin
# moverse salta su vigilante). Mientras no arranca, la consigna no pasa de
# ADELANTO por delante, y lo que se pida de mas se pierde (23/10, Alvin en el brazo)
PEGADO_S = 0.3
CORRECCION_MIN = (0.05, 0.05, 0.01)    # menos que esto no se le pide a la consigna
ASENTAR_MIN_S = 0.3
ASENTAR_MAX_S = 1.5
# Parado no es llegado: Alvin, para arrancar, necesita que la placa suba su PWM
# un rato, y mientras tanto no se mueve (23/10, en el brazo: se dio por llegado
# a 1.3 grados y luego se movio). Hasta que la placa esta a menos de esto de su
# consigna (el doble de su banda: ahi, parada, ya no empuja) o pasa
# ASENTAR_LARGO_S, se espera
SEG_LISTA = (0.6, 0.6 * 18 / 21, 0.25)          # grados, grados, mm
ASENTAR_LARGO_S = 3.0
QUIETA = (1.0, 1.0, 0.05)              # grados/s, grados/s, mm/s: por debajo, parada
VENTANA_QUIETA_S = 0.1    # la velocidad se mide en este tiempo
ESPERA_SEG_S = 0.5        # ya parada, lo que se espera a un seguimiento nuevo (5 Hz)

# Los puntos del traycho por juntas: P1 es el HOME (23/10: «home es P1»; en su
# codigo el robot arranca en P1). Las z, las de traycho.py (media escala, P1
# arriba del todo).
# HOME (24/10, tarde): el de antes, (0, -270) con Simon en 0, era el antebrazo
# contra la base (Sergio: «choca con la base, o sea con el robot, creo que
# dejamos el home muy pegado»). Con Simon en -10, «ponlo como entre 5 a 10
# grados mas lejos» (-17.5); tras el traycho de las 16:47, «es mejor que
# alejemos el home por ahi 10 grados, y el home de retorno se pego un poquito a
# la base»: Simon en SIMON_HOME (cuenta de la placa; 0 sigue siendo la base). Su
# punta, con x = 0: P1_MM (Alvin en +13.75).
# Y Simon no pasa de SIMON_MAX hacia la base: traycho_juntas.launch.py se lo pone
# de tope a juntas_nodo y a la ventana (simon_max), como teodoro_min.
SIMON_HOME = -27.5
SIMON_MAX = -5.0
P1_MM = (0.0, -362.3)
# SUPUESTO, el giro: solo trasladada (con el HOME de antes), P3, P5 y P7 quedaban
# a 20 cm de la base, mas cerca de lo que llega el brazo. Girada GIRO_GRADOS
# alrededor de P1, sin escalar, cabe (con el HOME de ahora, Alvin de -11 a +45 y
# Simon de -75 a -27.5).
GIRO_GRADOS = -35.0
# Teodoro no baja de aqui (mm de su motor; 0 es arriba del todo): lleva un
# soporte cilindrico; bajando a P2 choco con el (23/10, lo paro en -47) y
# bajando a P4 casi (lo paro en -10: «el soporte, en todos»; pero ese dia la
# placa habia perdido el HOME y se fijo Teodoro a 0 donde estaba, no arriba: el
# -10 no vale). 29/10, con el cero arriba otra vez («casi no esta bajando»): de
# -6 a -20; y luego, con las z de su codigo («debe bajar casi todo»), a -120:
# el traycho llega a -115.5 (el recorrido de la placa acaba en -135).
# traycho_juntas.launch.py lo pone de tope a juntas_nodo y a la ventana: no lo
# pasa ni el traycho ni «Ir a un punto». SUPUESTO, con margen
TEODORO_MIN_MM = -120.0
# 29/10, con las z de su codigo: P3, P5 y P7 (la fila de +x) quedan encima del
# soporte. Bajando a P3 (65 mm, Teodoro a -65.5) se bloqueo en -54.9: «es el
# soporte, limita P3 P5 P7 y sigue». Esos tres no bajan de Z_SOPORTE_MM:
# Teodoro a -50 (z = 220.51 en su 0). SUPUESTO, el margen de 5 mm; P5 y P7, sin
# probar.
PUNTOS_SOPORTE = (3, 5, 7)
Z_SOPORTE_MM = 170.5
# 30/10: bajando a P2 (Teodoro a -115.5) lo paro en -52.2 antes de chocar: «deja
# a teodoro que baje hasta ahi, porque se iba a chocar». P2, P4 y P6 no bajan de
# Z_HONDOS_MM: Teodoro a -52.1. SUPUESTO, P4 y P6 igual (23/10, bajando a P4:
# «el soporte, en todos»), sin probar.
PUNTOS_HONDOS = (2, 4, 6)
Z_HONDOS_MM = 168.4


def puntos_mm(p1=P1_MM, giro: float = GIRO_GRADOS) -> np.ndarray:
    """Los ocho puntos de su codigo con P1 en HOME: la misma figura (las mismas
    distancias), girada `giro` grados alrededor de P1."""
    p = T.puntos_mm()
    c, s = math.cos(math.radians(giro)), math.sin(math.radians(giro))
    rel = p[:, :2] - p[0, :2]
    p[:, :2] = np.asarray(p1, dtype=float) + rel @ np.array([[c, -s], [s, c]]).T
    for i in PUNTOS_SOPORTE:
        p[i - 1, 2] = max(p[i - 1, 2], Z_SOPORTE_MM)
    for i in PUNTOS_HONDOS:
        p[i - 1, 2] = max(p[i - 1, 2], Z_HONDOS_MM)
    return p


def velocidades(cal: A.Calibracion) -> np.ndarray:
    """Las de main.py (RPM de junta, RPM de junta, mm/min) en grados/s y mm/s."""
    return np.array([v * f for v, f in zip(cal.velocidades, A.POR_SEGUNDO)], dtype=float)


def a_grados(q) -> np.ndarray:
    """[rad, rad, mm] -> [grados, grados, mm]."""
    return np.array([math.degrees(q[0]), math.degrees(q[1]), float(q[2])])


def a_rad(q) -> np.ndarray:
    return np.array([math.radians(q[0]), math.radians(q[1]), float(q[2])])


def seguimiento_a_junta(seg, cal: A.Calibracion) -> np.ndarray:
    """La D de la placa (consigna - posicion, en sus unidades) en grados y mm de junta."""
    return np.array([seg[0] / (cal.signo[0] * cal.reduccion[0]),
                     seg[1] / (cal.signo[1] * cal.reduccion[1]),
                     seg[2] / cal.signo[2] * cal.paso_husillo_mm / 360.0])


def sin_topes(robot: R.Robot) -> R.Robot:
    return dataclasses.replace(robot, lim=((-math.pi, math.pi), (-math.pi, math.pi),
                                           (-math.inf, math.inf)))


def objetivo(p, robot: R.Robot) -> np.ndarray:
    """La IK del punto (rama down), en grados y mm. RutaError si no cabe en los topes."""
    return a_grados(R.ik_scara_rrp(p, robot, 'down'))


def tramos_juntas(pts, robot: R.Robot, desde=None, grados_max: float = T.GRADOS_MAX,
                  z_arriba=None) -> list:
    """[(nombre, punto mm)] para ir por juntas.

    Cada punto, y entre uno y otro trozos iguales EN JUNTAS (lo que hace el
    brazo, mas o menos: cada junta va a lo suyo) para que ninguna gire mas de
    grados_max en un ir. Levanta RutaError si un punto no cabe en los topes de
    `robot`. `desde`, donde esta la punta, no se comprueba: en HOME Simon esta
    en su tope, y de ahi se sale hacia dentro. Los trozos se quedan dentro.

    Con `z_arriba` (mm), en planta solo se va con Teodoro arriba: a cada punto
    que este mas bajo se llega por encima, se baja en vertical y se vuelve a
    subir (23/10: «no subio, quedo abajo y se choco, debe bajar y despues
    subir»). Si la punta esta abajo, lo primero es subir.
    """
    pts = [np.asarray(p, dtype=float) for p in pts]
    qa = R.ik_scara_rrp(pts[0] if desde is None else desde, sin_topes(robot), 'down')
    salida = []

    def dentro(q):
        q = np.array(q, dtype=float)
        for j, (lo, hi) in enumerate(robot.lim):
            eps = 1e-7 * max(1.0, abs(hi - lo))
            q[j] = min(max(q[j], lo + eps), hi - eps)
        return q

    def ir_a(nombre, b):
        nonlocal qa
        qb = R.ik_scara_rrp(b, robot, 'down')                    # levanta si no cabe
        n = max(1, math.ceil(np.max(np.degrees(np.abs(qb[:2] - qa[:2]))) / grados_max - 1e-9))
        for k in range(1, n + 1):
            p = b if k == n else R.fk_scara_rrp(dentro(qa + (qb - qa) * (k / n)), robot)
            salida.append((nombre if n == 1 else f'{nombre} ({k}/{n})', p))
        qa = qb

    if z_arriba is not None and desde is not None and desde[2] < z_arriba - 0.5:
        q = dentro(qa)
        q[2] = (z_arriba - robot.z_ref) / robot.z_dir
        ir_a('sube', R.fk_scara_rrp(q, robot))
    for i, b in enumerate(pts, 1):
        if z_arriba is None or b[2] >= z_arriba - 1e-9:
            ir_a(f'P{i}', b)
        else:
            encima = np.array([b[0], b[1], z_arriba])
            ir_a(f'P{i} encima', encima)
            ir_a(f'P{i}', b)                                     # baja
            ir_a(f'P{i} sube', encima)
    return salida


def camino(qa, qb, vel, robot: R.Robot, n: int = 120) -> np.ndarray:
    """Por donde va a ir la punta (mm): cada junta a su velocidad hasta la suya."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    d = qb - qa
    dura = float(np.max(np.abs(d) / vel))
    if dura <= 0.0:
        return np.array([R.fk_scara_rrp(a_rad(qb), robot)] * 2)
    libre = sin_topes(robot)
    return np.array([R.fk_scara_rrp(a_rad(qa + np.sign(d) * np.minimum(vel * t, np.abs(d))),
                                    libre) for t in np.linspace(0.0, dura, n)])


def camino_planta(pts, robot: R.Robot, z_arriba=None, vel=None, n: int = 30) -> np.ndarray:
    """Por donde va la punta (mm) en todo el traycho por juntas, de P1 a P1: el
    `camino` de cada ida de tramos_juntas. No son rectas: cada junta va a lo
    suyo, y asi la punta no se sale de donde llega el brazo (23/10: la recta de
    P2 a P3 que pintaba la ventana pasaba por la zona blanca)."""
    vel = velocidades(A.Calibracion()) if vel is None else vel
    tramos = tramos_juntas(pts, robot, z_arriba=z_arriba)
    libre = sin_topes(robot)
    qa = a_grados(R.ik_scara_rrp(pts[0], libre, 'down'))
    trozos = []
    for _, p in tramos:
        qb = a_grados(R.ik_scara_rrp(p, libre, 'down'))
        trozos.append(camino(qa, qb, vel, robot, n))
        qa = qb
    return np.vstack(trozos)


def _txt(v, signo=True) -> str:
    f = '{:+.2f}' if signo else '{:.2f}'
    return ', '.join(f'{n} {f.format(x)}{u}' for n, x, u in zip(NOMBRES, v, UNIDADES))


class Juntas:
    """Lleva el brazo a un punto por juntas.

    `paso(t, q, seg, t_seg)`, en cada tic: q, lo medido (grados, grados, mm);
    seg, el ultimo seguimiento de la placa ya en junta (seguimiento_a_junta) y
    t_seg, cuando llego. Devuelve la velocidad de cada junta (grados/s,
    grados/s, mm/s). Lo que haya que contar se queda en `avisos`, como
    (nivel, texto), con los niveles de /rosout: 20, 30, 40.
    """

    def __init__(self, robot: R.Robot, vel, dt: float = 0.02, tol=TOL,
                 correcciones: int = CORRECCIONES, retraso: float = RETRASO_S):
        self.robot = robot
        self.vel = np.asarray(vel, dtype=float)
        self.dt = float(dt)
        self.tol = np.asarray(tol, dtype=float)
        self.correcciones = int(correcciones)
        self.anticipo = np.asarray(ADELANTO) + self.vel * retraso
        self.avisos: list = []
        self.fase = 'quieto'          # quieto | ir | asentar | corregir | hecho
        self.nombre = ''
        self.destino = None           # mm
        self.obj = None               # grados, grados, mm
        self.dura = 0.0
        self._hist: deque = deque()
        self.v_corr = np.zeros(3)
        self.ticks = 0

    def activo(self) -> bool:
        return self.fase in ('ir', 'asentar', 'corregir')

    # ------------------------------------------------------------------ ordenes
    def ir(self, destino, q, t: float, nombre: str = 'ir') -> float:
        """A destino (mm). Levanta RutaError si no cabe. Devuelve lo que tarda en ir."""
        return self._empezar(np.asarray(destino, dtype=float), objetivo(destino, self.robot),
                             q, t, nombre)

    def a_juntas(self, obj, q, t: float, nombre: str = 'home') -> float:
        """A unas juntas (grados, grados, mm), recortadas a los topes."""
        obj = np.array(obj, dtype=float)
        rad = a_rad(obj)
        for j, (lo, hi) in enumerate(self.robot.lim):
            rad[j] = min(max(rad[j], lo), hi)
        obj = a_grados(rad)
        return self._empezar(R.fk_scara_rrp(rad, self.robot), obj, q, t, nombre)

    def parar(self, porque: str = 'quieto donde estoy'):
        if self.activo():
            self.avisos.append((30, f'parado: {porque}'))
        self.fase = 'quieto'

    def _empezar(self, destino, obj, q, t, nombre) -> float:
        q = np.asarray(q, dtype=float)
        self.destino, self.obj, self.nombre = destino, obj, nombre
        d = obj - q
        self.signo = np.sign(d)
        self.llegada = np.abs(d) <= self.anticipo
        self.dura = float(np.max(np.abs(d) / self.vel))
        self.t0 = t
        self._hist.clear()                 # lo parada que esta, desde ahora
        self.fase = 'ir'
        self.n_corr = 0
        self.antes = None
        self.camino = camino(q, obj, self.vel, self.robot)
        self.avisos.append((20, f'{nombre}: {self.dura:.1f} s (por juntas: '
                                + ', '.join(f'{n} {x:+.2f}{u} a {v:g}{u}/s'
                                            for n, x, u, v in zip(NOMBRES, d, UNIDADES, self.vel))
                                + ')'))
        return self.dura

    # ------------------------------------------------------------------ el tic
    def paso(self, t: float, q, seg=None, t_seg=None) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        self._hist.append((t, q))
        while len(self._hist) > 2 and t - self._hist[1][0] >= VENTANA_QUIETA_S:
            self._hist.popleft()
        if not self.activo():
            return np.zeros(3)
        falta = self.obj - q
        if self.fase == 'ir':
            # Sin corregir: cada una a su V hasta que le falta su anticipo (o se pasa)
            self.llegada |= (np.abs(falta) <= self.anticipo) | (falta * self.signo <= 0.0)
            if not self.llegada.all():
                if t - self.t0 > 3.0 * self.dura + 10.0:
                    self.parar(f'{self.nombre} no llega en {t - self.t0:.0f} s; le falta '
                               + _txt(falta))
                    return np.zeros(3)
                return np.where(self.llegada, 0.0, self.signo * self.vel)
            self._a_asentar(t)
        if self.fase == 'corregir':
            if self.ticks > 0:
                self.ticks -= 1
                return self.v_corr.copy()
            self._a_asentar(t)
        return self._asentar(t, q, falta, seg, t_seg)

    def _a_asentar(self, t):
        self.fase = 'asentar'
        self.t_fase = t
        self.t_quieta = None
        self.muestras = []

    def _quieta(self) -> bool:
        (t0, q0), (t1, q1) = self._hist[0], self._hist[-1]
        if t1 - t0 < 0.5 * VENTANA_QUIETA_S:
            return False
        return bool(np.all(np.abs(q1 - q0) / (t1 - t0) <= QUIETA))

    def _asentar(self, t, q, falta, seg, t_seg) -> np.ndarray:
        dur = t - self.t_fase
        if self.t_quieta is None:
            if dur >= ASENTAR_MIN_S and (self._quieta() or dur >= ASENTAR_MAX_S):
                self.t_quieta = t
            else:
                return np.zeros(3)
        # Donde esta la consigna de la placa: con un seguimiento de despues de
        # pararse; si no llega, la posicion (la consigna esta a menos de la banda)
        fresco = seg is not None and t_seg is not None and t_seg >= self.t_quieta
        if fresco and dur < ASENTAR_LARGO_S and (
                np.any(np.abs(seg) > SEG_LISTA) or not self._quieta()):
            self.muestras = []             # la placa aun lo esta llevando
            return np.zeros(3)
        self.muestras.append(float(np.linalg.norm(self.destino - R.fk_scara_rrp(
            a_rad(q), sin_topes(self.robot)))))
        if not fresco and t - self.t_quieta < ESPERA_SEG_S:
            return np.zeros(3)
        ref = q + np.asarray(seg, dtype=float) if fresco else q
        return self._decidir(t, q, falta, ref)

    def _decidir(self, t, q, falta, ref) -> np.ndarray:
        if self.antes is None:
            self.antes = self.muestras[-1]
        fuera = np.abs(falta) > self.tol
        mover = np.where(fuera, self.obj - ref, 0.0)
        mover = np.where(np.abs(mover) >= CORRECCION_MIN, mover, 0.0)
        if not fuera.any() or self.n_corr >= self.correcciones or not mover.any():
            self._hecho(falta, fuera)
            return np.zeros(3)
        m = np.abs(mover)
        dura = min(T_CORREGIR_S, float(np.min((m / V_CORREGIR_MIN)[m > 0.0])))
        dura = max(dura, float(np.max(m / (np.asarray(ADELANTO) / PEGADO_S))), self.dt)
        self.ticks = max(1, math.ceil(dura / self.dt - 1e-9))
        self.v_corr = mover / (self.ticks * self.dt)
        self.n_corr += 1
        self.fase = 'corregir'
        self.ticks -= 1
        return self.v_corr.copy()

    def _hecho(self, falta, fuera):
        a = np.array(self.muestras)
        txt = (f'{self.nombre} terminada | en el punto: medio {a.mean():.3f} mm, maximo '
               f'{a.max():.3f} mm | {self.n_corr} correcciones (al acabar de ir, '
               f'{self.antes:.2f} mm) | le falta {_txt(falta)}')
        nivel = 20
        if fuera.any():
            nivel = 30
            txt += (' | FUERA de la tolerancia ('
                    + ', '.join(n for n, f in zip(NOMBRES, fuera) if f)
                    + '): la placa no lo acerca mas')
        self.avisos.append((nivel, txt))
        self.fase = 'hecho'
