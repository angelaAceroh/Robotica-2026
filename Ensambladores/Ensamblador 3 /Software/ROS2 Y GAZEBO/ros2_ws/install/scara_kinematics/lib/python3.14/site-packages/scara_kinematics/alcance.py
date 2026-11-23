"""¿Llega la punta a (x, y, z)? Con los limites REALES de los motores.

Biblioteca sin ROS. La usan el nodo `vector` (antes de mandar nada a la placa)
y la ventana `efector` (para decirlo al momento, mientras escribes).

Dos espacios distintos, y hay que no mezclarlos:

    junta   lo que entiende la cinematica: theta1, theta2 en rad y d3 en mm,
            con theta = 0 = brazo estirado (asi salio del CAD).
    motor   lo que cuentan main.py y main_control.py, en las unidades de los
            programas de Sergio (Alvin.py, Simon.py, Teodoro.py): Alvin y Simon
            en grados de su junta, Teodoro en mm. Los recorridos que mide Sergio
            estan en ESTE espacio.

    motor = signo * reduccion * (grados(junta) - cero)          Alvin, Simon
    motor = signo * (d3 - cero) * 360 / paso_husillo            Teodoro

Como la placa ya cuenta a Teodoro en mm, paso_husillo = 360 (un «grado» de
Teodoro es un mm). Todos esos numeros viven en config/micropython.yaml: es el
unico sitio donde hay que tocarlos.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from scara_kinematics import ruta as R

MOTORES = ('Alvin', 'Simon', 'Teodoro')
UNIDADES = ('°', '°', ' mm')               # de cada motor en la placa
# Las velocidades de main.py, como en los programas de Sergio: Alvin y Simon en
# RPM de su junta, Teodoro en mm por minuto (Teodoro.py divide entre PULSOS_POR_MM)
VEL_UNIDADES = ('RPM', 'RPM', 'mm/min')
POR_SEGUNDO = (6.0, 6.0, 1.0 / 60.0)       # 1 RPM = 6 grados/s; 1 mm/min = 1/60 mm/s
# Con el codo recto (theta2 = b1 - b2) el brazo es singular y el control no
# cruza de una rama a otra. Si el recorrido de Simon lo cruza (desde el 21/10:
# de 0 a -280, recto en -120), las zonas se dibujan en la rama down, la del
# brazo en 0, y apartadas esto del codo recto.
RECTO_MARGEN = math.radians(10.0)


def lado_down(robot: R.Robot, q2_lo: float, q2_hi: float):
    """(q2_lo, q2_hi) recortado a la rama down si cruza el codo recto."""
    recto = robot.b1 - robot.b2
    if q2_lo < recto - RECTO_MARGEN < q2_hi:
        q2_hi = recto - RECTO_MARGEN
    return q2_lo, q2_hi


@dataclass
class Calibracion:
    limites: list = field(default_factory=lambda: [(-60.0, 60.0), (-280.0, 0.0), (-135.0, 0.0)])
    reduccion: list = field(default_factory=lambda: [1.0, 1.0])
    signo: list = field(default_factory=lambda: [-1.0, -1.0, 1.0])
    cero: list = field(default_factory=lambda: [-45.68, -90.01, 155.0])   # grados, grados, mm
    paso_husillo_mm: float = 360.0          # 360: Teodoro ya va en mm. 0 = sin medir
    velocidades: list = field(default_factory=lambda: [14.0, 30.0, 60.0])   # RPM, RPM, mm/min

    @staticmethod
    def desde_parametros(get) -> 'Calibracion':
        """`get(nombre, defecto)` -> valor. Sirve para un nodo y para un dict."""
        d = Calibracion()
        lim = [float(v) for v in get('limites_motor', [x for par in d.limites for x in par])]
        return Calibracion(
            limites=[(min(lim[i], lim[i + 1]), max(lim[i], lim[i + 1])) for i in (0, 2, 4)],
            reduccion=[float(v) for v in get('reduccion', d.reduccion)],
            signo=[float(v) for v in get('signo', d.signo)],
            cero=[float(v) for v in get('cero', d.cero)],
            paso_husillo_mm=float(get('paso_husillo_mm', d.paso_husillo_mm)),
            velocidades=[float(v) for v in get('velocidades', d.velocidades)])

    @property
    def husillo_medido(self) -> bool:
        return self.paso_husillo_mm > 0.0

    def a_motor(self, q) -> list:
        """[theta1 rad, theta2 rad, d3 mm] -> motor (grados, grados, mm). Teodoro None sin paso."""
        a = self.signo[0] * self.reduccion[0] * (math.degrees(q[0]) - self.cero[0])
        s = self.signo[1] * self.reduccion[1] * (math.degrees(q[1]) - self.cero[1])
        t = (self.signo[2] * (q[2] - self.cero[2]) * 360.0 / self.paso_husillo_mm
             if self.husillo_medido else None)
        return [a, s, t]

    def a_junta(self, m) -> list:
        """Motor (grados, grados, mm) -> [theta1 rad, theta2 rad, d3 mm]. d3 None sin paso."""
        t1 = math.radians(m[0] / (self.signo[0] * self.reduccion[0]) + self.cero[0])
        t2 = math.radians(m[1] / (self.signo[1] * self.reduccion[1]) + self.cero[1])
        d3 = (m[2] / self.signo[2] * self.paso_husillo_mm / 360.0 + self.cero[2]
              if self.husillo_medido and m[2] is not None else None)
        return [t1, t2, d3]

    def fuera(self, motor) -> list[str]:
        """Que motores se salen de su recorrido, dicho en castellano."""
        motivos = []
        for nombre, v, (lo, hi), u in zip(MOTORES, motor, self.limites, UNIDADES):
            if v is None:
                continue
            if v < lo - 1e-6 or v > hi + 1e-6:
                motivos.append(f'{nombre} tendria que ir a {v:+.1f}{u} y su recorrido '
                               f'es [{lo:g}, {hi:g}]{u}')
        return motivos


def segundos(delta: float, vel: float, i: int) -> float:
    """Lo que tarda el motor i en moverse `delta` (sus unidades) a `vel` (RPM o mm/min)."""
    return abs(delta) / (POR_SEGUNDO[i] * vel) if vel > 0 else 0.0


def vector_placa(destino, actual, cal: Calibracion) -> list[float]:
    """El vector que se manda a main.py, con las velocidades FIJAS de cada motor:

        [pos_alvin, 14, pos_simon, 30, pos_teodoro, 60]

    (RPM, RPM y mm/min). destino: motor (Teodoro None si falta el paso: se
    queda donde esta, que es `actual`). main.py da por llegado al que ya esta.
    """
    vec = []
    for i in range(3):
        pos = destino[i] if destino[i] is not None else actual[i]
        vec += [round(float(pos), 3), cal.velocidades[i]]
    return vec


@dataclass
class Veredicto:
    punto: tuple
    alcanzable: bool
    codo: str | None = None
    junta: list | None = None          # [theta1 rad, theta2 rad, d3 mm]
    motor: list | None = None          # Alvin y Simon en grados, Teodoro en mm
    motivos: list = field(default_factory=list)
    z_comprobada: bool = True

    def resumen(self) -> str:
        x, y, z = self.punto
        if self.alcanzable:
            t1, t2, d3 = self.junta
            m = self.motor
            teo = f'{m[2]:+.1f} mm' if m[2] is not None else 'sin calcular (falta el paso)'
            aviso = ('' if self.z_comprobada
                     else ' | OJO: z sin comprobar, falta el paso del husillo')
            return (f'({x:.1f}, {y:.1f}, {z:.1f}) mm ALCANZABLE, codo {self.codo} | '
                    f'junta {math.degrees(t1):+.1f}° {math.degrees(t2):+.1f}° {d3:.1f} mm | '
                    f'Alvin {m[0]:+.1f}° Simon {m[1]:+.1f}° Teodoro {teo}{aviso}')
        return f'({x:.1f}, {y:.1f}, {z:.1f}) mm NO ALCANZABLE: ' + '; '.join(self.motivos)


def alcanzar(P, cal: Calibracion, robot: R.Robot = R.ROBOT,
             motor_actual=None) -> Veredicto:
    """Los angulos para llevar la punta a P = (x, y, z) mm, y si caben.

    Prueba las dos ramas del codo. Si valen las dos, se queda con la que menos
    obliga a girar desde motor_actual (o con 'down' si no se sabe).
    """
    P = tuple(float(v) for v in P)
    validas, motivos = [], []
    for codo in ('down', 'up'):
        try:
            q = R.ik_scara_rrp(P, robot, codo)     # alcance y topes del CAD
        except R.RutaError as e:
            motivos.append(f'codo {codo}: {e}')
            continue
        m = cal.a_motor(q)
        malos = cal.fuera(m)
        if malos:
            motivos.append(f'codo {codo}: ' + ', '.join(malos))
        else:
            validas.append((codo, q, m))
    if not validas:
        # Si el punto ni siquiera esta en el anillo, las dos ramas dicen lo
        # mismo: basta con decirlo una vez.
        if len(motivos) == 2 and motivos[0].split(': ', 1)[1] == motivos[1].split(': ', 1)[1]:
            motivos = [motivos[0].split(': ', 1)[1]]
        return Veredicto(P, False, motivos=motivos, z_comprobada=cal.husillo_medido)
    if motor_actual is not None and len(validas) > 1:
        validas.sort(key=lambda v: sum(abs(a - b) for a, b in zip(v[2][:2], motor_actual[:2])))
    codo, q, m = validas[0]
    return Veredicto(P, True, codo, [float(v) for v in q], m,
                     z_comprobada=cal.husillo_medido)


def contorno_planta(cal: Calibracion, robot: R.Robot = R.ROBOT, n: int = 40) -> np.ndarray:
    """Borde de la zona que alcanza la punta, visto desde arriba (x, y en mm).

    Es la imagen del borde del rectangulo de limites de Alvin y Simon, recortado
    a los topes del CAD (Simon llega a -280 grados, pero el codo solo a 150) y,
    si cruza el brazo estirado (la singularidad), a la rama down (lado_down).
    """
    m0 = cal.a_motor([robot.lim[0][0], robot.lim[1][0], 0.0])
    m1 = cal.a_motor([robot.lim[0][1], robot.lim[1][1], 0.0])
    (a0, a1), (s0, s1) = [(max(lo, min(p, q)), min(hi, max(p, q)))
                          for (lo, hi), p, q in zip(cal.limites[:2], m0[:2], m1[:2])]
    t2 = sorted(cal.a_junta([0.0, s, None])[1] for s in (s0, s1))
    t2 = lado_down(robot, *t2)
    s0, s1 = sorted(cal.a_motor([0.0, v, 0.0])[1] for v in t2)
    borde = ([(a0, s0 + (s1 - s0) * k / n) for k in range(n + 1)]
             + [(a0 + (a1 - a0) * k / n, s1) for k in range(n + 1)]
             + [(a1, s1 - (s1 - s0) * k / n) for k in range(n + 1)]
             + [(a1 - (a1 - a0) * k / n, s0) for k in range(n + 1)])
    pts = []
    for ma, ms in borde:
        t1, t2, _ = cal.a_junta([ma, ms, None])
        pts.append(R.fk_scara_rrp([t1, t2, 0.0], robot)[:2])
    return np.array(pts)


def rango_z(cal: Calibracion, robot: R.Robot = R.ROBOT):
    """(z_min, z_max) en mm que permite Teodoro, o None si falta el paso."""
    if not cal.husillo_medido:
        return None
    lo, hi = cal.limites[2]
    zs = []
    for v in (lo, hi):
        d3 = cal.a_junta([0.0, 0.0, v])[2]
        zs.append(robot.z_ref + robot.z_dir * d3)
    return min(zs), max(zs)
