"""«traycho»: la trayectoria del ejemplo de control cinematico que paso Sergio,
en su brazo. La lanza el boton «traycho» de la ventana (traycho_gui).

Sus ocho puntos, en cm, tal cual estan en su codigo (el de otro SCARA, con
L1 = 0.20, a1 = 0.165 y a2 = 0.135 m):

    P1 [5.5, -6.5, 21.5]   P2 [-10, -15, 10]   P3 [19, 6, 15]   P4 [0, -15, 10]
    P5 [23.5, 6, 15]       P6 [10, -15, 10]    P7 [28.5, 6, 15] P8 = P1

Aquel robot llega a 30 cm de la base. Este, de 25 cm (en HOME, con el codo
doblado 120 grados, no se acerca mas) a 50, y con los recorridos de los motores
solo por delante: tal cual, casi ninguno cabe. Se mueven todos juntos
DESPLAZAMIENTO_CM, sin girarlos ni escalarlos, asi que la figura en planta y las
distancias entre puntos son las de su codigo. Con el HOME en (0, -270) quedan a
21 grados de junta de los topes en el punto mas justo. SUPUESTO: si los quieres
en otro sitio, cambialo (el test comprueba que siguen cabiendo).

Las z, no tal cual (23/10: «bajaste mucho a Teodoro»): a ESCALA_Z y con P1,
el mas alto, arriba del todo (Z_ARRIBA_MM, Teodoro a 0.5 mm de su tope). Asi
P2, P4 y P6 quedan 115 mm por debajo y P3, P5 y P7, 65: las de su codigo. Por el soporte cilindrico que lleva ahora: a media
escala choco con el (lo paro en -47 mm) y a un cuarto, bajando en P4, casi
(lo paro en -10 mm: «el soporte, en todos»; pero ese dia la placa habia perdido
el HOME y Teodoro se fijo a 0 donde estaba, no arriba). Hasta el 29/10 iban a
0.04 (4.6 mm); entonces, «casi no esta bajando»: a 0.10 (11.5 mm), y luego «se
supone que debe bajar casi todo y bajo menos de un cuarto»: a 1, como su codigo
(eligio eso con el soporte aun puesto; los puntos estan 9 cm mas afuera que el
23/10).

De un punto a otro va en recta (`ir` de control_nodo). Su codigo interpola las
juntas, asi que alli la punta hace curvas entre puntos. Si en una recta alguna
junta gira mas de GRADOS_MAX («que no haga pasos rapidos a mas de 20 grados»),
se parte en trozos iguales que no pasan de ahi.
"""
from __future__ import annotations

import math
import queue
import re
import time

import numpy as np

from scara_kinematics import ruta as R

PUNTOS_CM = ((5.5, -6.5, 21.5), (-10.0, -15.0, 10.0), (19.0, 6.0, 15.0), (0.0, -15.0, 10.0),
             (23.5, 6.0, 15.0), (10.0, -15.0, 10.0), (28.5, 6.0, 15.0), (5.5, -6.5, 21.5))
DESPLAZAMIENTO_CM = (8.0, -29.0)       # x, y. SUPUESTO: ver arriba
ESCALA_Z = 1.0                         # las bajadas de Teodoro: las de su codigo (29/10)
Z_ARRIBA_MM = 220.0                    # P1: Teodoro arriba (z = 220.51 en 0), menos el margen
GRADOS_MAX = 20.0                      # lo que puede girar una junta en un ir
MUESTRAS = 40                          # puntos de cada recta que se comprueban
ESPERA_EMPIEZA_S = 5.0                 # control_nodo tiene que decir «ir: N s» en esto
NODO_CONTROL = 'control'
NODO_PLACA = 'puente_placa'


def puntos_mm(desplazamiento_cm=DESPLAZAMIENTO_CM) -> np.ndarray:
    """Los ocho puntos de su codigo, desplazados, en mm; las z, a ESCALA_Z por
    debajo de Z_ARRIBA_MM."""
    p = np.array(PUNTOS_CM, dtype=float) * 10.0
    p[:, 0] += desplazamiento_cm[0] * 10.0
    p[:, 1] += desplazamiento_cm[1] * 10.0
    p[:, 2] = Z_ARRIBA_MM - (p[:, 2].max() - p[:, 2]) * ESCALA_Z
    return p


def _juntas(p, robot: R.Robot) -> np.ndarray:
    """q1, q2 en grados, en la rama down (la de HOME: control_nodo no cambia de rama)."""
    return np.degrees(R.ik_scara_rrp(p, robot, 'down')[:2])


def tramos(pts, robot: R.Robot, desde=None, grados_max: float = GRADOS_MAX) -> list:
    """[(nombre, punto mm)]: un ir por punto, partido en trozos iguales si alguna
    junta gira mas de grados_max. `desde` es donde esta la punta: la primera
    recta, hasta P1, sale de ahi. Levanta RutaError si algo no cabe (los puntos o
    las rectas entre ellos, con los topes de `robot`)."""
    pts = [np.asarray(p, dtype=float) for p in pts]
    a = pts[0] if desde is None else np.asarray(desde, dtype=float)
    salida = []
    for i, b in enumerate(pts, 1):
        for s in np.linspace(0.0, 1.0, MUESTRAS):
            R.ik_scara_rrp(a + (b - a) * s, robot, 'down')       # levanta si no cabe
        n = max(1, math.ceil(np.max(np.abs(_juntas(b, robot) - _juntas(a, robot)))
                             / grados_max))
        while True:
            trozos = [a + (b - a) * (k / n) for k in range(n + 1)]
            q = [_juntas(p, robot) for p in trozos]
            if all(np.max(np.abs(q1 - q0)) <= grados_max for q0, q1 in zip(q, q[1:])):
                break
            n += 1
        for k in range(1, n + 1):
            salida.append((f'P{i}' if n == 1 else f'P{i} ({k}/{n})', trozos[k]))
        a = b
    return salida


def limite_marcha(dura: float) -> float:
    """Lo que se espera a un ir que dijo durar `dura` s. 29/10, con las z de su
    codigo: Teodoro bajo 115 mm en 227 s (va a ~0.5 mm/s, no al 1 que se le
    pide) y con 2*dura + 15 (245 s) la subida iba justa. Mas que lo que aguanta
    juntas.Juntas (3*dura + 10, y luego corrige): que corte el, que sabe por que."""
    return 3.0 * dura + 30.0


def orden_ir(p_mm) -> str:
    return 'ir ' + ' '.join(f'{c:.1f}' for c in p_mm)


class Recorrido:
    """Manda los ir de uno en uno y pasa al siguiente cuando control_nodo dice
    «ir terminada» por /rosout.

    `mandar(texto) -> bool` publica en scara/control/orden; `decir(nivel, texto)`
    lo cuenta (nivel 20, 30, 40 como los de /rosout). `aviso()` se puede llamar
    desde el hilo de ROS; `paso()`, desde el de la ventana, cada poco.

    Se corta si control_nodo no lo hace, si se para (Parar, Esc, otra orden) o
    si la placa suelta los motores.
    """

    def __init__(self, tramos, mandar, decir):
        self.tramos = list(tramos)
        self.mandar = mandar
        self.decir = decir
        self.i = 0
        self.fase = 'mandar'           # mandar | empieza | marcha | hecho | cortado
        self.limite = 0.0
        self.dura = 0.0
        self.porque = ''
        self.ultimo_fallo = ''
        self._avisos: queue.Queue = queue.Queue()

    def activo(self) -> bool:
        return self.fase not in ('hecho', 'cortado')

    def nombre(self) -> str:
        return self.tramos[min(self.i, len(self.tramos) - 1)][0]

    def aviso(self, nodo: str, nivel: int, texto: str):
        self._avisos.put((nodo, nivel, texto))

    def cortar(self, porque: str):
        if self.activo():
            self.fase, self.porque = 'cortado', porque
            self.decir(40, f'traycho cortado en {self.nombre()}: {porque}')

    def paso(self, ahora: float | None = None):
        ahora = time.monotonic() if ahora is None else ahora
        while self.activo():
            try:
                nodo, nivel, texto = self._avisos.get_nowait()
            except queue.Empty:
                break
            self._leer(nodo, nivel, texto, ahora)
        if self.fase == 'mandar':
            nombre, p = self.tramos[self.i]
            orden = orden_ir(p)
            if not self.mandar(orden):
                self.cortar('control_nodo no escucha')
                return
            self.decir(20, f'traycho {nombre} ({self.i + 1} de {len(self.tramos)}): {orden}')
            self.fase, self.limite = 'empieza', ahora + ESPERA_EMPIEZA_S
        elif self.fase == 'empieza' and ahora > self.limite:
            self.cortar('control_nodo no contesta')
        elif self.fase == 'marcha' and ahora > self.limite:
            self.cortar(f'no acaba en {limite_marcha(self.dura):.0f} s')

    def _leer(self, nodo: str, nivel: int, texto: str, ahora: float):
        if nodo == NODO_PLACA:
            # Lo mismo que control_puntos: el porque llega antes que el PARADO
            if nivel >= 40 or texto.startswith('placa: VIGILANTE'):
                self.ultimo_fallo = texto.split(': ', 1)[-1]
            if 'motores sueltos' in texto:
                self.cortar('la placa ha soltado los motores'
                            + (f': {self.ultimo_fallo}' if self.ultimo_fallo else ''))
            return
        if nodo != NODO_CONTROL:
            return
        if self.fase == 'empieza':
            if texto.startswith('«ir') and 'no sale' in texto:
                self.cortar(f'control_nodo no lo hace: {texto.split(": ", 1)[-1]}')
                return
            m = re.match(r'ir: ([\d.]+) s', texto)
            if m:
                self.dura = float(m.group(1))
                self.fase, self.limite = 'marcha', ahora + limite_marcha(self.dura)
        elif self.fase == 'marcha':
            if texto.startswith('ir terminada'):
                m = re.search(r'medio ([\d.]+) mm, maximo ([\d.]+) mm', texto)
                self.decir(20, f'   traycho {self.nombre()}: llega en {self.dura:.1f} s'
                               + (f' | error medio {m.group(1)} mm, maximo {m.group(2)} mm'
                                  if m else ''))
                self.i += 1
                if self.i == len(self.tramos):
                    self.fase = 'hecho'
                    self.decir(20, '   traycho terminado: ha vuelto a P1')
                else:
                    self.fase = 'mandar'
            elif 'no sale' in texto or texto.startswith('parado'):
                self.cortar(texto)
