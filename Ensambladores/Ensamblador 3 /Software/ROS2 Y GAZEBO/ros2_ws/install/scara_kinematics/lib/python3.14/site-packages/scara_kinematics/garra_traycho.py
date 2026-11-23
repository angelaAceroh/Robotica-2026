"""El traycho con la garra: en cada punto, abre o cierra (GARRA.md).

Lo pidio Sergio el 19/11: «en la posicion home arranque con la garra abierta
[...] en la p1 haga chocolate, p2 abriendo mano, p3 agarra chocolate, p4
abriendo, p5 chocolate, p6 abriendo». Con las ordenes de garra.py (en la placa):
1 abre, 0 cierra.

El 20/11, con el brazo delante: «en p2 debe cerrar y esta abriendo, esta al
reves en todos»: P1..P6, al reves que el 19/11. HOME sigue abierta.

    al salir   1   abierta (es como esta en HOME)
    P1         1   abre
    P2         0   cierra: agarra el chocolate
    P3         1   abre
    P4         0   cierra
    P5         1   abre
    P6         0   cierra
    P7, P8     -   sigue cerrada, y vuelve a P1 (que abre)

P1 es el HOME del traycho por juntas (juntas.puntos_mm): la primera ida es a P1
y ahi abre. La garra se mueve al LLEGAR al punto: en los que estan abajo,
despues de bajar y antes de volver a subir. Para cambiar donde abre o cierra,
ACCIONES.

Sin ROS: lo usa garra_gui, y los tests con un control de mentira.
"""
from __future__ import annotations

import re
import time

import numpy as np

from scara_kinematics import ruta as R
from scara_kinematics import traycho as T

ABRIR, CERRAR = '1', '0'
QUE = {ABRIR: 'abre', CERRAR: 'cierra'}
CONTESTA = {ABRIR: 'GARRA abierta', CERRAR: 'GARRA cerrada'}   # lo que dice garra.py
ACCIONES = {1: ABRIR, 2: CERRAR, 3: ABRIR, 4: CERRAR, 5: ABRIR, 6: CERRAR}
AL_SALIR = ABRIR
ESPERA_S = 2.0           # lo que tarda el servo en llegar, antes de seguir: los 2 s
                         # del codigo probado (va de golpe y con poca fuerza). SUPUESTO
CONTESTA_S = 5.0         # la placa tiene que decir «GARRA ...» en esto (de golpe
                         # contesta ya; con rampa, al llegar: 2 s de 10 a 50 grados)

_PUNTO = re.compile(r'^P(\d+)(?: \((\d+)/(\d+)\))?$')


def accion_de(nombre: str, acciones=None):
    """La orden de la garra al acabar el tramo `nombre`, o None.

    Solo al llegar AL punto: 'P2' (la bajada) o el ultimo trozo de un ir
    partido, 'P1 (2/2)'. 'P2 encima', 'P2 sube' y 'P1 (1/2)' no."""
    acciones = ACCIONES if acciones is None else acciones
    m = _PUNTO.match(nombre)
    if not m or (m.group(2) is not None and m.group(2) != m.group(3)):
        return None
    return acciones.get(int(m.group(1)))


def resumen(acciones=None) -> str:
    acciones = ACCIONES if acciones is None else acciones
    return (f'{QUE[AL_SALIR]} al salir; '
            + ', '.join(f'{QUE[a]} en P{k}' for k, a in sorted(acciones.items())))


class RecorridoGarra(T.Recorrido):
    """El Recorrido del traycho (traycho.py) con la garra en medio.

    Al salir, y al acabar cada tramo que tenga accion (accion_de), manda la
    orden con `mandar_garra(texto) -> bool` (a scara/placa/orden, sin cortar el
    traycho), espera a que la placa diga «GARRA abierta|cerrada» por /rosout y
    luego ESPERA_S a que el servo llegue. Mientras, el brazo se queda quieto en
    el punto (juntas_nodo lo sujeta). Se corta si la placa no contesta o dice
    que no tiene la garra.
    """

    def __init__(self, tramos, mandar, decir, mandar_garra, acciones=None,
                 al_salir=AL_SALIR, espera: float = ESPERA_S):
        super().__init__(tramos, mandar, decir)
        self.mandar_garra = mandar_garra
        self.acciones = ACCIONES if acciones is None else acciones
        self.espera = float(espera)
        self.garra = None             # (orden, donde, despues) mientras mueve la garra
        self.mandada = False
        self.confirmada = False
        if al_salir is not None:
            self._a_garra(al_salir, 'al salir de HOME', 'mandar')

    def _a_garra(self, orden: str, donde: str, despues: str):
        self.garra = (orden, donde, despues)
        self.fase = 'garra'
        self.mandada = self.confirmada = False

    def paso(self, ahora: float | None = None):
        ahora = time.monotonic() if ahora is None else ahora
        super().paso(ahora)            # lee lo que haya llegado (-> _leer) y manda el ir
        if self.fase != 'garra':
            return
        orden, donde, despues = self.garra
        if not self.mandada:
            if not self.mandar_garra(orden):
                self.cortar('puente_placa no escucha (la garra)')
                return
            self.mandada = True
            self.limite = ahora + CONTESTA_S
            self.decir(20, f'   garra {donde}: {orden} ({QUE[orden]})')
        elif not self.confirmada and ahora > self.limite:
            self.cortar(f'la placa no contesta a la garra en {CONTESTA_S:g} s '
                        '(¿lleva garra.py y el main.py con la garra? GARRA.md)')
        elif self.confirmada and ahora >= self.limite:
            self.garra = None
            self.fase = despues
            if despues == 'hecho':
                self.decir(20, '   traycho terminado: ha vuelto a P1')
            else:
                super().paso(ahora)    # el siguiente ir, ya

    def _leer(self, nodo: str, nivel: int, texto: str, ahora: float):
        if self.fase == 'garra':
            if nodo == T.NODO_PLACA:
                orden = self.garra[0]
                if self.mandada and not self.confirmada and CONTESTA[orden] in texto:
                    self.confirmada = True
                    self.limite = ahora + self.espera
                    return
                if '[x]' in texto or nivel >= 40 or 'no conozco' in texto:
                    self.cortar(f'la garra: {texto}')
                    return
            super()._leer(nodo, nivel, texto, ahora)    # los motores sueltos, etc.
            return
        antes = self.i
        terminado = self.fase == 'marcha' and nodo == T.NODO_CONTROL and texto.startswith(
            'ir terminada')
        if terminado and self.i == len(self.tramos) - 1:
            # El ultimo: el «terminado» lo dice paso() despues de la garra, si la hay
            orden = accion_de(self.tramos[self.i][0], self.acciones)
            if orden is not None:
                m = re.search(r'medio ([\d.]+) mm, maximo ([\d.]+) mm', texto)
                self.decir(20, f'   traycho {self.nombre()}: llega en {self.dura:.1f} s'
                               + (f' | error medio {m.group(1)} mm, maximo {m.group(2)} mm'
                                  if m else ''))
                self.i += 1
                self._a_garra(orden, f'en {self.tramos[antes][0]}', 'hecho')
                return
        super()._leer(nodo, nivel, texto, ahora)
        if terminado and self.i == antes + 1 and self.fase == 'mandar':
            orden = accion_de(self.tramos[antes][0], self.acciones)
            if orden is not None:
                self._a_garra(orden, f'en {self.tramos[antes][0]}', 'mandar')

# Con la garra montada, la mano llega al suelo con Teodoro a -27.8 (19/11: el traycho
# bajaba a -51.5 hacia P2 y Sergio lo paro ahi; «esta lo mas abajo posible»), y si
# baja mas aplasta los cables del servo y la placa se reinicia sin parar. Por eso
# traycho_garra.launch.py pone teodoro_min a esto, y los puntos de abajo (P2..P7) no
# bajan de lo que deja. SUPUESTO: 3 mm por encima.
TEODORO_SUELO_MM = -25.0


def z_suelo(robot) -> float:
    """Lo mas bajo que deja el tope de Teodoro de `robot` (mm de z de la punta)."""
    lo, hi = robot.lim[2]
    return float(min(R.fk_scara_rrp([0.0, 0.0, lo], robot)[2],
                     R.fk_scara_rrp([0.0, 0.0, hi], robot)[2]))


def con_suelo(pts, z_min: float) -> np.ndarray:
    """Los puntos del traycho, ninguno por debajo de z_min (la x y la y, igual)."""
    p = np.array(pts, dtype=float)
    p[:, 2] = np.maximum(p[:, 2], z_min + 1e-6)
    return p
