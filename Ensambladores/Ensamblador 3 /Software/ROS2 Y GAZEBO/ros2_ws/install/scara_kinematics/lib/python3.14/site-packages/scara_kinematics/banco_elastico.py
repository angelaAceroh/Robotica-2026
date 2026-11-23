"""Banco del lazo con la transmision ELASTICA (24/10): el encoder esta en el
motor, y entre el motor y el brazo hay algo que cede (correa, engrane).

En el brazo, yendo a P3 por juntas (Alvin de -13 a -31 a 84 grados/s), el
motor llego a -30.9 y luego, frenado y sin empujar, volvio solo a -28.1 en 2 s;
y mientras tanto los pulsos de 20 ms lo movian medio grado y volvia (subia y
bajaba +-0.5 cada 60 ms, el ciclo de los pulsos). Suelto en -22.9 o en -28.5 no
volvia: no tira nada de fuera. Lo que se ve es el motor adelantado al brazo,
con la transmision tensa, y volviendo hacia el en cuanto deja de empujar.

Aqui, dos cuerpos en grados del eje del motor:

    motor   empujado por el puente (como banco_lazo.Fisica), poco roce
    muelle  K * (motor - brazo) + C * (velocidad del motor - la del brazo)
    brazo   con su arranque y su roce, como el Alvin de banco_lazo

Los numeros, SUPUESTO: que el conjunto arranque con 0.66-0.70 (lo medido) y
que con el motor parado el muelle se lleve un par de grados.
"""
from __future__ import annotations

import numpy as np

from scara_kinematics import banco_lazo as B

ELASTICO = {
    'Alvin': dict(arranque_motor=(0.12, 0.12), roce_motor=(0.06, 0.06), tau_motor=0.02,
                  k=0.30, c=0.002, arranque=(0.58, 0.58), roce=(0.26, 0.26), tau=0.05,
                  w_max=250.0, retraso=0.01),
}


class FisicaElastica:
    """Motor y brazo unidos por un muelle. `theta` es el motor (el encoder)."""

    def __init__(self, arranque_motor, roce_motor, tau_motor, k, c, arranque, roce, tau,
                 w_max, retraso=0.0):
        self.arr_m, self.roce_m, self.tau_m = tuple(arranque_motor), tuple(roce_motor), tau_motor
        self.k, self.c = float(k), float(c)
        self.arr_b, self.roce_b, self.tau_b = tuple(arranque), tuple(roce), tau
        self.w_max = float(w_max)
        self.retraso = float(retraso)
        self._cola: list = []
        self.theta = 0.0            # motor
        self.w = 0.0
        self.brazo = 0.0
        self.w_brazo = 0.0

    def poner(self, theta: float):
        self.theta = self.brazo = theta
        self.w = self.w_brazo = 0.0

    def puente(self, in1: int, in2: int, duty: float) -> float:
        d = max(0.0, min(1.0, duty / 1023.0))
        if d == 0.0:
            return 0.0
        if in1 == in2:
            return -d * self.w / self.w_max
        return (d if in1 else -d) - self.w / self.w_max

    @staticmethod
    def _cuerpo(w, f, arranque, roce, a_por_f, dt):
        """Nueva velocidad de un cuerpo con roce seco: (w, se mueve)."""
        if w == 0.0:
            if f > arranque[0]:
                f -= roce[0]
            elif f < -arranque[1]:
                f += roce[1]
            else:
                return 0.0
        else:
            f -= roce[0] if w > 0.0 else -roce[1]
        nuevo = w + f * a_por_f * dt
        if w != 0.0 and nuevo * w < 0.0:
            nuevo = 0.0
        return nuevo

    def paso(self, dt: float, in1: int, in2: int, duty: float):
        if self.retraso > 0.0:
            self._cola.append((in1, in2, duty))
            n = int(round(self.retraso / dt))
            in1, in2, duty = self._cola[0] if len(self._cola) > n else (0, 0, 0)
            if len(self._cola) > n:
                self._cola.pop(0)
        muelle = self.k * (self.theta - self.brazo) + self.c * (self.w - self.w_brazo)
        wm = self._cuerpo(self.w, self.puente(in1, in2, duty) - muelle, self.arr_m,
                          self.roce_m, self.w_max / self.tau_m, dt)
        wb = self._cuerpo(self.w_brazo, muelle, self.arr_b, self.roce_b,
                          self.w_max / self.tau_b, dt)
        self.theta += 0.5 * (self.w + wm) * dt
        self.brazo += 0.5 * (self.w_brazo + wb) * dt
        self.w, self.w_brazo = wm, wb


def banco(nombre='Alvin', firmware=None, pos=-13.2, ganancias=None, **cambios) -> B.Banco:
    """Un banco_lazo.Banco con la fisica elastica, quieto en `pos` (sus unidades)."""
    b = B.Banco(nombre, firmware, pos=pos, activar=False, ganancias=ganancias)
    datos = dict(ELASTICO[nombre])
    datos.update(cambios)
    b.fis = FisicaElastica(**datos)
    b.fis.poner(pos * b.m.f)
    b._encoder()
    b.m.ref = b.m.pos_ant = b.m.posicion()
    b.m.activar()
    return b


def ensayo_p3(nombre='Alvin', firmware=None, desde=-13.2, hasta=-31.0, vel=84.0,
              sujetar=4.0, ganancias=None, **cambios) -> dict:
    """El paso de P3 del 24/10: de `desde` a `hasta` a `vel` y quieto `sujetar` s.
    Lo que le falta al brazo y al motor al final, y si salto un vigilante."""
    b = banco(nombre, firmware, desde, ganancias, **cambios)
    obj = b.mover(hasta - desde, vel, sujetar=sujetar)
    f = b.m.f
    ultimo = b.desde(b.t - 1.0)
    return dict(falta_brazo=(obj - b.fis.brazo / f) * np.sign(hasta - desde),
                falta_motor=(obj - b.fis.theta / f) * np.sign(hasta - desde),
                vaiven=float(ultimo[:, 1].max() - ultimo[:, 1].min()) if len(ultimo) else 0.0,
                fallo=b.fallo, banco=b)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description='el paso de P3 con la transmision elastica')
    ap.add_argument('--firmware', default=None)
    ap.add_argument('--g', nargs=3, action='append', default=[],
                    metavar=('MOTOR', 'CLAVE', 'VALOR'), help="como la 'g' de la placa")
    args = ap.parse_args(argv)
    gan = {clave: float(valor) for _, clave, valor in args.g}
    for desde, hasta in ((-13.2, -31.0), (-9.7, -26.8), (-21.4, -28.0), (0.0, 18.0)):
        r = ensayo_p3('Alvin', args.firmware, desde, hasta, ganancias=gan)
        print(f'Alvin {desde:+6.1f} -> {hasta:+6.1f} | le falta: brazo {r["falta_brazo"]:+.2f}, '
              f'motor {r["falta_motor"]:+.2f} | vaiven el ultimo segundo {r["vaiven"]:.2f}'
              + (f' | FALLO {r["fallo"]}' if r['fallo'] else ''))


if __name__ == '__main__':
    main()
