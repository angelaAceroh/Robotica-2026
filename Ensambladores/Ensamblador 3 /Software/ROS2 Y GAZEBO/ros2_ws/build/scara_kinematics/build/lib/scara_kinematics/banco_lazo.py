"""Banco del lazo de cada motor: el `Motor` de main_control.py DE VERDAD, sin placa
y sin tiempo real, con un motor de mentira parecido a los del brazo.

La placa simulada (placa_simulada.py) es amable: arranca con el 10 % de PWM y
sin empujar se para en seco. Asi no se ve el temblor del brazo: Alvin no
arranca con menos de 700 (21/10) y, parado, se queda oscilando +-1.3 grados
unas 7 veces por segundo con el PWM de +700 a -700 (24/10, grabado con
grabar_placa). Aqui cada motor tiene:

    arranque   parado, con menos fuerza que esto (fraccion del PWM, por sentido)
               no se mueve
    roce       girando, lo que se come la friccion (por sentido): mucho menos
               que el arranque. Por eso, una vez suelto, se embala y se pasa
    w_max      grados de motor por segundo por unidad de PWM libre: girando
               estable va a w_max * (u - roce)
    tau        s, la inercia

El puente del motor (L298N) se modela como es: IN1 != IN2 empuja con ENA;
IN1 == IN2 con ENA > 0 frena (el motor en corto: la fuerza contra el giro sube
con la velocidad); ENA = 0, suelto (solo lo para el roce).

Sin ROS. Lo usan test/test_banco_lazo.py y, para comparar dos firmwares:

    python3 -m scara_kinematics.banco_lazo [--firmware otro_main_control.py]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import types

import numpy as np

# Los numeros del brazo (SUPUESTO lo que no se ha medido). Medido: el arranque
# de Alvin (660-700 en los dos sentidos, 21/10) y el de Simon (hacia - le basta
# 250, hacia + necesita 375-450, 24/10). El roce, w_max y tau, ajustados para
# que el lazo de antes (main_control.py del 23/10) tiemble como Alvin en el
# brazo el 24/10: +-1.3 grados a unos 7 Hz, con el PWM de +700 a -700.
FISICA = {
    'Alvin': dict(arranque=(0.66, 0.66), roce=(0.30, 0.30), w_max=250.0, tau=0.04,
                  retraso=0.01),
    'Simon': dict(arranque=(0.40, 0.25), roce=(0.25, 0.12), w_max=300.0, tau=0.04,
                  retraso=0.01),
    'Teodoro': dict(arranque=(0.40, 0.25), roce=(0.30, 0.15), w_max=150.0, tau=0.04,
                    retraso=0.01),
}
DT_FISICA = 0.0005            # s
PERIODO_S = 0.010             # el lazo de la placa (PERIODO_MS)


def firmware_por_defecto() -> str:
    aqui = os.path.dirname(os.path.abspath(__file__))
    candidatos = [os.path.join(aqui, '..', 'firmware', 'micropython', 'main_control.py')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidatos.append(os.path.join(get_package_share_directory('scara_kinematics'),
                                       'firmware', 'micropython', 'main_control.py'))
    except Exception:
        pass
    for ruta in candidatos:
        if os.path.isfile(ruta):
            return os.path.abspath(ruta)
    raise FileNotFoundError('no encuentro firmware/micropython/main_control.py')


def cargar_firmware(ruta: str | None = None) -> dict:
    """Lo de main_control.py hasta «motores = [...]» (constantes, tablas y la clase
    Motor), con un `machine` cuyos pines guardan su valor."""
    with open(ruta or firmware_por_defecto()) as f:
        codigo = f.read()
    codigo = codigo[:codigo.index('motores = [Motor(*m) for m in MOTORES]')]

    class Pin:
        OUT = IN = IRQ_RISING = IRQ_FALLING = 0

        def __init__(self, n=None, *args, **kwargs):
            self.v = 0

        def value(self, v=None):
            if v is None:
                return self.v
            self.v = v

        def irq(self, **kwargs):
            pass

    class PWM:
        def __init__(self, *args, **kwargs):
            self.d = 0

        def duty(self, d=None):
            if d is None:
                return self.d
            self.d = d

    maquina = types.ModuleType('machine')
    maquina.Pin, maquina.PWM, maquina.Timer = Pin, PWM, object
    antes = sys.modules.get('machine')
    sys.modules['machine'] = maquina
    try:
        g = {'__name__': 'main_control'}
        exec(compile(codigo, ruta or 'main_control.py', 'exec'), g)
    finally:
        if antes is None:
            sys.modules.pop('machine', None)
        else:
            sys.modules['machine'] = antes
    return g


class Fisica:
    """Un motor con su reductora y lo que mueve, en grados del eje del motor."""

    def __init__(self, arranque, roce, w_max, tau, carga: float = 0.0, retraso: float = 0.0):
        self.arranque = tuple(arranque)      # (hacia +, hacia -)
        self.roce = tuple(roce)
        self.w_max = float(w_max)
        self.tau = float(tau)
        self.carga = float(carga)            # fuerza constante (peso), en PWM; + empuja hacia +
        self.w = 0.0                          # grados/s
        self.theta = 0.0                      # grados
        # Lo que manda el puente tarda esto en hacerse fuerza (la bobina del
        # motor, el PWM a 1 kHz): s
        self.retraso = float(retraso)
        self._cola: list = []

    def fuerza(self, in1: int, in2: int, duty: float) -> float:
        """Lo que hace el puente, en unidades de PWM (sin el roce)."""
        d = max(0.0, min(1.0, duty / 1023.0))
        if d == 0.0:
            return self.carga                                # suelto
        if in1 == in2:
            return self.carga - d * self.w / self.w_max      # frenando: en corto
        u = d if in1 else -d
        return self.carga + u - self.w / self.w_max

    def paso(self, dt: float, in1: int, in2: int, duty: float):
        if self.retraso > 0.0:
            self._cola.append((in1, in2, duty))
            n = int(round(self.retraso / dt))
            in1, in2, duty = self._cola[0] if len(self._cola) > n else (0, 0, 0)
            if len(self._cola) > n:
                self._cola.pop(0)
        f = self.fuerza(in1, in2, duty)
        if self.w == 0.0:
            if f > self.arranque[0]:
                f -= self.roce[0]
            elif f < -self.arranque[1]:
                f += self.roce[1]
            else:
                return                                       # pegado
        else:
            f -= self.roce[0] if self.w > 0.0 else -self.roce[1]
        nuevo = self.w + f * self.w_max / self.tau * dt
        if self.w != 0.0 and nuevo * self.w < 0.0:
            nuevo = 0.0                                      # se para; para volver, arranque
        self.theta += 0.5 * (self.w + nuevo) * dt
        self.w = nuevo


class Banco:
    """Un motor de main_control.py con su Fisica. `correr(segundos, v)`: v en sus
    unidades por segundo (grados de junta/s o mm/s), un numero o una funcion del
    tiempo; lo que pasa queda en `traza`: (t, posicion, consigna, pwm), en sus
    unidades."""

    def __init__(self, nombre: str, firmware: str | dict | None = None, fisica=None,
                 pos: float = 0.0, activar: bool = True, ganancias=None, **cambios):
        g = firmware if isinstance(firmware, dict) else cargar_firmware(firmware)
        fila = next(m for m in g['MOTORES'] if m[0] == nombre)
        self.m = m = g['Motor'](*fila)
        for clave, valor in (ganancias or {}).items():      # como 'g <motor> <clave> <valor>'
            setattr(m, clave, valor)
        datos = dict(FISICA[nombre] if fisica is None else fisica)
        datos.update(cambios)
        self.fis = Fisica(**datos)
        self.fis.theta = pos * m.f
        self._encoder()
        m.ref = m.pos_ant = m.posicion()
        if activar:
            m.activar()
        self.t = 0.0
        self.traza: list = []
        self.fallo = None

    def _encoder(self):
        m = self.m
        m.pulsos = int(math.floor(self.fis.theta / 360.0 * m.PULSOS_REV + 0.5))

    def correr(self, segundos: float, v=0.0):
        m = self.m
        sub = int(round(PERIODO_S / DT_FISICA))
        for _ in range(int(round(segundos / PERIODO_S))):
            vel = v(self.t) if callable(v) else v
            m.vel = vel * m.f if m.activo else 0.0
            m.paso(int(PERIODO_S * 1000))
            if m.fallo and self.fallo is None:
                self.fallo = (round(self.t, 2), m.fallo)
                m.soltar()
            for _ in range(sub):
                self.fis.paso(DT_FISICA, m.IN1.value(), m.IN2.value(), m.ENA.duty())
                self.t += DT_FISICA
            self._encoder()
            self.traza.append((self.t, m.medida(), m.ref / m.f, m.pwm))
        return self

    def mover(self, delta: float, vel: float, sujetar: float = 1.0):
        """Como juntas_nodo: vel hacia delta (sus unidades) hasta que la consigna ha
        avanzado delta, y luego 0 durante `sujetar` segundos."""
        m = self.m
        objetivo = m.ref / m.f + delta
        s = 1.0 if delta > 0 else -1.0
        fin = self.t + abs(delta) / vel * 3.0 + 2.0
        while (objetivo - m.ref / m.f) * s > 1e-9 and self.t < fin and m.activo:
            falta = abs(objetivo - m.ref / m.f)
            self.correr(PERIODO_S, s * min(vel, falta / PERIODO_S))
        self.correr(sujetar, 0.0)
        return objetivo

    # ----------------------------------------------------------------- medidas
    def desde(self, t0: float) -> np.ndarray:
        a = np.array(self.traza)
        return a[a[:, 0] >= t0 - 1e-9]


def inversiones(x, umbral: float) -> int:
    """Cambios de sentido de mas de `umbral` (de pico a pico)."""
    n, ext, sentido = 0, x[0], 0
    for v in x[1:]:
        if sentido >= 0 and v > ext:
            ext = v
            sentido = sentido or (1 if v - x[0] > umbral else 0)
        elif sentido <= 0 and v < ext:
            ext = v
            sentido = sentido or (-1 if x[0] - v > umbral else 0)
        if sentido > 0 and ext - v > umbral:
            n, ext, sentido = n + 1, v, -1
        elif sentido < 0 and v - ext > umbral:
            n, ext, sentido = n + 1, v, 1
    return n


def cambios_pwm(pwm) -> int:
    """Veces que el PWM salta de empujar hacia un lado a empujar hacia el otro."""
    s = [1 if p > 0 else -1 for p in pwm if p != 0]
    return sum(1 for a, b in zip(s, s[1:]) if a != b)


# ------------------------------------------------------------------- ensayos
def ensayo_sujetar(nombre, firmware=None, fisica=None, error=1.0, segundos=3.0,
                   ganancias=None, **cambios):
    """Activado con la consigna `error` (sus unidades) por delante y V = 0."""
    b = Banco(nombre, firmware, fisica, pos=-20.0, ganancias=ganancias, **cambios)
    b.m.ref += error * b.m.f
    b.correr(segundos)
    a = b.desde(1.0)
    return dict(inversiones=inversiones(a[:, 1], 0.2), cambios_pwm=cambios_pwm(a[:, 3]),
                error=abs(a[-1, 2] - a[-1, 1]), pico=float(np.max(np.abs(a[:, 2] - a[:, 1]))),
                fallo=b.fallo)


def ensayo_escalones(nombre, firmware=None, fisica=None, vel=None, pasos=None,
                     ganancias=None, **cambios):
    """Pasos como los de juntas_nodo (a vel y luego 0): lo que se pasa, lo que le
    falta al cabo de 1 s y cuanto tiembla parado."""
    b = Banco(nombre, firmware, fisica, pos=-20.0, ganancias=ganancias, **cambios)
    vel = vel or (84.0 if nombre == 'Alvin' else 60.0)
    pasos = pasos or [0.5, -1.0, 2.0, -5.0, 10.0, -0.7, 3.0, -15.0]
    pasado, falta, inv = [], [], 0
    for d in pasos:
        t0 = b.t
        obj = b.mover(d, vel, sujetar=1.2)
        a = b.desde(t0)
        s = 1.0 if d > 0 else -1.0
        pasado.append(max(0.0, float(np.max((a[:, 1] - obj) * s))))
        falta.append(abs(obj - a[-1, 1]))
        inv += inversiones(b.desde(b.t - 0.8)[:, 1], 0.2)
        if b.fallo:
            break
    return dict(se_pasa=max(pasado), falta=max(falta), inversiones=inv, fallo=b.fallo)


def ensayo_despacio(nombre, firmware=None, fisica=None, vel=3.0, segundos=4.0,
                    ganancias=None, **cambios):
    """Velocidad constante y lenta, como control_nodo: el error al seguirla."""
    b = Banco(nombre, firmware, fisica, pos=-20.0, ganancias=ganancias, **cambios)
    b.correr(segundos, vel)
    a = b.desde(1.0)
    e = a[:, 2] - a[:, 1]
    return dict(error_medio=float(np.sqrt(np.mean(e ** 2))), error_max=float(np.max(np.abs(e))),
                cambios_pwm=cambios_pwm(a[:, 3]), fallo=b.fallo)


def variantes(nombre: str):
    """La fisica del motor y otras cerca (lo que no se ha medido puede ser otro)."""
    base = FISICA[nombre]
    yield 'base', dict(base)
    for clave, factores in (('roce', (0.6, 1.3)), ('tau', (0.6, 1.6)), ('w_max', (0.6, 1.5)),
                            ('arranque', (0.9, 1.1))):
        for k in factores:
            d = dict(base)
            v = d[clave]
            d[clave] = tuple(x * k for x in v) if isinstance(v, tuple) else v * k
            if clave == 'roce':        # el roce no pasa del arranque
                d['roce'] = tuple(min(r, a - 0.02) for r, a in zip(d['roce'], d['arranque']))
            yield f'{clave}x{k:g}', d
    for r in (0.0, 0.02):
        yield f'retraso{r:g}', dict(base, retraso=r)


def informe(firmware=None, motores=('Alvin', 'Simon'), ganancias=None) -> str:
    """ganancias: {motor: {clave: valor}}, como 'g' (sin tocar el firmware)."""
    g = cargar_firmware(firmware)
    filas = []
    for nombre in motores:
        gan = (ganancias or {}).get(nombre)
        for etiqueta, fis in variantes(nombre):
            s = ensayo_sujetar(nombre, g, fis, ganancias=gan)
            e = ensayo_escalones(nombre, g, fis, ganancias=gan)
            d = ensayo_despacio(nombre, g, fis, ganancias=gan)
            filas.append(f'{nombre:8s} {etiqueta:12s} | sujetar: inv {s["inversiones"]:3d} '
                         f'pwm+- {s["cambios_pwm"]:3d} err {s["error"]:.2f} | escalones: se pasa '
                         f'{e["se_pasa"]:.2f} falta {e["falta"]:.2f} inv {e["inversiones"]:3d} | '
                         f'despacio: err {d["error_medio"]:.2f} max {d["error_max"]:.2f} '
                         f'pwm+- {d["cambios_pwm"]:3d}'
                         + ''.join(f' | FALLO {x["fallo"]}' for x in (s, e, d) if x['fallo']))
    return '\n'.join(filas)


def main(argv=None):
    ap = argparse.ArgumentParser(description='el lazo de main_control.py contra motores '
                                             'como los del brazo')
    ap.add_argument('--firmware', default=None)
    ap.add_argument('--motores', nargs='+', default=['Alvin', 'Simon'])
    ap.add_argument('--g', nargs=3, action='append', default=[],
                    metavar=('MOTOR', 'CLAVE', 'VALOR'), help="como la 'g' de la placa")
    args = ap.parse_args(argv)
    gan: dict = {}
    for motor, clave, valor in args.g:
        gan.setdefault(motor.capitalize(), {})[clave] = float(valor)
    print(informe(args.firmware, args.motores, gan))


if __name__ == '__main__':
    main()
