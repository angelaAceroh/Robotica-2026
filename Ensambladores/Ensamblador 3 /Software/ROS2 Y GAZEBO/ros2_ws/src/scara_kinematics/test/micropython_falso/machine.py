"""`machine` de mentira: la ESP32 con sus tres motores, para correr main.py en el PC.

main.py hace `from machine import Pin, PWM, Timer`; con esta carpeta en el
PYTHONPATH le llega esto en vez de la placa. Cada motor tiene inercia (rpm de
primer orden, 90 RPM con el PWM a tope) y su encoder genera los flancos en
cuadratura, uno a uno, llamando al IRQ como lo haria el de verdad.

Fallos que se pueden provocar con la variable de entorno FALLOS:
    simon_b     el canal B de Simon no cambia nunca (como el 11 de septiembre)
    alvin_inv   Alvin gira al reves de lo que cuenta su encoder

Tambien pone time.ticks_ms y time.ticks_diff, que en CPython no existen.
"""
import gc
import os
import threading
import time

time.ticks_ms = lambda: int(time.monotonic() * 1000)
time.ticks_diff = lambda a, b: a - b

PINES = {}
FALLOS = os.environ.get('FALLOS', '')


class Pin:
    OUT = 1
    IN = 0
    IRQ_RISING = 1
    IRQ_FALLING = 2

    def __init__(self, n, mode=None):
        self.n, self.v, self.h = n, 0, None
        PINES[n] = self

    def value(self, v=None):
        if v is None:
            return self.v
        self.v = v

    def irq(self, trigger=None, handler=None):
        self.h = handler


class PWM:
    def __init__(self, pin, freq=0):
        self.pin, self.d = pin, 0

    def duty(self, d=None):
        if d is None:
            return self.d
        self.d = d


class Timer:
    PERIODIC = 0

    def __init__(self, n):
        pass

    def init(self, period=10, mode=None, callback=None):
        def bucle():
            while True:
                time.sleep(period / 1000)
                callback(self)
        threading.Thread(target=bucle, daemon=True).start()


# nombre, IN1, IN2, ENA, ENC_A, ENC_B: los mismos pines que la tabla de main.py
# (los de Alvin.py, Simon.py y Teodoro.py de Sergio)
SIM = [('alvin', 4, 16, 17, 26, 25), ('simon', 18, 19, 5, 12, 13),
       ('teodoro', 21, 22, 23, 27, 14)]
ORDEN = [0, 1, 3, 2]          # +1 en la tabla PASOS: 00 -> 01 -> 11 -> 10
PULSOS = int(os.environ.get('PULSOS', '5500'))   # por vuelta del eje del motor


def _fisica():
    pwms = {}
    est = {s[0]: {'rpm': 0.0, 'acum': 0.0, 'k': 0} for s in SIM}
    dt = 0.001
    while True:
        time.sleep(dt)
        for nombre, i1, i2, ena, a, b in SIM:
            if a not in PINES or ena not in PINES:
                continue
            if nombre not in pwms:
                for o in gc.get_objects():
                    if isinstance(o, PWM) and o.pin.n == ena:
                        pwms[nombre] = o
                if nombre not in pwms:
                    continue
            sentido = PINES[i1].v - PINES[i2].v
            if FALLOS == 'alvin_inv' and nombre == 'alvin':
                sentido = -sentido
            e = est[nombre]
            e['rpm'] += (90.0 * sentido * (pwms[nombre].d or 0) / 1023.0 - e['rpm']) * dt / 0.06
            e['acum'] += e['rpm'] / 60.0 * PULSOS * dt
            while abs(e['acum']) >= 1:
                paso = 1 if e['acum'] > 0 else -1
                e['acum'] -= paso
                e['k'] = (e['k'] + paso) % 4
                na, nb = ORDEN[e['k']] >> 1, ORDEN[e['k']] & 1
                if FALLOS == 'simon_b' and nombre == 'simon':
                    nb = 0
                for pin, nuevo in ((PINES[a], na), (PINES[b], nb)):
                    if pin.v != nuevo:
                        pin.v = nuevo
                        if pin.h:
                            pin.h(pin)


threading.Thread(target=_fisica, daemon=True).start()
