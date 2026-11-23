"""La placa de mentira del control cinematico: main_control.py DE VERDAD, corriendo
en el PC, con tres motores simulados detras.

    ros2 run scara_kinematics placa_simulada                        # dice el puerto
    ros2 run scara_kinematics placa_simulada --enlace /tmp/scara_placa
    ros2 launch scara_kinematics control_placa.launch.py simular:=true   # la lanza sola

Es para probar la cadena entera (control_nodo -> puente_placa -> USB ->
main_control.py -> motores) sin el brazo. El programa de la placa no se toca: se
ejecuta tal cual, con un `machine` de mentira en vez de la ESP32 y un puerto
serie falso (pty) en vez del USB.

Cada motor tiene inercia (60 ms), friccion (no arranca con menos del 10 % de PWM
y por debajo del 8 % la reductora lo para en unos 10 ms), 80 RPM con el PWM a tope y un encoder en
cuadratura que llama al IRQ flanco a flanco. Los pines y los pulsos por vuelta
salen de la tabla MOTORES del propio main_control.py (5500 por vuelta del motor).
Es mas amable que el de verdad (sin holguras ni tirones), asi que lo que se mida
aqui es un techo, como Gazebo.

Fallos que se pueden provocar (--fallo):
    alvin_inv           Alvin gira al reves de lo que cuenta su encoder
    simon_sin_encoder   el encoder de Simon no cuenta

    --stdio      habla por la entrada y la salida estandar en vez de un pty (tests)
    --pos A S T  donde estan los motores al arrancar, en las unidades de la placa
                 (grados, grados, mm; por defecto, a media carrera: 50 -100 -50)
"""
from __future__ import annotations

import argparse
import ast
import os
import pty
import select as _select
import sys
import tempfile
import threading
import time
import tty
import types

ORDEN = [0, 1, 3, 2]          # +1 en la tabla PASOS: 00 -> 01 -> 11 -> 10
RPM_MAX = 80.0                # con el PWM a tope
U_ARRANQUE = 0.10             # parado, con menos que esto no arranca
U_ROCE = 0.08                 # girando, por debajo de esto se para
TAU = 0.06                    # s, la inercia
TAU_FRENO = 0.008             # s: sin fuerza, la reductora lo para casi en seco


def tabla(codigo: str, nombre: str):
    """La variable `nombre` de un programa de MicroPython, sin ejecutarlo: se
    evaluan, por orden, las asignaciones de arriba que son solo cuentas
    (TEODORO_POR_VUELTA = 3.14159265 * (0.5 * 20) + 10, MOTORES = [...])."""
    vistas: dict = {}
    for nodo in ast.parse(codigo).body:
        if not (isinstance(nodo, ast.Assign) and len(nodo.targets) == 1
                and isinstance(nodo.targets[0], ast.Name)):
            continue
        try:
            valor = eval(compile(ast.Expression(nodo.value), '<tabla>', 'eval'),
                         {'__builtins__': {}}, dict(vistas))
        except Exception:
            continue                       # Pin(...), comprensiones...: no son cuentas
        vistas[nodo.targets[0].id] = valor
        if nodo.targets[0].id == nombre:
            return valor
    raise KeyError(f'el programa no tiene {nombre}')


def _maquina(fallo: str, motores):
    """El modulo `machine`: Pin, PWM y Timer, y la fisica de los motores en un hilo.

    motores: la tabla MOTORES de main_control.py (nombre, IN1, IN2, ENA, ENC_A,
    ENC_B, PULSOS_REV, ...).
    """
    sim = [(m[0].lower(),) + tuple(m[1:7]) for m in motores]
    pines: dict[int, object] = {}
    pwms: dict[int, object] = {}

    class Pin:
        OUT = 1
        IN = 0
        IRQ_RISING = 1
        IRQ_FALLING = 2

        def __init__(self, n, mode=None):
            self.n, self.v, self.h = n, 0, None
            pines[n] = self

        def value(self, v=None):
            if v is None:
                return self.v
            self.v = v

        def irq(self, trigger=None, handler=None):
            self.h = handler

    class PWM:
        def __init__(self, pin, freq=0):
            self.pin, self.d = pin, 0
            pwms[pin.n] = self

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
                siguiente = time.monotonic()
                while True:
                    siguiente += period / 1000.0
                    time.sleep(max(0.0, siguiente - time.monotonic()))
                    callback(self)
            threading.Thread(target=bucle, daemon=True).start()

    def fisica():
        est = {s[0]: {'rpm': 0.0, 'acum': 0.0, 'k': 0} for s in sim}
        antes = time.monotonic()
        while True:
            time.sleep(0.0005)
            ahora = time.monotonic()
            dt, antes = min(ahora - antes, 0.02), ahora
            for nombre, i1, i2, ena, a, b, pulsos in sim:
                if a not in pines or b not in pines or ena not in pwms:
                    continue
                u = (pines[i1].v - pines[i2].v) * (pwms[ena].d or 0) / 1023.0
                if fallo == 'alvin_inv' and nombre == 'alvin':
                    u = -u
                e = est[nombre]
                if abs(e['rpm']) < 0.05 and abs(u) < U_ARRANQUE:
                    e['rpm'] = 0.0                        # pegado por la friccion
                elif abs(u) <= U_ROCE:
                    e['rpm'] -= e['rpm'] * min(dt / TAU_FRENO, 1.0)
                else:
                    meta = RPM_MAX * (abs(u) - U_ROCE) / (1.0 - U_ROCE)
                    meta = meta if u >= 0.0 else -meta
                    e['rpm'] += (meta - e['rpm']) * min(dt / TAU, 1.0)
                if fallo == 'simon_sin_encoder' and nombre == 'simon':
                    continue
                e['acum'] += e['rpm'] / 60.0 * pulsos * dt
                while abs(e['acum']) >= 1.0:
                    paso = 1 if e['acum'] > 0 else -1
                    e['acum'] -= paso
                    e['k'] = (e['k'] + paso) % 4
                    na, nb = ORDEN[e['k']] >> 1, ORDEN[e['k']] & 1
                    for pin, nuevo in ((pines[a], na), (pines[b], nb)):
                        if pin.v != nuevo:
                            pin.v = nuevo
                            if pin.h:
                                pin.h(pin)

    threading.Thread(target=fisica, daemon=True).start()
    # Lo que MicroPython tiene en `time` y CPython no
    t0 = time.monotonic()
    time.ticks_ms = lambda: int((time.monotonic() - t0) * 1000)
    time.ticks_diff = lambda a, b: a - b
    m = types.ModuleType('machine')
    m.Pin, m.PWM, m.Timer = Pin, PWM, Timer
    return m


def _select_amable():
    """`select` de mentira: con timeout 0 espera medio milisegundo. El bucle de la
    placa pregunta sin parar si ha llegado algo; en el PC eso se comeria la CPU y
    no dejaria correr a los motores."""
    m = types.ModuleType('select')
    m.select = lambda r, w, x, t=None: _select.select(r, w, x, max(t or 0.0, 0.0005))
    return m


class _Entrada:
    """sys.stdin de la placa: lee del descriptor byte a byte, sin bufer de Python.
    Con uno, select diria que no hay nada mientras quedan lineas en el bufer."""

    def __init__(self, fd: int):
        self.fd = fd

    def fileno(self) -> int:
        return self.fd

    def readline(self) -> str:
        datos = bytearray()
        while not datos.endswith(b'\n'):
            try:
                c = os.read(self.fd, 1)
            except BlockingIOError:
                _select.select([self.fd], [], [], 0.05)
                continue
            if not c:
                break
            datos += c
        return datos.decode('utf-8', 'replace')


class _Salida:
    """sys.stdout de la placa. Con `tirar`, si nadie lee (el puente aun no ha
    abierto el puerto) lo que no cabe se pierde, como en la UART de verdad; si se
    esperara, el bucle de la placa se quedaria parado en un print."""

    def __init__(self, fd: int, tirar: bool = False):
        self.fd, self.tirar = fd, tirar

    def write(self, texto: str) -> int:
        datos = texto.replace('\n', '\r\n').encode()     # como la REPL de la ESP32
        while datos:
            try:
                n = os.write(self.fd, datos)
            except BlockingIOError:
                if self.tirar:
                    break
                time.sleep(0.001)
                continue
            datos = datos[n:]
        return len(texto)

    def flush(self):
        pass


def _firmware() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        carpeta = os.path.join(get_package_share_directory('scara_kinematics'),
                               'firmware', 'micropython')
    except Exception:             # sin ROS cargado, o el paquete sin instalar
        carpeta = ''
    for ruta in (os.path.join(carpeta, 'main_control.py'),
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                              'firmware', 'micropython', 'main_control.py')):
        if os.path.isfile(ruta):
            return os.path.abspath(ruta)
    raise FileNotFoundError('no encuentro firmware/micropython/main_control.py')


def main(argv=None):
    ap = argparse.ArgumentParser(description='main_control.py con motores simulados')
    ap.add_argument('--firmware', default=None, help='otro main_control.py')
    ap.add_argument('--enlace', default='', help='enlace fijo al puerto falso')
    ap.add_argument('--stdio', action='store_true', help='por stdin/stdout, sin pty')
    ap.add_argument('--fallo', default='', choices=['', 'alvin_inv', 'simon_sin_encoder'])
    ap.add_argument('--pos', nargs=3, type=float, default=[50.0, -100.0, -50.0],
                    metavar=('ALVIN', 'SIMON', 'TEODORO'))
    ap.add_argument('--flash', default='', help='carpeta de su flash (por defecto, una nueva)')
    args, _ = ap.parse_known_args(argv)

    ruta = args.firmware or _firmware()
    with open(ruta) as f:
        codigo = f.read()

    motores = tabla(codigo, 'MOTORES')
    # Su flash: donde estan los motores al arrancar, en pulsos, como los guarda la placa
    flash = args.flash or tempfile.mkdtemp(prefix='scara_placa_simulada_')
    os.makedirs(flash, exist_ok=True)
    for m, pos in zip(motores, args.pos):
        nombre, pulsos_rev, por_vuelta = m[0], m[6], m[7]
        with open(os.path.join(flash, f'posicion_{nombre}.txt'), 'w') as f:
            f.write(str(int(round(pos / por_vuelta * pulsos_rev))))
    os.chdir(flash)

    if args.stdio:
        fd_in, fd_out = sys.stdin.fileno(), sys.stdout.fileno()
    else:
        placa, pc = pty.openpty()
        tty.setraw(placa)
        tty.setraw(pc)
        nombre = os.ttyname(pc)
        if args.enlace:
            try:
                os.unlink(args.enlace)
            except FileNotFoundError:
                pass
            os.symlink(nombre, args.enlace)
        print(f'placa simulada (main_control.py de verdad): puerto {args.enlace or nombre}'
              + (f' -> {nombre}' if args.enlace else '')
              + (f' | fallo: {args.fallo}' if args.fallo else '')
              + f' | motores en {args.pos[0]:g} {args.pos[1]:g} {args.pos[2]:g}', flush=True)
        # Sin bloquear: si nadie lee, lo que no cabe se tira (ver _Salida)
        os.set_blocking(placa, False)
        fd_in = fd_out = placa
        # El lado del PC (pc) no se cierra aqui: si no, cuando el puente aun no lo
        # ha abierto (o lo cierra para volver a abrirlo) la placa leeria fin de archivo.

    sys.modules['machine'] = _maquina(args.fallo, motores)
    sys.modules['select'] = _select_amable()
    sys.stdin = _Entrada(fd_in)
    sys.stdout = _Salida(fd_out, tirar=not args.stdio)
    try:
        exec(compile(codigo, ruta, 'exec'), {'__name__': '__main__'})
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout = sys.__stdout__
        if args.enlace and not args.stdio:
            try:
                os.unlink(args.enlace)
            except OSError:
                pass


if __name__ == '__main__':
    main()
