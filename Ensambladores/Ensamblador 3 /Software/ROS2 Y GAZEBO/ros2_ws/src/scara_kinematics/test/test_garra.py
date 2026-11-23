"""La garra (GARRA.md): garra.py en la placa, el 1 y el 0 por main_control.py, y
el traycho que abre y cierra en cada punto (garra_traycho, sin ROS).

    pytest src/scara_kinematics/test/test_garra.py      (desde ~/ros2_ws)
"""
import importlib.util
import os
import queue
import re
import subprocess
import sys
import threading
import time
import types

import numpy as np
import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import garra_traycho as GT
from scara_kinematics import juntas as J
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE = os.path.join(PAQUETE, 'firmware', 'micropython')
GARRA_PY = os.path.join(FIRMWARE, 'garra.py')


# ============================================================ garra.py, en la placa
def _garra_py(monkeypatch, mem32=None, adc=None):
    """garra.py con un `machine` de mentira que guarda el PWM que crea, y un reloj
    (ticks_ms) que se adelanta a mano: reloj[0] en ms. mem32: los registros (dict).
    adc: lo que lee el ADC del GPIO 32 (una funcion, en uV), el 3V3 que vigila."""
    creados = []
    reloj = [0]

    class Pin:
        IN, PULL_UP, PULL_DOWN = 1, 2, 1

        def __init__(self, n, mode=None, pull=None):
            self.n = n

    class PWM:
        def __init__(self, pin, freq=0):
            self.pin, self.freq, self.d = pin, freq, 0
            creados.append(self)

        def duty(self, d=None):
            if d is None:
                return self.d
            self.d = d

    machine = types.ModuleType('machine')
    machine.Pin, machine.PWM = Pin, PWM
    if mem32 is not None:
        machine.mem32 = mem32
    if adc is not None:
        class ADC:
            ATTN_11DB = 3

            def __init__(self, pin):
                assert pin.n == 32

            def atten(self, a):
                pass

            def read_uv(self):
                return adc()
        machine.ADC = ADC
    monkeypatch.setitem(sys.modules, 'machine', machine)
    monkeypatch.setattr(time, 'ticks_ms', lambda: reloj[0], raising=False)
    monkeypatch.setattr(time, 'ticks_diff', lambda a, b: a - b, raising=False)
    spec = importlib.util.spec_from_file_location('garra_de_la_placa', GARRA_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, creados, reloj


def _rampa(g, reloj, ms, cada=10):
    """Como el lazo de main.py: paso() cada `cada` ms durante `ms`; los duty que manda."""
    duties = []
    for _ in range(ms // cada):
        reloj[0] += cada
        g.paso()
        duties.append(g.servo.duty())
    return duties


def test_garra_py_arranca_abierta_y_obedece_al_1_y_al_0_de_golpe(monkeypatch, capsys):
    g, creados, reloj = _garra_py(monkeypatch)
    assert len(creados) == 1 and creados[0].pin.n == 33 and creados[0].freq == 50
    # Los duty del codigo probado, int(26 + angulo / 180 * 102) a 50 Hz (0.6 y 1.05 ms)
    assert g.estado == 'abierta' and creados[0].d == 31             # 10 grados
    assert g.orden('0') and g.estado == 'cerrada' and creados[0].d == 54       # 50 grados
    assert g.orden('1') and g.estado == 'abierta' and creados[0].d == 31
    assert not g.orden('2') and creados[0].d == 31
    dicho = capsys.readouterr().out.splitlines()
    assert dicho[:3] == ['GARRA abierta 10', 'GARRA cerrada 50', 'GARRA abierta 10']
    assert dicho[3].startswith('[x] garra')


def test_garra_py_apaga_el_detector_de_tension_solo_al_moverse(monkeypatch):
    """19/11: de golpe, el servo (al 3V3) tumbaba la placa por el detector de caidas
    de tension. Se apaga al mandar y vuelve AGUANTAR_MS despues."""
    REG, ON = 0x3FF480D4, 1 << 30
    mem32 = {REG: 0x43ffc000}                       # lo que tenia la placa (umbral 0)
    g, creados, reloj = _garra_py(monkeypatch, mem32)
    assert mem32[REG] == 0x43ffc000                 # al arrancar no se toca
    g.orden('0')
    assert mem32[REG] == 0x43ffc000 & ~ON           # solo el bit del detector
    _rampa(g, reloj, g.AGUANTAR_MS - 10)
    assert not mem32[REG] & ON
    _rampa(g, reloj, 20)
    assert mem32[REG] == 0x43ffc000
    g.orden('0')                                    # ya cerrada, pero suelta: empuja otra vez
    assert not mem32[REG] & ON and g.servo.duty() == 54
    _rampa(g, reloj, g.AGUANTAR_MS + 10)
    assert mem32[REG] == 0x43ffc000
    g.VELOCIDAD = 20.0                              # con rampa: hasta llegar + AGUANTAR_MS
    g.orden('1')
    _rampa(g, reloj, 2000 + g.AGUANTAR_MS - 50)
    assert g.estado == 'abierta' and not mem32[REG] & ON
    _rampa(g, reloj, 100)
    assert mem32[REG] == 0x43ffc000


def test_garra_py_empuja_un_rato_y_suelta(monkeypatch):
    """19/11 21:34: sujetando, el servo bloqueado tiraba corriente sin parar y la placa
    se reinicio con el detector ya puesto. Senal EMPUJON_MS y luego duty 0 (sin pulso)."""
    g, creados, reloj = _garra_py(monkeypatch)
    _rampa(g, reloj, g.EMPUJON_MS + 10)
    assert g.servo.duty() == 0                      # al arrancar tambien se suelta
    g.orden('0')
    duties = _rampa(g, reloj, g.EMPUJON_MS + 100)
    assert duties[:g.EMPUJON_MS // 10 - 1] == [54] * (g.EMPUJON_MS // 10 - 1)
    assert duties[-1] == 0 and g.estado == 'cerrada'
    g.orden('0')                                    # suelta, se ha podido abrir: otra vez
    assert g.servo.duty() == 54
    _rampa(g, reloj, g.EMPUJON_MS + 10)
    assert g.servo.duty() == 0


def test_garra_py_dice_si_al_servo_le_llega_corriente(monkeypatch, capsys):
    """19/11 21:31: la causa era un falso contacto del rojo/marron. Sin corriente, el
    3V3 (su 0.43 en el ADC del GPIO 32, ~1400 mV) no baja; si el servo tira, baja."""
    leido = [1_400_000]
    g, creados, reloj = _garra_py(monkeypatch, adc=lambda: leido[0])
    _rampa(g, reloj, g.EMPUJON_MS + 10)             # al arrancar: no dice nada
    capsys.readouterr()
    g.orden('0')
    leido[0] = 1_390_000                            # ruido: 10 mV
    _rampa(g, reloj, g.EMPUJON_MS + 10)
    dicho = capsys.readouterr().out.splitlines()
    assert dicho[0] == 'GARRA cerrada 50' and dicho[1].startswith('GARRA corriente NO 10 (')
    leido[0] = 1_400_000
    g.orden('1')
    reloj[0] += 100
    leido[0] = 970_000                              # se mueve: el 3V3 baja ~1 V
    g.paso()
    leido[0] = 1_400_000
    _rampa(g, reloj, g.EMPUJON_MS)
    assert capsys.readouterr().out.splitlines() == ['GARRA abierta 10', 'GARRA corriente si 430']


def test_garra_py_cierra_poco_a_poco_sin_saltos(monkeypatch, capsys):
    """19/11: con el servo al 3V3, de golpe la placa se reiniciaba (66 veces); en
    rampa, no. De 10 a 50 grados en 2 s, sin saltar mas de un duty (~1.8 grados)."""
    g, creados, reloj = _garra_py(monkeypatch)
    g.VELOCIDAD, g.FRECUENCIA = 20.0, 50             # la rampa, con los duty de 50 Hz
    g.orden('0')
    duties = _rampa(g, reloj, 2500)
    assert duties[0] in (31, 32) and all(abs(b - a) <= 1 for a, b in zip(duties, duties[1:]))
    llega = duties.index(54)
    assert 1900 <= (llega + 1) * 10 <= 2100                         # 40 grados a 20/s
    dicho = capsys.readouterr().out.splitlines()
    assert dicho[1:] == ['GARRA cerrando 50', 'GARRA cerrada 50']      # al llegar, una vez


def test_garra_py_cambia_de_idea_a_mitad_y_si_ya_esta_lo_dice(monkeypatch, capsys):
    g, creados, reloj = _garra_py(monkeypatch)
    g.VELOCIDAD, g.FRECUENCIA = 20.0, 50
    g.orden('0')
    _rampa(g, reloj, 1000)                                          # va por 30 grados
    assert g.estado == 'cerrando' and abs(g.angulo - 30.0) < 0.5
    g.orden('1')                                                    # vuelve desde ahi
    _rampa(g, reloj, 1100)
    assert g.estado == 'abierta' and creados[0].d == 31
    capsys.readouterr()
    g.orden('1')                                                    # ya abierta
    assert g.estado == 'abierta' and capsys.readouterr().out.splitlines() == ['GARRA abierta 10']


class _Placa:
    """main_control.py en placa_simulada --stdio; con_garra: garra.py a mano."""

    def __init__(self, carpeta, con_garra=True):
        camino = [PAQUETE] + ([FIRMWARE] if con_garra else []) + [os.environ.get('PYTHONPATH', '')]
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(camino))
        cmd = [sys.executable, '-u', '-m', 'scara_kinematics.placa_simulada', '--stdio',
               '--firmware', os.path.join(FIRMWARE, 'main_control.py'), '--flash', carpeta]
        self.p = subprocess.Popen(cmd, env=env, cwd=carpeta, stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.lineas: queue.Queue = queue.Queue()
        threading.Thread(target=self._leer, daemon=True).start()
        self.arranque = self.esperar(r'^=+$', 15.0)

    def _leer(self):
        for linea in self.p.stdout:
            linea = linea.strip()
            if not linea.startswith(('P ', 'D ')):
                self.lineas.put(linea)

    def mandar(self, linea: str):
        self.p.stdin.write(linea + '\r\n')
        self.p.stdin.flush()

    def esperar(self, patron: str, plazo: float = 5.0) -> list:
        vistas, fin = [], time.monotonic() + plazo
        while time.monotonic() < fin:
            try:
                linea = self.lineas.get(timeout=0.05)
            except queue.Empty:
                continue
            vistas.append(linea)
            if re.search(patron, linea):
                return vistas
        raise AssertionError(f'no llego «{patron}». Llego:\n' + '\n'.join(vistas[-20:]))

    def cerrar(self):
        self.p.kill()
        self.p.wait()


def test_main_control_con_garra_abre_y_cierra_y_los_motores_van_igual(tmp_path):
    placa = _Placa(str(tmp_path))
    try:
        assert 'GARRA abierta 10' in placa.arranque
        assert any('1/0 = abrir/cerrar la garra' in x for x in placa.arranque)
        placa.mandar('0')
        placa.esperar(r'^GARRA cerrada 50$')
        placa.mandar('1')
        placa.esperar(r'^GARRA abierta 10$')
        placa.mandar('a')
        placa.esperar(r'^ACTIVO AAA')
        placa.mandar('0')                      # con los motores activos, tambien
        placa.esperar(r'^GARRA cerrada 50$')
        placa.mandar('q')
        placa.esperar(r'^PARADO')
    finally:
        placa.cerrar()


def test_main_control_sin_garra_py_lo_dice_y_lo_demas_igual(tmp_path):
    placa = _Placa(str(tmp_path), con_garra=False)
    try:
        assert not any('GARRA' in x for x in placa.arranque)
        assert any('(sin garra.py)' in x for x in placa.arranque)
        placa.mandar('1')
        placa.esperar(r'^\[x\] no hay garra\.py en la placa')
        placa.mandar('l')
        placa.esperar(r'^LIMITES Alvin')
    finally:
        placa.cerrar()


# ============================================================ el traycho con garra
def _yaml(nombre):
    with open(os.path.join(PAQUETE, 'config', nombre)) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def _robot():
    """Con los topes de traycho_juntas.launch.py (como en test_juntas)."""
    mp = _yaml('micropython.yaml')
    cal = A.Calibracion.desde_parametros(lambda n, d: mp.get(n, d))
    rp = _yaml('scara_urdf3.yaml')
    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    lim = list(PV.limites_junta(cal, robot, 0.5))
    lim[1] = (max(lim[1][0], cal.a_junta([0.0, J.SIMON_MAX, 0.0])[1]), lim[1][1])
    lim[2] = (max(lim[2][0], cal.a_junta([0.0, 0.0, J.TEODORO_MIN_MM])[2]), lim[2][1])
    return PV.robot_con_limites(robot, lim)


def test_la_garra_solo_al_llegar_al_punto():
    assert GT.accion_de('P1') == GT.ABRIR
    assert GT.accion_de('P1 (2/2)') == GT.ABRIR
    assert GT.accion_de('P1 (1/2)') is None
    assert GT.accion_de('P2') == GT.CERRAR
    for nombre in ('P2 encima', 'P2 sube', 'P2 encima (1/3)', 'sube', 'P7', 'P8'):
        assert GT.accion_de(nombre) is None, nombre
    assert [GT.ACCIONES[k] for k in range(1, 7)] == ['1', '0', '1', '0', '1', '0']


class _Brazo:
    """Lo de la ventana: lo mandado (ir y garra, en orden) y lo dicho."""

    def __init__(self, escucha_garra=True):
        self.eventos, self.dicho, self.escucha_garra = [], [], escucha_garra

    def mandar(self, texto):
        self.eventos.append(('ir', texto))
        return True

    def mandar_garra(self, texto):
        if self.escucha_garra:
            self.eventos.append(('garra', texto))
        return self.escucha_garra

    def decir(self, nivel, texto):
        self.dicho.append((nivel, texto))


def _correr(r, b, contesta=True, dt=0.1, hasta=2000.0):
    """juntas_nodo y la placa de mentira: cada ir acaba enseguida y la garra
    contesta como garra.py."""
    t, visto = 0.0, 0
    while r.activo() and t < hasta:
        r.paso(t)
        for tipo, texto in b.eventos[visto:]:
            if tipo == 'ir':
                r.aviso('control', 20, 'ir: 1.0 s (por juntas)')
                r.aviso('control', 20, 'ir terminada | en el punto: medio 0.100 mm, '
                                       'maximo 0.200 mm | 0 correcciones')
            elif contesta:
                r.aviso('puente_placa', 20, 'placa: GARRA '
                        + ('abierta 10' if texto == GT.ABRIR else 'cerrada 50'))
        visto = len(b.eventos)
        t += dt
    return t


def test_el_traycho_abre_en_home_y_luego_abre_y_cierra_en_p1_a_p6():
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, _robot(), desde=pts[0], z_arriba=float(pts[0][2]))
    b = _Brazo()
    r = GT.RecorridoGarra(tramos, b.mandar, b.decir, b.mandar_garra)
    _correr(r, b)
    assert r.fase == 'hecho', b.dicho[-5:]

    # Lo que tenia que pasar: abre al salir, y cada ir, con su garra detras si la lleva
    esperado = [('garra', '1')]
    for nombre, p in tramos:
        esperado.append(('ir', 'ir ' + ' '.join(f'{c:.1f}' for c in p)))
        if GT.accion_de(nombre):
            esperado.append(('garra', GT.accion_de(nombre)))
    assert b.eventos == esperado
    assert [t for k, t in b.eventos if k == 'garra'] == ['1', '1', '0', '1', '0', '1', '0']

    # Donde: en P1 (el HOME), y en P2..P6 abajo, antes de volver a subir
    nombres = [n for n, _ in tramos]
    con_garra = [n for n in nombres if GT.accion_de(n)]
    assert [re.match(r'P\d', n).group() for n in con_garra] == ['P1', 'P2', 'P3', 'P4',
                                                                  'P5', 'P6']
    for k in range(2, 7):
        i = nombres.index(f'P{k}')
        assert nombres[i + 1] == f'P{k} sube'
        assert tramos[i][1][2] < pts[0][2] - 10.0          # abajo de verdad
    assert sum('traycho terminado' in t for _, t in b.dicho) == 1


def test_espera_a_que_la_placa_conteste_y_al_servo_antes_de_seguir():
    p = np.array([0.0, -362.3, 220.0])
    b = _Brazo()
    r = GT.RecorridoGarra([('P1', p), ('P2', p)], b.mandar, b.decir, b.mandar_garra)
    r.paso(0.0)
    assert b.eventos == [('garra', '1')]
    r.paso(1.0)
    assert len(b.eventos) == 1                       # sin contestar, no sigue
    r.aviso('puente_placa', 20, 'placa: GARRA abierta 10')
    r.paso(1.1)
    r.paso(1.1 + GT.ESPERA_S - 0.05)
    assert len(b.eventos) == 1                       # el servo aun va
    r.paso(1.1 + GT.ESPERA_S + 0.01)
    assert b.eventos[-1] == ('ir', 'ir 0.0 -362.3 220.0')


@pytest.mark.parametrize('nodo, nivel, texto', [
    ('puente_placa', 20, 'placa: [x] no entiendo: 1'),                 # main.py sin la garra
    ('puente_placa', 20, 'placa: [x] no hay garra.py en la placa (GARRA.md)'),
    ('puente_placa', 40, 'no conozco «1»: activar, soltar, ?, l, g, h, fijar'),  # puente_placa
    ('puente_placa', 30, 'placa: PARADO 1 2 3  (motores sueltos; para seguir, «activar»)'),
])
def test_se_corta_si_la_placa_no_tiene_la_garra_o_suelta(nodo, nivel, texto):
    p = np.array([0.0, -362.3, 220.0])
    b = _Brazo()
    r = GT.RecorridoGarra([('P1', p)], b.mandar, b.decir, b.mandar_garra)
    r.paso(0.0)
    r.aviso(nodo, nivel, texto)
    r.paso(0.1)
    assert r.fase == 'cortado' and not any(k == 'ir' for k, _ in b.eventos)
    assert 'traycho cortado' in b.dicho[-1][1]


def test_se_corta_si_la_garra_no_contesta_o_nadie_escucha():
    p = np.array([0.0, -362.3, 220.0])
    b = _Brazo()
    r = GT.RecorridoGarra([('P1', p)], b.mandar, b.decir, b.mandar_garra)
    r.paso(0.0)
    r.paso(GT.CONTESTA_S - 0.1)
    assert r.activo()
    r.paso(GT.CONTESTA_S + 0.1)
    assert r.fase == 'cortado' and 'no contesta' in b.dicho[-1][1]

    b = _Brazo(escucha_garra=False)
    r = GT.RecorridoGarra([('P1', p)], b.mandar, b.decir, b.mandar_garra)
    r.paso(0.0)
    assert r.fase == 'cortado' and 'no escucha' in b.dicho[-1][1]


def test_si_el_ultimo_punto_lleva_garra_termina_despues_de_moverla():
    p = np.array([0.0, -362.3, 220.0])
    b = _Brazo()
    r = GT.RecorridoGarra([('P1', p), ('P2', p)], b.mandar, b.decir, b.mandar_garra,
                          acciones={2: GT.CERRAR})
    _correr(r, b)
    assert r.fase == 'hecho'
    assert [k for k, _ in b.eventos] == ['garra', 'ir', 'ir', 'garra']
    assert b.eventos[-1] == ('garra', '0')
    assert sum('traycho terminado' in t for _, t in b.dicho) == 1
    assert 'terminado' in b.dicho[-1][1]


def test_puente_garra_pasa_el_1_y_el_0_y_lo_demas_a_puente_placa(monkeypatch):
    from std_msgs.msg import String
    from scara_kinematics import puente_garra as PG

    al_padre = []
    monkeypatch.setattr(PG.PP.PuentePlaca, 'on_orden', lambda self, msg: al_padre.append(msg.data))
    puente = object.__new__(PG.PuenteGarra)          # sin ROS: solo on_orden
    mandado = []
    puente._mandar = mandado.append
    for texto in ('1', '0', 'abrir', 'Cerrar', 'activar', '?', 'fijar simon 0'):
        puente.on_orden(String(data=texto))
    assert mandado == ['1', '0', '1', '0']
    assert al_padre == ['activar', '?', 'fijar simon 0']


# ============================================================ el suelo de la mano
def _launch_garra():
    ruta = os.path.join(PAQUETE, 'launch', 'traycho_garra.launch.py')
    spec = importlib.util.spec_from_file_location('traycho_garra_launch', ruta)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_el_launch_de_la_garra_pone_el_suelo_de_la_mano():
    from launch.actions import DeclareLaunchArgument
    args = [a for a in _launch_garra().generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument) and a.name == 'teodoro_min']
    assert len(args) == 1 and float(args[0].default_value[0].text) == GT.TEODORO_SUELO_MM
    assert -27.8 < GT.TEODORO_SUELO_MM < 0


def test_con_la_garra_la_mano_no_baja_del_suelo_y_sale_subiendo():
    """19/11: la mano llega al suelo con Teodoro a -27.8 («esta lo mas abajo posible»).
    Desde ahi el traycho sube primero en vertical, y ninguna ida baja de -25."""
    mp = _yaml('micropython.yaml')
    cal = A.Calibracion.desde_parametros(lambda n, d: mp.get(n, d))
    r = _robot()
    lim = list(r.lim)
    lim[2] = (cal.a_junta([0.0, 0.0, GT.TEODORO_SUELO_MM])[2], lim[2][1])
    r = PV.robot_con_limites(r, lim)
    z = GT.z_suelo(r)
    assert z == pytest.approx(220.51 + GT.TEODORO_SUELO_MM, abs=0.01)    # z = 220.51 en su 0
    pts = GT.con_suelo(J.puntos_mm(), z)
    assert np.allclose(pts[:, :2], J.puntos_mm()[:, :2]) and pts[0][2] == J.puntos_mm()[0][2]
    assert np.all(pts[1:7, 2] == pytest.approx(z, abs=1e-3))           # P2..P7, en el suelo
    desde = R.fk_scara_rrp(J.a_rad(cal.a_junta([0.0, 0.0, -27.88])), J.sin_topes(r))
    tramos = J.tramos_juntas(pts, r, desde=desde, z_arriba=float(pts[0][2]))
    assert tramos[0][0] == 'sube'
    teodoro = [cal.a_motor(R.ik_scara_rrp(p, r, 'down'))[2] for _, p in tramos]
    assert min(teodoro) >= GT.TEODORO_SUELO_MM - 0.01
