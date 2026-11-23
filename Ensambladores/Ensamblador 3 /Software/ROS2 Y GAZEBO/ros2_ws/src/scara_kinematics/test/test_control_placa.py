"""El control cinematico en el brazo de verdad, sin el brazo.

Dos partes:
  - placa_vel: el protocolo con la placa y las cuentas de junta a motor;
  - firmware/micropython/main_control.py DE VERDAD, ejecutado en el PC por
    placa_simulada (motores con inercia y friccion, encoder de 5500 pulsos por
    vuelta): que siga la velocidad, respete los topes y salten los vigilantes.

    pytest src/scara_kinematics/test/test_control_placa.py      (desde ~/ros2_ws)
"""
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
import types

import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import placa_simulada as PS
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

AQUI = os.path.dirname(os.path.abspath(__file__))
PAQUETE = os.path.dirname(AQUI)
FIRMWARE = os.path.join(PAQUETE, 'firmware', 'micropython')
YAML_PLACA = os.path.join(PAQUETE, 'config', 'micropython.yaml')


def _yaml(ruta: str) -> dict:
    with open(ruta) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def _cal_placa() -> A.Calibracion:
    p = _yaml(YAML_PLACA)
    return A.Calibracion.desde_parametros(lambda nombre, defecto: p.get(nombre, defecto))


def _robot() -> R.Robot:
    rp = _yaml(os.path.join(PAQUETE, 'config', 'scara_urdf3.yaml'))
    return R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                   rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                 (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))


# =========================================================
# placa_vel
# =========================================================

def test_la_v_lleva_su_checksum_y_las_lineas_de_la_placa_se_leen():
    v = PV.linea_v([-1.5, 0.25, 0.0])
    cuerpo, cs = v.rsplit(' *', 1)
    assert cuerpo == 'V -1.500 0.250 0.000' and cs == PV.checksum(cuerpo)
    p = PV.leer(PV.con_checksum('P 1234 -10.000 -22.500 0.000 AAL'))
    assert p == PV.Muestra(1234, [-10.0, -22.5, 0.0], [True, True, False])
    d = PV.leer(PV.con_checksum('D 1234 0.050 -0.020 0.000 123 0 0'))
    assert d.seguimiento == [0.05, -0.02, 0.0] and d.pwm == [123.0, 0.0, 0.0]
    assert PV.leer('PARADO -10.00 -22.50 0.00') is None
    assert PV.leer('LISTO 1 2 3') is None
    with pytest.raises(PV.ChecksumMalo):
        PV.leer('P 1234 -10.000 -22.500 0.000 AAL *00')
    with pytest.raises(PV.ChecksumMalo):
        PV.leer('P 1234 -10.000 -22.500 0.000 AAL')
    assert PV.leer_limites('LIMITES Alvin 0 100 Simon -200 0 Teodoro -100 0') == \
        [0.0, 100.0, -200.0, 0.0, -100.0, 0.0]


# Una calibracion de ejemplo, distinta de la de la placa: los numeros de antes
# del 21/10 (grados de motor, sin paso del husillo)
VIEJA = dict(limites=[(-20.0, 0.0), (-45.0, 0.0), (-30.0, 40.0)], signo=[1.0, 1.0, 1.0],
             cero=[0.0, 0.0, 0.0], paso_husillo_mm=0.0)


def test_velocidad_de_junta_a_motor():
    cal = A.Calibracion(reduccion=[2.0, 1.0], signo=[-1.0, 1.0, 1.0], paso_husillo_mm=8.0)
    a, s, t = PV.vel_a_motor([math.radians(3.0), math.radians(-1.0), 4.0], cal)
    assert a == pytest.approx(-6.0) and s == pytest.approx(-1.0)
    assert t == pytest.approx(180.0)                       # 4 mm/s con 8 mm por vuelta
    assert PV.vel_a_motor([0.1, 0.1, 4.0], A.Calibracion(**VIEJA))[2] == 0.0    # sin paso
    # el cero no cuenta en una velocidad
    con_cero = A.Calibracion(cero=[5.0, -7.0, 0.0])
    assert PV.vel_a_motor([0.1, 0.2, 0.0], con_cero) == PV.vel_a_motor([0.1, 0.2, 0.0],
                                                                       A.Calibracion())


def test_los_topes_de_junta_salen_de_los_recorridos_de_los_motores():
    lim = PV.limites_junta(A.Calibracion(**VIEJA), R.ROBOT, margen=0.5)
    assert [math.degrees(v) for v in lim[0]] == pytest.approx([-19.5, -0.5])
    assert [math.degrees(v) for v in lim[1]] == pytest.approx([-44.5, -0.5])
    assert lim[2] == R.ROBOT.lim[2]            # sin paso del husillo: los del CAD
    al_reves = PV.limites_junta(A.Calibracion(**{**VIEJA, 'signo': [-1.0, 1.0, 1.0]}),
                                R.ROBOT, 0.5)
    assert [math.degrees(v) for v in al_reves[0]] == pytest.approx([0.5, 19.5])
    husillo = PV.limites_junta(A.Calibracion(**{**VIEJA, 'paso_husillo_mm': 8.0,
                                                'cero': [0.0, 0.0, 95.0]}), R.ROBOT, 0.0)
    assert husillo[2] == pytest.approx((95.0 - 30 * 8 / 360, 95.0 + 40 * 8 / 360))
    # Con el cero en d3 = 0, los -30..40 grados de Teodoro son +-1 mm: fuera del CAD
    with pytest.raises(ValueError, match='Teodoro'):
        PV.limites_junta(A.Calibracion(**{**VIEJA, 'paso_husillo_mm': 8.0}), R.ROBOT, 0.0)


def test_con_la_calibracion_de_la_placa_cabe_todo():
    """config/micropython.yaml: Teodoro en mm (paso 360) y los recorridos de los
    programas de Sergio, pasados a junta sin salirse del CAD."""
    cal, robot = _cal_placa(), _robot()
    lim = PV.limites_junta(cal, robot, margen=0.5)
    # Alvin -59.5..59.5 desde el home (cero -45.68), con signo -1
    assert [math.degrees(v) for v in lim[0]] == pytest.approx([-105.18, 13.82])
    # Simon -279.5..-0.5 -> codo -89.51..189.49, recortado al CAD
    assert [math.degrees(v) for v in lim[1]] == pytest.approx([-89.51, 150.0], abs=1e-3)
    assert lim[2] == pytest.approx((20.5, 154.5))          # d3 = 155 + Teodoro, en mm
    assert PV.vel_a_motor([0.0, 0.0, 8.0], cal)[2] == pytest.approx(8.0)      # mm/s, sube
    assert cal.a_motor([0.0, 0.0, 70.0])[2] == pytest.approx(-85.0)


def test_la_zona_de_trabajo_cabe_en_los_topes():
    rob = PV.robot_con_limites(R.ROBOT, PV.limites_junta(A.Calibracion(), R.ROBOT))
    z = PV.zona_trabajo(rob)
    assert z['holgura'] > 20.0
    q = R.ik_scara_rrp(z['centro'], rob, 'down')        # levanta RutaError si no cabe
    assert rob.fuera(q) is None
    x0, x1, y0, y1 = z['caja']
    assert x0 < z['centro'][0] < x1 and y0 < z['centro'][1] < y1


def test_el_reloj_de_la_placa_se_pasa_al_del_pc():
    reloj = PV.Reloj()
    desfase = 1000.0
    for i in range(300):
        ms = 20 * i
        retraso = 0.002 + (0.015 if i % 3 else 0.0)          # el USB tarda lo que quiere
        t = reloj(ms, ms * 1e-3 + desfase + retraso)
    assert t == pytest.approx(ms * 1e-3 + desfase + 0.002, abs=1e-9)
    # ticks_ms da la vuelta a los 2^30 ms
    r2 = PV.Reloj()
    r2(PV.TICKS_PERIODO - 10, 5.0)
    assert r2(10, 5.020) == pytest.approx(5.020)
    # la placa se reinicia: su reloj vuelve a empezar
    assert r2(5, 9.0) == pytest.approx(9.0)


# =========================================================
# main_control.py, en la placa simulada
# =========================================================

def _tabla(nombre: str):
    """La variable `nombre` de main_control.py, sin ejecutarlo."""
    with open(os.path.join(FIRMWARE, 'main_control.py')) as f:
        return PS.tabla(f.read(), nombre)


def test_la_tabla_de_motores_es_la_de_los_programas_de_sergio():
    """Pines y cuentas de Alvin.py, Simon.py y Teodoro.py (los que mueven el brazo,
    21/10): IN1, IN2, ENA, ENC_A, ENC_B, pulsos por vuelta del motor, y lo que
    avanza la junta por vuelta."""
    tabla = {m[0]: m[1:] for m in _tabla('MOTORES')}
    assert tabla['Alvin'][:7] == (4, 16, 17, 26, 25, 5500, 360.0)
    assert tabla['Simon'][:6] == (18, 19, 5, 12, 13, 5500)
    assert 5500 * 360.0 / tabla['Simon'][6] == pytest.approx((22000 / 4) * (21 / 18))
    assert tabla['Teodoro'][:6] == (21, 22, 23, 27, 14, 5500)
    assert 5500 / tabla['Teodoro'][6] == pytest.approx(
        (22000 / 4) / ((3.14159265 * (0.5 * 20)) + 10))           # PULSOS_POR_MM
    assert _tabla('SIN_H') == ('Alvin',)                        # NO DARLE A LA H


def test_las_velocidades_maximas_del_puente_caben_en_las_de_la_placa():
    """La placa deja lo que manda el puente: el de control_nodo (su defecto) y el
    de por juntas (traycho_juntas.launch.py), que es justo su VEL_MAX."""
    with open(os.path.join(PAQUETE, 'scara_kinematics', 'puente_placa.py')) as f:
        defecto = re.search(r"p\('vel_max_motor', (\[[^]]*\])\)", f.read()).group(1)
    with open(os.path.join(PAQUETE, 'launch', 'traycho_juntas.launch.py')) as f:
        juntas = re.search(r"^VEL_MAX_MOTOR = (\[[^]]*\])", f.read(), re.M).group(1)
    placa = [m[-1] for m in _tabla('MOTORES')]
    assert all(float(v) <= m for v, m in zip(defecto.strip('[]').split(','), placa))
    assert [float(v) for v in juntas.strip('[]').split(',')] == placa


class Placa:
    """main_control.py en placa_simulada --stdio: se le escribe y se lee lo que dice."""

    def __init__(self, carpeta, pos=(50.0, -100.0, -50.0), fallo=''):
        env = dict(os.environ, PYTHONPATH=PAQUETE + os.pathsep + os.environ.get('PYTHONPATH', ''))
        cmd = [sys.executable, '-u', '-m', 'scara_kinematics.placa_simulada', '--stdio',
               '--firmware', os.path.join(FIRMWARE, 'main_control.py'), '--flash', carpeta,
               '--pos'] + [str(v) for v in pos] + (['--fallo', fallo] if fallo else [])
        self.carpeta = carpeta
        self.p = subprocess.Popen(cmd, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True)
        self.lineas: queue.Queue = queue.Queue()
        self.ultima: PV.Muestra | None = None
        threading.Thread(target=self._leer, daemon=True).start()
        self.esperar(r'^=+$', 15.0)

    def _leer(self):
        for linea in self.p.stdout:
            linea = linea.strip()
            if linea.startswith('P '):
                self.ultima = PV.leer(linea)
            elif not linea.startswith('D '):
                self.lineas.put(linea)

    def mandar(self, linea: str):
        self.p.stdin.write(linea + '\r\n')
        self.p.stdin.flush()

    def velocidad(self, vel, segundos: float):
        """La V a 50 por segundo durante `segundos`, como el puente."""
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            self.mandar(PV.linea_v(vel))
            time.sleep(0.02)

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

    def muestra(self) -> PV.Muestra:
        """Una P que haya llegado despues de llamar a esto."""
        vieja, fin = self.ultima, time.monotonic() + 1.0
        while (self.ultima is None or self.ultima is vieja) and time.monotonic() < fin:
            time.sleep(0.01)
        return self.ultima

    def pos(self) -> list:
        time.sleep(0.1)
        return list(self.muestra().pos)

    def cerrar(self):
        self.p.kill()
        self.p.wait()


@pytest.fixture
def placa(tmp_path):
    p = Placa(str(tmp_path))
    yield p
    p.cerrar()


def test_los_limites_de_la_placa_son_los_del_yaml(placa):
    yaml_lim = _yaml(YAML_PLACA)['limites_motor']
    placa.mandar('l')
    linea = placa.esperar(r'^LIMITES')[-1]
    assert PV.leer_limites(linea) == [float(v) for v in yaml_lim]


def test_suelto_no_se_mueve_y_una_v_estropeada_se_tira(placa):
    antes = placa.pos()
    placa.velocidad([-5.0, 5.0, 0.0], 0.5)               # sin 'a': motores sueltos
    assert placa.pos() == pytest.approx(antes, abs=0.01)
    assert placa.muestra().activo == [False, False, False]
    placa.mandar('a alvin simon')
    placa.esperar(r'^ACTIVO AAL')
    placa.mandar('V -5.000 5.000 0.000 *00')
    placa.esperar(r'^ERROR V checksum')
    assert placa.pos() == pytest.approx(antes, abs=0.15)


def test_sigue_la_velocidad_y_no_pasa_del_tope(tmp_path):
    placa = Placa(str(tmp_path), pos=(-50.0, -100.0, -3.0))
    try:
        placa.mandar('a')
        placa.esperar(r'^ACTIVO AAA')
        placa.velocidad([-4.0, 3.0, 1.0], 1.0)            # grados/s, grados/s, mm/s
        a, s, t = placa.pos()
        assert a == pytest.approx(-54.0, abs=0.75)     # con ARRANQUE 700 va a saltos
        assert s == pytest.approx(-97.0, abs=0.5)        # grados de junta (con su 21/18)
        assert t == pytest.approx(-2.0, abs=0.15)        # mm
        # contra los topes (Alvin -60, Teodoro 0), a lo que manda el puente con control_nodo
        placa.velocidad([-15.0, 0.0, 10.0], 1.5)
        placa.velocidad([0.0, 0.0, 0.0], 0.3)
        a, s, t = placa.pos()
        assert a == pytest.approx(-60.0, abs=0.75)  # con ARRANQUE 700 se pasa hasta 0.55
        assert t == pytest.approx(0.0, abs=0.2)     # con ARRANQUE 450 se pasa hasta 0.14 (banda 0.12)
        placa.mandar('h simon')                          # la h, solo suelto
        placa.esperar(r'h y fijar, solo con los motores sueltos')
    finally:
        placa.cerrar()


def test_la_h_y_fijar(placa):
    placa.mandar('h')
    placa.esperar(r'^\[x\] uso: h <motor>')
    placa.mandar('h alvin')
    placa.esperar(r'a Alvin no se le da la h')
    placa.mandar('fijar alvin 37.5')
    placa.esperar(r'^\[f\] Alvin: ahora esta en 37\.5\d grados')
    placa.mandar('h teodoro')
    placa.esperar(r'^\[f\] Teodoro: ahora esta en 0\.00 mm')
    assert placa.pos()[0] == pytest.approx(37.5, abs=0.07)       # un pulso: 0.065 grados
    assert placa.pos()[2] == 0.0
    with open(os.path.join(placa.carpeta, 'posicion_Alvin.txt')) as f:
        assert int(f.read()) == round(37.5 / 360 * 5500)            # en pulsos, como siempre


def test_si_el_pc_se_calla_sujeta_y_luego_suelta(placa):
    placa.mandar('a')
    placa.esperar(r'^ACTIVO AAA')
    placa.velocidad([2.0, 0.0, 0.0], 0.3)
    placa.esperar(r'^VIGILANTE sin ordenes del PC en 300 ms', 1.0)
    quieto = placa.pos()
    vistas = placa.esperar(r'^PARADO', 4.0)
    assert any('suelto los motores' in v for v in vistas)
    assert placa.pos() == pytest.approx(quieto, abs=0.2)
    assert placa.muestra().activo == [False, False, False]


def test_fuera_de_su_recorrido_no_se_activa(tmp_path):
    placa = Placa(str(tmp_path), pos=(-70.0, -100.0, -50.0))       # Alvin, 10 grados fuera
    try:
        placa.mandar('a')
        assert re.search(r'Alvin esta en -(69\.9|70\.0)\d y su recorrido es \[-60, 60\]: si no es '
                         r'verdad, fijar alvin', placa.esperar(r'^RECHAZADO')[-1])
        assert placa.muestra().activo == [False, False, False]
    finally:
        placa.cerrar()


@pytest.mark.parametrize('fallo, motivo, max_mov', [
    # con ARRANQUE 700 (lo que necesita Alvin para moverse, 21/10) va mas lejos
    # que con 123 antes de que se vea que va al reves: 1.6 grados en la simulada
    ('alvin_inv', r'^ERROR Alvin va al reves', 2.0),
    ('simon_sin_encoder', r'^ERROR Simon .* no se ha movido', 0.2),
])
def test_un_motor_mal_suelta_a_los_tres(tmp_path, fallo, motivo, max_mov):
    placa = Placa(str(tmp_path), fallo=fallo)
    try:
        antes = placa.pos()
        placa.mandar('a alvin simon')
        placa.esperar(r'^ACTIVO')
        hilo = threading.Thread(target=placa.velocidad, args=([-3.0, -3.0, 0.0], 1.5))
        hilo.start()
        placa.esperar(motivo, 3.0)
        placa.esperar(r'^PARADO', 1.0)
        hilo.join()
        despues = placa.pos()
        assert placa.muestra().activo == [False, False, False]
        # lo que se ha llegado a mover el que falla, antes de que lo pare la placa
        i = 0 if fallo.startswith('alvin') else 1
        assert abs(despues[i] - antes[i]) < max_mov
    finally:
        placa.cerrar()


def test_teodoro_sube_despues_de_bajar(tmp_path):
    """Parado tras bajar, Teodoro se queda con la consigna hasta 1.5 grados de
    motor por encima (ADELANTO_MAX). Con BANDA 1.0, asentado no volvia a empujar
    hasta 2.0 (SALIDA_BANDA): la consigna esperaba al motor y el motor a la
    consigna, y no subia (traycho en la placa simulada, 21/10)."""
    placa = Placa(str(tmp_path), pos=(0.0, -100.0, -40.0))
    try:
        placa.mandar('a')
        placa.esperar(r'^ACTIVO AAA')
        placa.velocidad([0.0, 0.0, -8.0], 2.5)
        # regulando: control_nodo pide un poco hacia arriba (0.26 grados/s de
        # motor, menos que VEL_QUIETA) y la consigna sube hasta ADELANTO_MAX
        placa.velocidad([0.0, 0.0, 0.03], 7.0)
        abajo = placa.pos()[2]
        placa.velocidad([0.0, 0.0, 8.0], 2.5)
        assert placa.pos()[2] - abajo > 15.0
    finally:
        placa.cerrar()


def _motor_del_firmware(nombre, carpeta, monkeypatch):
    """Un Motor de main_control.py, solo: su codigo hasta «motores = [...]», con un
    `machine` que no hace nada. Su encoder se mueve a mano (m.pulsos)."""
    with open(os.path.join(FIRMWARE, 'main_control.py')) as f:
        codigo = f.read()
    codigo = codigo[:codigo.index('motores = [Motor(*m) for m in MOTORES]')]

    class Pin:
        OUT = IN = IRQ_RISING = IRQ_FALLING = 0

        def __init__(self, *args, **kwargs):
            pass

        def value(self, v=None):
            return 0

        def irq(self, **kwargs):
            pass

    class PWM:
        def __init__(self, *args, **kwargs):
            pass

        def duty(self, d=None):
            return 0

    maquina = types.ModuleType('machine')
    maquina.Pin, maquina.PWM, maquina.Timer = Pin, PWM, object
    monkeypatch.setitem(sys.modules, 'machine', maquina)
    monkeypatch.chdir(carpeta)
    g = {'__name__': 'main_control'}
    exec(compile(codigo, 'main_control.py', 'exec'), g)
    return g['Motor'](*next(m for m in g['MOTORES'] if m[0] == nombre))


@pytest.mark.parametrize('sentido', [-1.0, 1.0], ids=['bajando', 'subiendo'])
def test_teodoro_no_se_embala_cuando_la_cremallera_se_suelta(tmp_path, monkeypatch, sentido):
    """Con la cremallera rozando, Teodoro va a tirones y su integral se cargaba en
    cada tiron. Cuando rozaba menos, ese integral lo embalaba y lo seguia
    empujando aunque ya se hubiera pasado de la consigna (21/10, traycho en el
    brazo: bajando a 2 mm/s, «se ha ido 0.62 mm», y subiendo a 1.8 mm/s,
    «-0.45 mm»; la placa solto los tres). Teodoro va sin integral: el roce lo
    vence el suelo de ARRANQUE."""
    t = _motor_del_firmware('Teodoro', tmp_path, monkeypatch)
    pulsos_mm = t.PULSOS_REV / (360.0 / t.f)
    t.pulsos = int(-60.0 * pulsos_mm)
    t.ref = t.pos_ant = t.posicion()
    t.activo = True
    estado = {'v': 0.0, 'acum': 0.0, 'deseada': t.medida()}
    errores_mm = []

    def mover(segundos, roce):
        """control_nodo: 0.8 mm/s mas K = 2 por lo que se aleja de su recta. El
        motor, parado con PWM 0, se sostiene solo. Con roce, empujando con
        ARRANQUE va a 1 mm/s y sin empujar se pega en 10 ms. Sin roce, con 200
        ya va a 2 mm/s y sin empujar sigue, frenando en 0.3 s."""
        for _ in range(int(segundos * 100)):
            estado['deseada'] += sentido * 0.8 * 0.01
            t.vel = (sentido * 0.8 + 2.0 * (estado['deseada'] - t.medida())) * t.f
            t.paso(10)
            minimo, rapido, frena = (t.arranque, 1.0, 0.63) if roce else (200.0, 2.0, 0.033)
            if abs(t.pwm) >= minimo:
                estado['v'] += (math.copysign(rapido, t.pwm) - estado['v']) * 0.2
            else:
                estado['v'] -= estado['v'] * frena
            estado['acum'] += estado['v'] * 0.01 * pulsos_mm
            t.pulsos += int(estado['acum'])
            estado['acum'] -= int(estado['acum'])
            errores_mm.append(abs(t.e / t.f))

    mover(20.0, roce=True)
    # La cremallera se suelta de golpe: un tiron de 0.3 mm, y luego roza menos
    t.pulsos += int(sentido * 0.3 * pulsos_mm)
    estado['v'] = sentido * 2.0
    mover(2.0, roce=False)
    assert t.fallo is None, t.fallo
    assert max(errores_mm) < 0.35


def test_alvin_cambia_de_sentido_sin_empujar_al_reves(tmp_path, monkeypatch):
    """Yendo hacia un lado mas despacio de lo que se le pide, el integral se carga
    hacia ese lado. Al cambiar de sentido seguia empujando hacia el de antes, y se
    gastaba a KI * ADELANTO_MAX: 21/10, traycho en el brazo, Alvin se quedo 10 s
    quieto con PWM de -311 a -23 y la punta, 53 mm atras. Tiene que arrancar
    hacia el otro lado enseguida."""
    a = _motor_del_firmware('Alvin', tmp_path, monkeypatch)
    pulsos_grado = a.PULSOS_REV / 360.0
    a.activo = True
    acum = [0.0]

    def mover(segundos, sentido):
        """Se le piden 2 grados/s. Con ARRANQUE va a 1.5 hacia donde empuja; con
        menos, no se mueve."""
        a.vel = sentido * 2.0 * a.f
        for _ in range(int(segundos * 100)):
            a.paso(10)
            if abs(a.pwm) >= a.arranque:
                acum[0] += math.copysign(1.5, a.pwm) * 0.01 * pulsos_grado
            a.pulsos += int(acum[0])
            acum[0] -= int(acum[0])

    mover(15.0, 1.0)
    alla = a.medida()
    mover(2.0, -1.0)
    assert a.medida() < alla - 0.5
    assert a.fallo is None, a.fallo
