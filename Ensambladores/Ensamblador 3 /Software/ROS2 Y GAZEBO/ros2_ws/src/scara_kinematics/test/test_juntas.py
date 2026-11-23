"""El ir por juntas (juntas.py), sin ROS: con una placa de mentira que hace lo
que main_control.py (la consigna integra la velocidad que llega y no se adelanta
al motor mas de ADELANTO; el motor no empuja dentro de la banda).

    pytest src/scara_kinematics/test/test_juntas.py      (desde ~/ros2_ws)
"""
import math
import os
import re
from collections import deque

import numpy as np
import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import juntas as J
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R
from scara_kinematics import traycho as T

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PULSO = (360 / 5500, 360 / 5500 * 18 / 21, (3.14159265 * 10 + 10) / 5500)   # lo que cuenta el encoder


def _yaml(nombre):
    with open(os.path.join(PAQUETE, 'config', nombre)) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def _cal():
    mp = _yaml('micropython.yaml')
    return A.Calibracion.desde_parametros(lambda n, d: mp.get(n, d))


def _robot():
    """Con los topes que le pone traycho_juntas.launch.py a juntas_nodo: los de
    los motores, Teodoro no por debajo de TEODORO_MIN_MM y Simon no mas cerca de
    la base que SIMON_MAX (el lado de q2_min: test_home_lejos)."""
    rp = _yaml('scara_urdf3.yaml')
    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    lim = list(PV.limites_junta(_cal(), robot, 0.5))
    lim[1] = (max(lim[1][0], _cal().a_junta([0.0, J.SIMON_MAX, 0.0])[1]), lim[1][1])
    lim[2] = (max(lim[2][0], _cal().a_junta([0.0, 0.0, J.TEODORO_MIN_MM])[2]), lim[2][1])
    return PV.robot_con_limites(robot, lim)


def _home():
    """Los tres motores en 0 (Simon con el antebrazo contra la base, Teodoro
    arriba), en grados y mm: el HOME de antes del 24/10 por la tarde."""
    return J.a_grados(_cal().a_junta([0.0, 0.0, 0.0]))


class Placa:
    """main_control.py y sus motores, por junta (grados, grados, mm), cada 10 ms.

    vmax: lo que da cada motor (si es menos que la V pedida, la consigna le
    espera); retraso: tics de 10 ms de la V hasta la placa."""

    def __init__(self, q0, vmax=(100.0, 200.0, 1.5), retraso=3, banda=(0.3, 0.26, 0.12),
                 pegado_s=0.0):
        # empujando desde parado, tanto sin moverse: Alvin y Simon (Teodoro, no:
        # en el brazo baja y sube a 1 mm/s)
        self.pegado_s = np.array([pegado_s, pegado_s, 0.0])
        self.t_empuje = np.zeros(3)
        self.pos = np.array(q0, dtype=float)
        self.ref = self.pos.copy()
        self.vmax = np.array(vmax)
        self.banda = np.array(banda)
        self.cola = deque([np.zeros(3)] * retraso)
        self.asentado = np.zeros(3, dtype=bool)
        self.vel = np.zeros(3)

    def medida(self):
        return np.round(self.pos / PULSO) * PULSO

    def paso(self, v_pc, s=0.01):
        self.cola.append(np.asarray(v_pc, dtype=float))
        v = self.cola.popleft()
        adel = np.array(J.ADELANTO)
        nuevo = self.ref + v * s
        nuevo = np.where(v > 0, np.minimum(nuevo, np.maximum(self.ref, self.pos + adel)), nuevo)
        nuevo = np.where(v < 0, np.maximum(nuevo, np.minimum(self.ref, self.pos - adel)), nuevo)
        self.ref = nuevo
        e = self.ref - self.pos
        quieta = np.abs(v) < np.array(J.V_CORREGIR_MIN) * 0.8          # VEL_QUIETA
        dentro = (np.abs(e) < self.banda) | (quieta & self.asentado & (np.abs(e) < 2 * self.banda))
        self.asentado = np.where(dentro, quieta, False)
        # la anticipativa (v) mas el error, y nunca menos que lo que arranca
        u = np.clip(v + 20.0 * e, -self.vmax, self.vmax)
        u = np.where(np.abs(u) < 0.1 * self.vmax, np.sign(e) * 0.1 * self.vmax, u)
        # con inercia: un poco de lo que llevaba; y si estaba parado, no arranca
        # hasta que lleva pegado_s empujando (Alvin: la placa sube el PWM)
        self.t_empuje = np.where(dentro, 0.0, self.t_empuje + s)
        arranca = self.t_empuje >= self.pegado_s
        self.vel = np.where(dentro, 0.3 * self.vel,
                            np.where(arranca, 0.5 * self.vel + 0.5 * u, 0.0))
        self.pos = self.pos + self.vel * s


def _correr(j, placa, t_max=120.0):
    """El nodo cada 20 ms; la placa cada 10 y su seguimiento cada 200. Devuelve
    lo que mando el nodo en cada tic: (fase, v)."""
    t, v, seg, t_seg, mandado = 0.0, np.zeros(3), None, None, []
    for k in range(int(t_max / 0.01)):
        if k % 2 == 0:
            fase = j.fase
            v = j.paso(t, placa.medida(), seg, t_seg)
            mandado.append((fase, v.copy()))
            if not j.activo():
                break
        placa.paso(v)
        t += 0.01
        if k % 20 == 19:
            seg, t_seg = placa.ref - placa.pos, t
    return mandado


def test_las_velocidades_son_las_del_vector():
    assert J.velocidades(_cal()) == pytest.approx([84.0, 180.0, 1.0])     # 14 y 30 RPM, 60 mm/min


def test_el_seguimiento_de_la_placa_en_junta():
    """Alvin y Simon con signo -1: 0.5 grados de motor por delante son -0.5 de junta."""
    assert J.seguimiento_a_junta([0.5, -0.2, 0.1], _cal()) == pytest.approx([-0.5, 0.2, 0.1])


def test_cada_junta_a_su_velocidad_y_sin_corregir_por_el_camino():
    robot = _robot()
    j = J.Juntas(robot, J.velocidades(_cal()))
    q0 = _home()
    j.ir((135.0, -355.0, 216.0), q0, 0.0)
    mandado = _correr(j, Placa(q0))
    en_ir = [v for fase, v in mandado if fase == 'ir']
    assert en_ir
    for v in en_ir:
        # cada una, a 0 o a su velocidad entera: nada de un poco mas o menos
        assert all(x == 0.0 or abs(x) == pytest.approx(V) for x, V in zip(v, j.vel))
    assert j.fase == 'hecho'


@pytest.mark.parametrize('vmax, retraso, pegado_s', [
    ((100.0, 200.0, 1.5), 3, 0.0),   # los motores dan la V
    ((40.0, 90.0, 0.8), 3, 0.0),     # no la dan: la consigna les espera
    ((100.0, 200.0, 1.5), 8, 0.0),   # la orden tarda 80 ms: se pasa y vuelve
    ((100.0, 200.0, 1.5), 3, 0.25),  # tardan en arrancar (Alvin en el brazo, 23/10)
])
def test_el_traycho_entero_llega_a_cada_punto(vmax, retraso, pegado_s):
    """Desde HOME (Simon y Teodoro en su tope), todas las idas del traycho:
    cada una acaba dentro de la tolerancia, con 3 correcciones como mucho."""
    robot = _robot()
    j = J.Juntas(robot, J.velocidades(_cal()))
    placa = Placa(_home(), vmax, retraso, pegado_s=pegado_s)
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, robot,
                             desde=R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot)),
                             z_arriba=pts[0][2])
    for nombre, p in tramos:
        dura = j.ir(p, placa.medida(), 0.0)
        j.avisos.clear()
        _correr(j, placa, t_max=120.0 + 3.0 * dura)   # 29/10: Teodoro baja 115 mm
        assert j.fase == 'hecho', nombre
        nivel, texto = j.avisos[-1]
        assert nivel == 20 and texto.startswith('ir terminada'), (nombre, texto)
        assert np.all(np.abs(j.obj - placa.medida()) <= j.tol), (nombre, texto)
        assert j.n_corr <= J.CORRECCIONES
        assert np.linalg.norm(p - R.fk_scara_rrp(J.a_rad(placa.medida()), robot)) < 4.0


def test_el_recorrido_del_boton_entiende_lo_que_dice():
    """Los mensajes son los que espera traycho.Recorrido (los de control_nodo)."""
    robot = _robot()
    j = J.Juntas(robot, J.velocidades(_cal()))
    placa = Placa(_home())
    destino = (135.0, -355.0, 216.0)
    mandado = []
    r = T.Recorrido([('P1', np.array(destino))], lambda texto: mandado.append(texto) or True,
                    lambda nivel, texto: None)
    r.paso(0.0)
    assert mandado == ['ir 135.0 -355.0 216.0']
    j.ir([float(x) for x in mandado[0].split()[1:]], placa.medida(), 0.0)
    _correr(j, placa)
    for nivel, texto in j.avisos:
        r.aviso('control', nivel, texto)
    r.paso(1.0)
    assert r.fase == 'hecho', j.avisos


def test_parar_y_si_no_cabe():
    robot = _robot()
    j = J.Juntas(robot, J.velocidades(_cal()))
    q0 = _home()
    with pytest.raises(R.RutaError):
        j.ir((600.0, 0.0, 200.0), q0, 0.0)
    with pytest.raises(R.RutaError, match='d3'):
        j.ir((135.0, -355.0, 300.0), q0, 0.0)                # Teodoro no sube tanto
    j.ir((135.0, -355.0, 216.0), q0, 0.0)
    assert np.any(j.paso(0.0, q0) != 0.0)
    j.parar('la placa ha soltado los motores')
    assert np.all(j.paso(0.02, q0) == 0.0) and not j.activo()
    assert j.avisos[-1] == (30, 'parado: la placa ha soltado los motores')


def test_si_no_se_mueve_se_para_y_lo_dice():
    robot = _robot()
    j = J.Juntas(robot, J.velocidades(_cal()))
    q0 = _home()
    dura = j.ir((135.0, -355.0, 216.0), q0, 0.0)
    t = 0.0
    while j.activo() and t < 3 * dura + 20.0:
        j.paso(t, q0)                                        # los motores, sueltos
        t += 0.02
    assert not j.activo() and j.avisos[-1][1].startswith('parado:')
    assert 'no llega' in j.avisos[-1][1]


def test_los_tramos_salen_del_tope_y_no_pasan_de_20_grados():
    robot = _robot()
    home = R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot))
    with pytest.raises(R.RutaError, match='theta2'):
        R.ik_scara_rrp(home, robot, 'down')                  # HOME esta en el tope de Simon
    tramos = J.tramos_juntas(J.puntos_mm(), robot, desde=home)
    qa = _home()
    llegadas = []
    for nombre, p in tramos:
        qb = J.objetivo(p, robot)                            # dentro de los topes
        assert np.all(np.abs(qb[:2] - qa[:2]) <= T.GRADOS_MAX + 1e-6), nombre
        if re.fullmatch(r'P\d+( \((\d+)/\2\))?', nombre):
            llegadas.append(p)
        qa = qb
    assert np.allclose(llegadas, J.puntos_mm())


def test_el_camino_que_se_pinta_acaba_en_el_punto():
    robot = _robot()
    vel = J.velocidades(_cal())
    obj = J.objetivo((135.0, -355.0, 216.0), robot)
    c = J.camino(_home(), obj, vel, robot)
    assert np.allclose(c[-1], (135.0, -355.0, 216.0), atol=1e-6)
    assert np.allclose(c[0], R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot)))
    # Simon (180 grados/s) acaba antes que Alvin (84): no es una recta
    a, b = c[0][:2], c[-1][:2]
    u = (b - a) / np.linalg.norm(b - a)
    lejos = max(abs(u[0] * (p[1] - a[1]) - u[1] * (p[0] - a[0])) for p in c)
    assert lejos > 1.0


def test_el_nodo_dice_avisos_de_todos_los_niveles():
    """rclpy no deja escribir con niveles distintos desde la misma linea: con
    un warning detras de un info, juntas_nodo se caia (23/10, placa simulada)."""
    import rclpy
    from scara_kinematics import juntas_nodo as JN
    rclpy.init()
    try:
        nodo = JN.JuntasNodo()
        for nivel in (20, 30, 40, 20, 30):
            nodo.ir.avisos.append((nivel, f'aviso de nivel {nivel}'))
            nodo._avisos()
        assert nodo.ir.avisos == []
        nodo.destroy_node()
    finally:
        rclpy.shutdown()


def test_p1_es_el_home_y_la_figura_es_la_suya():
    """«home es P1»: P1 en x = 0 con Simon en SIMON_HOME (24/10, tarde: el de
    antes, con Simon en 0, era el antebrazo contra la base) y Teodoro arriba, la
    misma figura que en su codigo (girada, sin escalar) y todo lejos de los topes."""
    robot, cal = _robot(), _cal()
    p = J.puntos_mm()
    assert p[0][0] == 0.0
    m = cal.a_motor(J.a_rad(J.objetivo(p[0], robot)))
    assert m[1] == pytest.approx(J.SIMON_HOME, abs=0.05)
    assert m[2] == pytest.approx(-0.5, abs=1.0)
    assert np.allclose(p[0], p[-1])
    suyos = np.array(T.PUNTOS_CM) * 10.0
    d = lambda a: np.linalg.norm(a[:, None, :2] - a[None, :, :2], axis=2)
    assert np.allclose(d(p), d(suyos))                       # las mismas distancias
    z = T.puntos_mm()[:, 2]                                  # las z, las de traycho.py
    for i in J.PUNTOS_SOPORTE:                               # menos P3, P5 y P7 (29/10)
        z[i - 1] = max(z[i - 1], J.Z_SOPORTE_MM)
    for i in J.PUNTOS_HONDOS:                                # y P2, P4 y P6 (30/10)
        z[i - 1] = max(z[i - 1], J.Z_HONDOS_MM)
    assert np.allclose(p[:, 2], z)
    lo = np.degrees([robot.lim[0][0], robot.lim[1][0]])
    hi = np.degrees([robot.lim[0][1], robot.lim[1][1]])
    for q in (J.objetivo(x, robot)[:2] for x in p[1:-1]):
        assert np.all(q - lo > 9.0) and np.all(hi - q > 9.0)


def test_en_planta_solo_va_arriba_y_en_cada_punto_baja_y_sube():
    """23/10: «no subio, quedo abajo y se choco, debe bajar y despues subir»."""
    robot = _robot()
    pts = J.puntos_mm()
    z_arriba = pts[0][2]
    home = R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot))
    tramos = J.tramos_juntas(pts, robot, desde=home, z_arriba=z_arriba)
    a = home
    for nombre, b in tramos:
        en_planta = np.linalg.norm(b[:2] - a[:2]) > 1e-6
        if en_planta:                                   # en planta, arriba y sin tocar z
            # (saliendo de mas arriba, de Teodoro en 0, a trozos: entre las dos)
            assert a[2] >= z_arriba - 1e-9, nombre
            assert z_arriba - 1e-9 <= b[2] <= max(a[2], z_arriba) + 1e-9, nombre
        a = b
    # cada punto bajo: encima, baja, sube; y los puntos, en orden
    nombres = [n for n, _ in tramos]
    for i, p in enumerate(pts, 1):
        if p[2] < z_arriba:
            k = nombres.index(f'P{i}')
            assert nombres[k - 1].startswith(f'P{i} encima') and nombres[k + 1] == f'P{i} sube'
            assert np.allclose(tramos[k][1], p)
            assert np.allclose(tramos[k - 1][1], tramos[k + 1][1])
    llegadas = [p for n, p in tramos if re.fullmatch(r'P\d+( \((\d+)/\2\))?', n)]
    assert np.allclose(llegadas, pts)


def test_si_esta_abajo_lo_primero_es_subir():
    robot = _robot()
    pts = J.puntos_mm()
    abajo = (-150.0, -300.0, 170.0)
    tramos = J.tramos_juntas(pts, robot, desde=abajo, z_arriba=pts[0][2])
    nombre, p = tramos[0]
    assert nombre == 'sube'
    assert np.allclose(p[:2], abajo[:2], atol=1e-6) and p[2] == pytest.approx(pts[0][2])


def test_teodoro_no_baja_hasta_el_soporte():
    """23/10: con el soporte cilindrico, a -47 mm chocaba, y bajando a P4 casi
    (lo paro en -10, con el cero mal). Ni el traycho ni un ir suelto lo bajan de
    TEODORO_MIN_MM (29/10: -120; el traycho, desde el 30/10, llega a -52.1)."""
    robot, cal = _robot(), _cal()
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, robot, desde=pts[0], z_arriba=pts[0][2])
    teodoro = [cal.a_motor(J.a_rad(J.objetivo(p, robot)))[2] for _, p in tramos]
    assert min(teodoro) >= J.TEODORO_MIN_MM and min(teodoro) == pytest.approx(-52.11, abs=0.01)
    j = J.Juntas(robot, J.velocidades(cal))
    for teo in (-130.0, -125.0):
        abajo = R.fk_scara_rrp(cal.a_junta([0.0, -20.0, teo]), J.sin_topes(robot))
        with pytest.raises(R.RutaError, match='d3'):
            j.ir(abajo, _home(), 0.0)


def test_el_camino_que_se_pinta_no_sale_de_donde_llega():
    """23/10: la planta pintaba rectas, y la de P2 a P3 cruzaba la zona blanca
    (donde no llega). El camino por juntas no sale de los topes."""
    robot = _robot()
    pts = J.puntos_mm()
    c = J.camino_planta(pts, robot, z_arriba=pts[0][2])
    libre = J.sin_topes(robot)
    for p in c:
        q = R.ik_scara_rrp(p, libre, 'down')
        assert robot.fuera(q, tol=1e-6) is None, p
    # y con el HOME de entonces, (0, -273), la recta de P2 a P3 si se sale: era
    # el dibujo lo que estaba mal (con el de ahora, mas lejos de la base, ya no)
    viejos = J.puntos_mm(p1=(0.0, -273.0))
    a, b = viejos[1], viejos[2]
    assert any(robot.fuera(R.ik_scara_rrp(a + (b - a) * s, libre, 'down')) for s in
               np.linspace(0.0, 1.0, 50))


def test_ir_con_una_z_que_no_alcanza_va_a_la_mas_cercana():
    """23/10: la casilla Z de la ventana venia en 150, por debajo del soporte, y
    «Ir en recta» no salia nunca. La z se lleva a la que Teodoro alcanza (29/10:
    con el tope en -120, 150 ya llega; se prueba con 90)."""
    import rclpy
    from scara_kinematics import juntas_nodo as JN
    rclpy.init()
    try:
        nodo = JN.JuntasNodo()
        nodo.robot = r = _robot()                  # el del launch, con el soporte
        nodo.ir = J.Juntas(r, J.velocidades(_cal()), nodo.dt)
        z = sorted(r.z_ref + r.z_dir * d for d in r.lim[2])
        nodo.q = J.objetivo((0.0, -300.0, z[1] - 1.0), r)
        nodo.placa = 'ACTIVA'
        nodo._orden('ir -94 -311 90 v=20 perfil=trapecio', 0.0)
        assert nodo.ir.activo()
        assert np.allclose(nodo.ir.destino, (-94.0, -311.0, z[0] + 0.01))
        nodo.destroy_node()
    finally:
        rclpy.shutdown()


def test_home_va_a_p1():
    """24/10: «pon bien el home que es P1». Antes, home eran los tres motores en
    0 (Simon contra su tope); ahora es P1, arriba."""
    import rclpy
    from scara_kinematics import juntas_nodo as JN
    rclpy.init()
    try:
        nodo = JN.JuntasNodo()
        nodo.robot = r = _robot()
        nodo.ir = J.Juntas(r, J.velocidades(_cal()), nodo.dt)
        z = sorted(r.z_ref + r.z_dir * d for d in r.lim[2])
        nodo.q = J.objetivo((60.0, -300.0, z[1] - 3.0), r)
        nodo.placa = 'ACTIVA'
        nodo._orden('home', 0.0)
        assert nodo.ir.activo()
        assert nodo.ir.nombre == 'home'
        p1 = J.puntos_mm()[0]
        assert np.allclose(nodo.ir.destino[:2], p1[:2])
        assert z[0] < nodo.ir.destino[2] < z[1]
        assert abs(nodo.ir.destino[2] - min(p1[2], z[1] - 0.01)) < 1e-6
        nodo.destroy_node()
    finally:
        rclpy.shutdown()


def test_p3_p5_p7_no_bajan_hasta_el_soporte():
    """29/10: con las z de su codigo, bajando a P3 se bloqueo en -54.9 («es el
    soporte, limita P3 P5 P7 y sigue»). P3, P5 y P7 se quedan en Z_SOPORTE_MM
    (Teodoro a -50); P1 y P8 siguen con las z de su codigo."""
    pts, suyos = J.puntos_mm(), T.puntos_mm()
    assert all(pts[i - 1, 2] == J.Z_SOPORTE_MM for i in J.PUNTOS_SOPORTE)
    assert np.allclose(pts[[0, 7], 2], suyos[[0, 7], 2])
    teodoro = [_cal().a_motor(J.a_rad(J.objetivo(pts[i - 1], _robot())))[2]
               for i in J.PUNTOS_SOPORTE]
    assert teodoro == pytest.approx([-50.0] * 3, abs=0.02)


def test_p2_p4_p6_no_bajan_hasta_donde_chocaba():
    """30/10: bajando a P2 (a -115.5) lo paro en -52.2: «deja a teodoro que baje
    hasta ahi, porque se iba a chocar». P2, P4 y P6 se quedan en Z_HONDOS_MM
    (Teodoro a -52.1, sin pasar de donde lo paro)."""
    pts = J.puntos_mm()
    assert all(pts[i - 1, 2] == J.Z_HONDOS_MM for i in J.PUNTOS_HONDOS)
    teodoro = [_cal().a_motor(J.a_rad(J.objetivo(pts[i - 1], _robot())))[2]
               for i in J.PUNTOS_HONDOS]
    assert teodoro == pytest.approx([-52.11] * 3, abs=0.02)
    assert min(teodoro) > -52.17
