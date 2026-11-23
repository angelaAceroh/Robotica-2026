"""Comprueba control_cinematico.py contra la teoria, con la planta ideal.

Si cada junta hace la velocidad que se le manda, el control cinematico deja el
error en e(t) = e0 * exp(-K t). Estos tests son eso y las protecciones.

    pytest src/scara_kinematics/test/test_control.py          (desde ~/ros2_ws)
"""
import math

import numpy as np
import pytest

from scara_kinematics import control_cinematico as C
from scara_kinematics import ruta as R

ROB = R.ROBOT
Q_HOME = R.ik_scara_rrp(R.HOME, ROB, 'down')


def _norma(h):
    return np.linalg.norm(h['e'], axis=1)


def test_la_inversa_es_la_del_jacobiano():
    ley = C.Ley()
    rnd = np.random.default_rng(1)
    for _ in range(200):
        p = R.HOME + rnd.uniform([-60, -60, -40], [60, 60, 40])
        q = R.ik_scara_rrp(p, ROB, 'down')
        u = rnd.uniform(-50, 50, 3)
        qd, lam = ley.inversa(q, u)
        assert lam == 0.0
        assert np.allclose(R.jacobiano_scara_rrp(q, ROB) @ qd, u, atol=1e-9)


@pytest.mark.parametrize('k', [2.0, 5.0, 10.0])
def test_un_salto_cae_con_tau_igual_a_1_entre_k(k):
    """Regulacion: el error cae al 37 % en 1/K segundos."""
    ley = C.Ley(k=k)
    h = C.simular(C.Fija(R.HOME + [8.0, -6.0, 0.0]), Q_HOME, ley, dt=0.001,
                  t_extra=6.0 / k)
    assert h['sat'].max() < 1.0                       # sin saturar: la teoria pura
    assert C.tau(h['t'], _norma(h)) == pytest.approx(1.0 / k, rel=0.02)
    assert _norma(h)[-1] < 8.0 * 1.02 * math.exp(-5.5)


def test_el_salto_va_en_linea_recta():
    """e_dot = -K e: el error no cambia de direccion, la punta va recta."""
    p_fin = R.HOME + [40.0, 30.0, 4.0]
    h = C.simular(C.Fija(p_fin), Q_HOME, C.Ley(k=5.0), dt=0.001, t_extra=1.5)
    a, d = R.HOME, p_fin - R.HOME
    d = d / np.linalg.norm(d)
    fuera = [np.linalg.norm((p - a) - np.dot(p - a, d) * d) for p in h['p']]
    assert max(fuera) < 0.05                          # mm, en 50 mm de recorrido


def test_la_saturacion_no_tuerce_la_punta():
    """Recortadas por el mismo factor, la punta sigue la misma direccion."""
    ley = C.Ley(k=50.0)
    pd = R.HOME + [80.0, 60.0, 30.0]
    s = ley.paso(Q_HOME, pd, np.zeros(3))
    assert s.sat > 1.0
    assert np.all(np.abs(s.qd) <= ley.qdot_max + 1e-12)
    v = R.jacobiano_scara_rrp(Q_HOME, ROB) @ s.qd
    assert np.allclose(v / np.linalg.norm(v), s.e / np.linalg.norm(s.e), atol=1e-9)


def test_con_retraso_sigue_y_con_demasiada_k_no():
    """Discreto y con retraso, una K demasiado alta oscila y se va."""
    ref = C.Fija(R.HOME + [5.0, 0.0, 0.0])
    bien = C.simular(ref, Q_HOME, C.Ley(k=5.0), dt=0.01, retraso=2, t_extra=2.0)
    assert _norma(bien)[-1] < 0.01
    mal = C.simular(ref, Q_HOME, C.Ley(k=80.0), dt=0.01, retraso=2, t_extra=2.0)
    assert _norma(mal)[-1] > 1.0


class _SinFF(C.Referencia):
    """La misma referencia sin xd_dot: solo el termino K*e."""

    def __init__(self, ref):
        self.ref, self.duracion = ref, ref.duracion

    def en(self, t):
        return self.ref.en(t)[0], np.zeros(3)


def test_sigue_un_circulo_con_feed_forward():
    circ = C.Circulo((330.0, 0.0), 40.0, 150.0, 20.0)
    q0 = R.ik_scara_rrp(circ.inicio(), ROB, 'down')
    h = C.simular(circ, q0, C.Ley(k=5.0), dt=0.01, retraso=1)
    assert _norma(h).max() < 0.1
    # Sin el feed-forward es un lazo P persiguiendo una rampa: va v/K por detras
    sin_ff = C.simular(_SinFF(circ), q0, C.Ley(k=5.0), dt=0.01, retraso=1)
    assert _norma(sin_ff).max() == pytest.approx(20.0 / 5.0, rel=0.1)


def test_arranca_fuera_del_camino_y_converge():
    circ = C.Circulo((330.0, 0.0), 40.0, 150.0, 20.0)
    q0 = R.ik_scara_rrp(circ.inicio() + [0.0, 15.0, 0.0], ROB, 'down')
    h = C.simular(circ, q0, C.Ley(k=5.0), dt=0.01, retraso=1)
    e = _norma(h)
    assert e[0] == pytest.approx(15.0, rel=1e-6)
    assert h['sat'].max() < 1.0
    assert C.tau(h['t'], e) == pytest.approx(0.2, rel=0.05)
    assert e[h['t'] > 2.0].max() < 0.1


def test_el_husillo_satura_y_frena_a_las_tres():
    """5 mm en z con K = 5 piden 25 mm/s y el husillo da 8: hasta que el error
    baja de 8/K = 1.6 mm, cae en rampa, no en exponencial, y el plano espera."""
    p0 = R.fk_scara_rrp(Q_HOME, ROB)
    h = C.simular(C.Fija(p0 + [10.0, 0.0, 5.0]), Q_HOME, C.Ley(k=5.0), dt=0.001,
                  t_extra=2.0)
    assert h['sat'][0] == pytest.approx(25.0 / 8.0, rel=1e-6)
    assert np.all(np.abs(h['qd'][:, 2]) <= 8.0 + 1e-9)
    assert C.tau(h['t'], _norma(h)) > 1.5 / 5.0
    # Recortadas por igual, la punta sigue yendo en linea recta
    d = np.array([10.0, 0.0, 5.0]) / math.hypot(10.0, 5.0)
    fuera = [np.linalg.norm((p - p0) - np.dot(p - p0, d) * d) for p in h['p']]
    assert max(fuera) < 0.02


def test_un_tope_no_se_deja_pasar_pero_si_salir():
    ley = C.Ley()
    q = np.array([Q_HOME[0], Q_HOME[1], R.D3_LIM[0]])        # husillo abajo del todo
    p = R.fk_scara_rrp(q, ROB)
    baja = ley.paso(q, p + [0.0, 0.0, -10.0], np.zeros(3))
    assert baja.qd[2] == 0.0 and baja.topes == ('d3',)
    sube = ley.paso(q, p + [0.0, 0.0, 10.0], np.zeros(3))
    assert sube.qd[2] > 0.0 and sube.topes == ()


def test_casi_estirado_no_pide_velocidades_enormes():
    ley = C.Ley()
    q = np.array([0.0, ROB.b1 - ROB.b2 + 0.01, 50.0])         # 0.6 deg de estirado
    s = ley.paso(q, R.fk_scara_rrp(q, ROB) + [5.0, 5.0, 0.0], np.zeros(3))
    assert s.amort > 0.0
    assert s.sat < 1.0


def test_las_referencias_dan_su_propia_derivada():
    refs = [C.Recta(R.HOME, R.P4, 30.0),
            C.Circulo((330.0, 0.0), 40.0, 150.0, 20.0, vueltas=1.5),
            C.Articular([0.0, 0.0, 30.0], Q_HOME)]
    for ref in refs:
        for f in (0.13, 0.5, 0.81):
            t, h = f * ref.duracion, 1e-5
            num = (ref.en(t + h)[0] - ref.en(t - h)[0]) / (2 * h)
            assert np.allclose(num, ref.en(t)[1], atol=1e-4), ref


def test_la_aproximacion_empieza_donde_esta_el_brazo():
    q_rep = np.array([0.0, 0.0, 30.0])
    ap = C.Articular(q_rep, Q_HOME)
    assert np.allclose(ap.inicio(), R.fk_scara_rrp(q_rep, ROB))
    assert np.allclose(ap.fin(), R.HOME, atol=1e-9)
    pico = max(np.max(np.abs(ap.q(t)[1]) / C.QDOT_MAX)
               for t in np.linspace(0.0, ap.duracion, 300))
    assert pico <= C.MARGEN_VEL + 1e-3


def test_revisar_rechaza_lo_que_no_cabe():
    with pytest.raises(C.ControlError, match='alcance'):
        C.revisar(C.Fija([600.0, 0.0, 150.0]), ROB, 'down')
    with pytest.raises(C.ControlError, match='estirado'):     # llega a 500 antes
        C.revisar(C.Recta(R.HOME, [600.0, 0.0, 150.0], 40.0), ROB, 'down')
    with pytest.raises(C.ControlError):
        C.revisar(C.Recta(R.HOME, [300.0, 0.0, 300.0], 40.0), ROB, 'down')   # z
    with pytest.raises(C.ControlError, match='estirado'):
        C.revisar(C.Recta(R.HOME, [499.5, 20.0, 150.0], 40.0), ROB, 'down')


def test_a_tiempo_estira_lo_que_va_demasiado_rapido():
    rc = C.Recta(R.P1, R.P2, 30.0)              # bajada de 40 mm a 30 mm/s: el husillo da 8
    f = C.a_tiempo(rc, ROB, 'down')
    assert f > 1.0
    assert C.revisar(rc, ROB, 'down') == pytest.approx(C.MARGEN_VEL, rel=1e-6)


@pytest.mark.parametrize('nombre', list(R.RUTAS))
def test_las_rutas_de_ruta_py_se_siguen(nombre):
    ruta = R.RUTAS[nombre]
    partes = []
    for i in range(ruta.shape[1] - 1):
        tipo = R.tipo_de_tramo(ruta[:, i], ruta[:, i + 1])
        rc = C.Recta(ruta[:, i], ruta[:, i + 1], R.VELOCIDADES[tipo], tipo=tipo)
        C.a_tiempo(rc, ROB, 'down')
        partes.append(rc)
    sec = C.Secuencia(nombre, partes)
    h = C.simular(sec, R.ik_scara_rrp(ruta[:, 0], ROB, 'down'), C.Ley(k=5.0),
                  dt=0.01, retraso=1, t_extra=1.0)
    assert _norma(h).max() < 0.5
    assert np.allclose(h['p'][-1], ruta[:, -1], atol=0.05)


def test_rama():
    assert C.rama(Q_HOME) == 'down'
    assert C.rama(R.ik_scara_rrp(R.HOME, ROB, 'up')) == 'up'
    assert C.rama([0.0, 0.0, 0.0]) == 'down'            # el reposo


def test_tau():
    t = np.linspace(0.0, 2.0, 2001)
    assert C.tau(t, 3.0 * np.exp(-t / 0.4)) == pytest.approx(0.4, abs=1e-4)
    assert C.tau(t, np.ones_like(t)) is None


def test_el_alcance_de_la_ventana_es_el_de_la_ik():
    """alcance_plano (el sombreado de control_gui) dice lo mismo que la IK y la
    comprobacion de singularidad que usa `revisar`, punto a punto."""
    rnd = np.random.default_rng(7)
    X = rnd.uniform(-520, 520, 3000)
    Y = rnd.uniform(-520, 520, 3000)
    a = C.alcance_plano(X, Y, ROB)
    for codo in ('down', 'up'):
        for x, y, ok in zip(X, Y, a[codo]):
            try:
                q = R.ik_scara_rrp([x, y, R.Z_SEGURO], ROB, codo)
                esperado = ROB.sin_codo(q) >= R.SIN_CODO_MIN
            except R.RutaError:
                esperado = False
            assert ok == esperado, (codo, x, y)
    # Las rutas de ruta.py y el circulo de partida caen dentro, con el codo down.
    puntos = np.hstack(list(R.RUTAS.values()))
    assert C.alcance_plano(puntos[0], puntos[1], ROB)['down'].all()
