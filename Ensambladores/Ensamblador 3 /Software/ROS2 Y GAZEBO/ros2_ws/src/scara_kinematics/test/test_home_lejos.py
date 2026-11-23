"""HOME mas lejos de la base (24/10, tarde). Sergio: «choca con la base, o sea
con el robot, creo que dejamos el home muy pegado»; con Simon en -10, «ponlo como
entre 5 a 10 grados mas lejos» (-17.5); tras el traycho de las 16:47, «es
mejor que alejemos el home por ahi 10 grados» (-27.5). En Simon 0 (la cuenta de
la placa) el antebrazo toca la base: P1 va con Simon en SIMON_HOME y Simon no
pasa de SIMON_MAX."""
import importlib.util
import math
import os
import re

import numpy as np
import pytest

from scara_kinematics import juntas as J
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

from test_juntas import Placa, _cal, _correr, _home, _robot

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _launch():
    ruta = os.path.join(PAQUETE, 'launch', 'traycho_juntas.launch.py')
    spec = importlib.util.spec_from_file_location('traycho_juntas_launch', ruta)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_p1_esta_10_grados_mas_lejos_que_el_de_las_16_47():
    cal, robot = _cal(), _robot()
    m = cal.a_motor(J.a_rad(J.objetivo(J.puntos_mm()[0], robot)))
    assert m[1] == pytest.approx(-17.5 - 10.0, abs=0.05)
    assert m[1] == pytest.approx(J.SIMON_HOME, abs=0.05)
    assert J.SIMON_HOME < J.SIMON_MAX < 0.0


def test_el_launch_no_deja_a_simon_acercarse_a_la_base():
    """El tope de Simon del lado de la base (q2_min) en SIMON_MAX, y el del otro
    lado como estaba. Igual que el _robot de los tests."""
    L, cal = _launch(), _cal()
    rp = _robot()
    lim = PV.limites_junta(cal, rp, 0.5)
    q2 = L.simon_lejos(cal, lim[1], J.SIMON_MAX)
    assert q2[0] == pytest.approx(cal.a_junta([0.0, J.SIMON_MAX, 0.0])[1])
    assert q2[1] == pytest.approx(lim[1][1])
    assert q2 == pytest.approx(list(rp.lim[1]))
    # el HOME de antes ya no cabe
    with pytest.raises(R.RutaError):
        J.objetivo((0.0, -273.0, J.puntos_mm()[0][2]), rp)
    # y con simon_max:=0 se queda como antes (el margen de 0.5)
    assert L.simon_lejos(cal, lim[1], 0.0) == pytest.approx(list(lim[1]))


def test_desde_la_base_sale_a_p1_y_hace_el_traycho():
    """La placa en 0 (el antebrazo contra la base, donde se fijo): el primer ir,
    a P1, lo saca hacia fuera, y el traycho entero no vuelve a acercarse."""
    robot = _robot()
    cal = _cal()
    j = J.Juntas(robot, J.velocidades(cal))
    placa = Placa(_home(), (30.0, 30.0, 1.5), 3)
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, robot,
                             desde=R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot)),
                             z_arriba=pts[0][2])
    simon, nombres = [], []
    for nombre, p in tramos:
        j.ir(p, placa.medida(), 0.0)
        j.avisos.clear()
        _correr(j, placa)
        assert j.fase == 'hecho', nombre
        simon.append(cal.a_motor(J.a_rad(placa.medida()))[1])
        nombres.append(nombre)
    # de la base a P1 son 27.5 grados de Simon: en trozos de 20 como mucho
    primero = next(i for i, n in enumerate(nombres) if re.fullmatch(r'P1( \((\d+)/\2\))?', n))
    assert primero == 1 and nombres[0] == 'P1 (1/2)'
    assert simon[primero] == pytest.approx(J.SIMON_HOME, abs=0.5)
    assert max(simon) < J.SIMON_MAX
    assert simon[-1] == pytest.approx(J.SIMON_HOME, abs=0.5)


def test_el_home_queda_9_cm_mas_afuera_que_el_de_antes():
    p1 = np.asarray(J.puntos_mm()[0][:2])
    assert math.hypot(*p1) == pytest.approx(362.3, abs=0.1)
    assert math.hypot(*p1) - 273.0 > 85.0
