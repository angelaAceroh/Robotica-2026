"""Simon lento pero seguro (24/10): su transmision resbala, asi que la placa no
le da mas de PWM 700 (550 hasta el 19/11: con 550 se paraba en -33.2) y el
traycho por juntas lo lleva a 30 grados/s."""
import importlib.util
import os
import re
from dataclasses import replace

import numpy as np
import pytest

from scara_kinematics import juntas as J
from scara_kinematics import ruta as R

from test_juntas import Placa, _cal, _correr, _home, _robot

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _launch():
    ruta = os.path.join(PAQUETE, 'launch', 'traycho_juntas.launch.py')
    spec = importlib.util.spec_from_file_location('traycho_juntas_launch', ruta)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_la_placa_no_le_da_a_simon_mas_de_700():
    with open(os.path.join(PAQUETE, 'firmware', 'micropython', 'main_control.py')) as f:
        ganancias = f.read().split('\nGANANCIAS = [', 1)[1]
    fila = re.search(r"^\s*\('Simon',([^)]*)\)", ganancias, re.M).group(1)
    kp, ki, kv, arr, arr_menos, tope, *_ = [float(x) for x in fila.split(',')]
    assert tope == 700 and arr <= tope and arr_menos <= tope


def test_el_launch_lleva_a_simon_a_30_y_a_los_otros_igual():
    L = _launch()
    assert L.SIMON_LENTO == 30.0
    rpm, vel_max = L.simon_lento([14.0, 30.0, 60.0], L.SIMON_LENTO)
    assert rpm == pytest.approx([14.0, 5.0, 60.0])            # 5 RPM = 30 grados/s
    assert vel_max == pytest.approx([90.0, 36.0, 10.0])
    # y a juntas_nodo le llega eso: Simon a 30, Alvin y Teodoro como en el yaml
    assert J.velocidades(replace(_cal(), velocidades=rpm)) == pytest.approx([84.0, 30.0, 1.0])


def test_vel_simon_no_lo_sube_por_encima_del_yaml_ni_de_la_placa():
    L = _launch()
    rpm, vel_max = L.simon_lento([14.0, 30.0, 60.0], 500.0)
    assert rpm[1] == 30.0 and vel_max[1] == L.VEL_MAX_MOTOR[1]


@pytest.mark.parametrize('vmax_simon', [30.0, 20.0])
def test_el_traycho_entero_con_simon_lento(vmax_simon):
    """Simon a 30 grados/s, y tambien si con PWM 550 no llega a tanto (20): la
    consigna le espera y cada ida acaba en su punto."""
    robot = _robot()
    L = _launch()
    rpm, _ = L.simon_lento(_cal().velocidades, L.SIMON_LENTO)
    j = J.Juntas(robot, J.velocidades(replace(_cal(), velocidades=rpm)))
    placa = Placa(_home(), (100.0, vmax_simon, 1.5), 3)
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, robot,
                             desde=R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot)),
                             z_arriba=pts[0][2])
    for nombre, p in tramos:
        j.ir(p, placa.medida(), 0.0)
        j.avisos.clear()
        mandado = _correr(j, placa)
        assert all(abs(v[1]) <= 30.0 + 1e-9 for _, v in mandado), nombre
        assert j.fase == 'hecho', nombre
        assert np.all(np.abs(j.obj - placa.medida()) <= j.tol), nombre
