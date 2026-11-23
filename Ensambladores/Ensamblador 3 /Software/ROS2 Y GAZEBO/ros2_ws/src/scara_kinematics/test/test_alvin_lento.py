"""Alvin lento (24/10, Sergio: «si vas muy rapido la circunferencia de soporte
se cae»): el traycho por juntas lo lleva a 30 grados/s en vez de 84."""
import importlib.util
import os
from dataclasses import replace

import numpy as np
import pytest

from scara_kinematics import banco_lazo as B
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


def _lentos(L, velocidades, vel_alvin=None):
    rpm, vel_max = L.simon_lento(velocidades, L.SIMON_LENTO)
    return L.alvin_lento(rpm, vel_max, L.ALVIN_LENTO if vel_alvin is None else vel_alvin)


def test_el_launch_lleva_a_alvin_y_a_simon_a_30():
    L = _launch()
    assert L.ALVIN_LENTO == 30.0
    rpm, vel_max = _lentos(L, [14.0, 30.0, 60.0])
    assert rpm == pytest.approx([5.0, 5.0, 60.0])             # 5 RPM = 30 grados/s
    assert vel_max == pytest.approx([36.0, 36.0, 10.0])
    assert J.velocidades(replace(_cal(), velocidades=rpm)) == pytest.approx([30.0, 30.0, 1.0])


def test_vel_alvin_no_lo_sube_por_encima_del_yaml_ni_de_la_placa():
    L = _launch()
    rpm, vel_max = _lentos(L, [14.0, 30.0, 60.0], 500.0)
    assert rpm[0] == 14.0 and vel_max[0] == L.VEL_MAX_MOTOR[0]


def test_el_traycho_entero_con_alvin_lento():
    robot = _robot()
    L = _launch()
    rpm, _ = _lentos(L, _cal().velocidades)
    j = J.Juntas(robot, J.velocidades(replace(_cal(), velocidades=rpm)))
    placa = Placa(_home(), (30.0, 30.0, 1.5), 3)
    pts = J.puntos_mm()
    tramos = J.tramos_juntas(pts, robot,
                             desde=R.fk_scara_rrp(J.a_rad(_home()), J.sin_topes(robot)),
                             z_arriba=pts[0][2])
    for nombre, p in tramos:
        j.ir(p, placa.medida(), 0.0)
        j.avisos.clear()
        mandado = _correr(j, placa)
        assert all(abs(v[0]) <= 30.0 + 1e-9 for _, v in mandado), nombre
        assert j.fase == 'hecho', nombre
        assert np.all(np.abs(j.obj - placa.medida()) <= j.tol), nombre


def test_a_30_alvin_va_seguido_y_llega():
    """Alvin necesita PWM 700 para arrancar: despacio podria ir a tirones (para
    y arranca), que es justo lo que no le conviene al soporte. En el banco, con
    las 11 variantes, un paso de 15 grados a 30 grados/s: yendo, sin pararse
    a mitad, y al final a menos de 0.6."""
    for etiqueta, fis in B.variantes('Alvin'):
        b = B.Banco('Alvin', None, fis, pos=-20.0)
        t0 = b.t
        obj = b.mover(15.0, 30.0, sujetar=2.0)
        a = b.desde(t0)
        assert b.fallo is None, (etiqueta, b.fallo)
        assert abs(obj - b.m.medida()) < 0.6, etiqueta
        # en la parte del medio del camino, no se queda parado (ni va hacia atras)
        x = a[:, 1]
        medio = (x > -20.0 + 3.0) & (x < -20.0 + 12.0)
        t_medio = a[medio, 0]
        assert t_medio.max() - t_medio.min() < 9.0 / 30.0 * 2.0, etiqueta
        assert B.inversiones(x[medio], 0.2) == 0, etiqueta
