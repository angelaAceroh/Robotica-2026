"""control_puntos: lo que se puede comprobar sin ROS (leer los puntos, que los
de la fase 1 quepan en el brazo, las graficas).

    pytest src/scara_kinematics/test/test_control_puntos.py      (desde ~/ros2_ws)
"""
import csv
import math
import os

import numpy as np
import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import control_puntos as CP
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _robot_placa() -> R.Robot:
    """El robot con los topes que le pone control_placa.launch.py a control_nodo."""
    with open(os.path.join(PAQUETE, 'config', 'scara_urdf3.yaml')) as f:
        rp = yaml.safe_load(f)['/**']['ros__parameters']
    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    return PV.robot_con_limites(robot, PV.limites_junta(_cal_placa(), robot, 0.5))


def _cal_placa() -> A.Calibracion:
    with open(os.path.join(PAQUETE, 'config', 'micropython.yaml')) as f:
        mp = yaml.safe_load(f)['/**']['ros__parameters']
    return A.Calibracion.desde_parametros(lambda n, d: mp.get(n, d))


@pytest.mark.parametrize('texto', ['[40, -10, 12]', '40 -10 12', '40,-10,12', ' (40; -10; 12) '])
def test_lee_el_punto_en_cm_y_lo_da_en_mm(texto):
    assert CP.leer_punto(texto) == pytest.approx([400.0, -100.0, 120.0])


@pytest.mark.parametrize('texto', ['salir', 'SALIR', 'q', ' exit '])
def test_salir(texto):
    assert CP.leer_punto(texto) is None


@pytest.mark.parametrize('texto, dice', [('40 -10', 'tres numeros'), ('a b c', 'no son numeros'),
                                         ('', 'tres numeros')])
def test_un_punto_mal_escrito_se_explica(texto, dice):
    with pytest.raises(CP.PuntoMalo, match=dice):
        CP.leer_punto(texto)


def test_la_orden_de_control_nodo():
    assert CP.orden_ir([380.0, -220.0, 120.0]) == 'ir 380.0 -220.0 120.0'
    assert CP.orden_ir([380.0, -220.0, 120.0], 20.0) == 'ir 380.0 -220.0 120.0 v=20'


def test_los_puntos_de_la_fase_1_y_sus_rectas_caben_en_el_brazo():
    """Con los topes de los motores: los puntos, y cada recta entre ellos en la
    misma rama del codo (control_nodo no cambia de rama a mitad). Y de uno a
    otro ninguna junta gira mas de 20 grados (Sergio, 21/10)."""
    robot = _robot_placa()
    cal = _cal_placa()
    pts = np.array(CP.PUNTOS_CM).reshape(-1, 3) * 10.0
    for a, b in zip(pts[:-1], pts[1:]):
        for s in np.linspace(0.0, 1.0, 50):
            R.ik_scara_rrp(a + (b - a) * s, robot, 'down')       # levanta si no cabe
        ma, mb = (np.array(cal.a_motor(R.ik_scara_rrp(p, robot, 'down'))) for p in (a, b))
        assert np.all(np.abs(mb - ma)[:2] <= 20.0), (a, b)
    assert np.allclose(pts[0], pts[-1])                          # cierra la vuelta


def test_lo_que_no_llega_se_dice_antes_de_mandarlo():
    robot = _robot_placa()
    assert CP.motivo_no_llega([50.0, -350.0, 190.0], robot) is None
    assert 'alcance' in CP.motivo_no_llega([700.0, 0.0, 100.0], robot)
    # Ahi el CAD llega (hombro a -114 grados), pero Alvin solo va 60 mas alla del home
    assert CP.motivo_no_llega([-250.0, -100.0, 150.0], R.ROBOT) is None
    assert 'theta1' in CP.motivo_no_llega([-250.0, -100.0, 150.0], robot)
    # Teodoro: de 0 a -135 mm, la z de 85.5 a 220.5 (con el margen, medio mm menos)
    assert CP.motivo_no_llega([200.0, -250.0, 230.0], robot) is not None


def test_las_graficas_y_el_csv(tmp_path):
    reg = CP.Registro()
    for k in range(200):
        t = 0.02 * k
        if k == 50:
            reg.marcas.append((t, 'P1'))
        if k == 120:
            reg.marcas += [(t, 'fase 2'), (t, 'T1')]
        p = np.array([380.0 - t, -220.0, 120.0])
        reg.poner('punta_ref', p, t)
        reg.poner('punta_real', p + 0.5, t)
        reg.poner('error', [-0.5, -0.5, -0.5], t)
        reg.poner('qd', [5.0, -5.0, 2.0], t)
        reg.poner('q', [math.radians(-15.0), math.radians(-30.0), 0.05], t)
    png = str(tmp_path / 'g.png')
    fig = CP.graficas(reg, [0.2, 0.2, 8.0], png, ventana=False)
    assert os.path.getsize(png) > 10_000
    titulos = [ax.get_title(loc='left') for ax in fig.axes]
    for n in ('1.', '2.', '3.', '4.'):
        assert any(t.startswith(n) for t in titulos)
    reg.guardar_csv(str(tmp_path / 'g.csv'))
    with open(tmp_path / 'g.csv') as f:
        filas = list(csv.reader(f))
    assert filas[0] == ['t_s', 'campo', 'a', 'b', 'c']
    assert sum(f[1] == 'q' for f in filas) == 200
    assert ['2.400', 'marca', 'fase 2', '', ''] in filas


def test_sin_apuntar_no_se_guarda_nada():
    reg = CP.Registro()
    reg.apuntando = False
    reg.poner('error', [1.0, 2.0, 3.0])
    assert len(reg.serie('error')[0]) == 0
