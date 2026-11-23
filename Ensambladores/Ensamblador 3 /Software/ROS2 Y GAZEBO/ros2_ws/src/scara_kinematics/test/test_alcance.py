"""Comprueba alcance.py: de (x, y, z) a los grados de los motores, y si caben.

    pytest src/scara_kinematics/test/test_alcance.py          (desde ~/ros2_ws)
"""
import math
import os
import random

import numpy as np
import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import ruta as R

HERE = os.path.dirname(__file__)


def _yaml():
    with open(os.path.join(os.path.dirname(HERE), 'config', 'micropython.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def test_el_yaml_trae_los_recorridos_de_los_programas_de_sergio():
    cal = A.Calibracion.desde_parametros(lambda n, d: _yaml().get(n, d))
    # los que dio Sergio con el brazo delante (21/10)
    assert cal.limites == [(-60.0, 60.0), (-280.0, 0.0), (-135.0, 0.0)]
    assert cal.paso_husillo_mm == 360.0       # la placa ya cuenta a Teodoro en mm
    assert cal == A.Calibracion()             # los defectos del codigo son los del YAML


@pytest.mark.parametrize('signo', [[1.0, 1.0, 1.0], [-1.0, 1.0, -1.0]])
def test_junta_motor_ida_y_vuelta(signo):
    cal = A.Calibracion(signo=signo, reduccion=[2.0, 0.5], cero=[3.0, -7.0, 20.0],
                        paso_husillo_mm=8.0)
    q = [0.3, -0.7, 61.0]
    assert np.allclose(cal.a_junta(cal.a_motor(q)), q)


def test_dentro_de_los_limites_siempre_llega():
    """Motores al azar dentro de su recorrido -> FK -> alcanzar devuelve esos motores."""
    cal = A.Calibracion()
    rnd = random.Random(7)
    for _ in range(200):
        # Simon: desde el home (codo doblado 120 grados, Simon en 0) hasta 10
        # grados antes de recto (Simon en -120): la rama down, la del brazo en 0
        m = [rnd.uniform(-59.5, 59.5), rnd.uniform(-109.5, -0.5)]
        t1, t2, _ = cal.a_junta(m + [None])
        P = R.fk_scara_rrp([t1, t2, 60.0], R.ROBOT)
        v = A.alcanzar(P, cal)
        assert v.alcanzable, v.resumen()
        assert np.allclose(v.motor[:2], m, atol=1e-6)
        assert np.allclose(R.fk_scara_rrp(v.junta, R.ROBOT), P, atol=1e-6)


def test_los_puntos_de_las_rutas_caben_con_estos_limites():
    """Con los recorridos del 21/10 y el home en (0, -270), si."""
    cal = A.Calibracion()
    for puntos in R.PUNTOS.values():
        for P in puntos.values():
            assert A.alcanzar(P, cal).alcanzable, P


def test_fuera_del_anillo_lo_dice_una_vez():
    v = A.alcanzar((600.0, 0.0, 150.0), A.Calibracion())
    assert not v.alcanzable and len(v.motivos) == 1 and 'alcance' in v.motivos[0]


def test_sin_paso_la_z_no_se_comprueba():
    sin_paso = A.Calibracion(paso_husillo_mm=0.0)
    v = A.alcanzar((470.0, -60.0, 125.0), sin_paso)
    assert v.alcanzable and not v.z_comprobada and v.motor[2] is None
    assert A.rango_z(sin_paso) is None


def test_teodoro_en_mm():
    """Teodoro va de 0 (arriba, d3 = 155 mm) a -135 mm (baja, d3 = 20): la z de
    la punta va de z_ref + 20 a z_ref + 155."""
    cal = A.Calibracion()
    assert A.rango_z(cal) == pytest.approx((R.ROBOT.z_ref + 20.0, R.ROBOT.z_ref + 155.0))
    v = A.alcanzar((470.0, -60.0, R.ROBOT.z_ref + 70.0), cal)
    assert v.alcanzable and v.z_comprobada and v.motor[2] == pytest.approx(-85.0)
    v = A.alcanzar((470.0, -60.0, R.ROBOT.z_ref + 160.0), cal)
    assert not v.alcanzable and any('Teodoro' in m and 'mm' in m for m in v.motivos)


def test_con_paso_teodoro_tambien_limita():
    cal = A.Calibracion(paso_husillo_mm=8.0, cero=[0.0, 0.0, 60.0])
    lo, hi = A.rango_z(cal)
    z_medio = (lo + hi) / 2
    assert A.alcanzar((470.0, -60.0, z_medio), cal).alcanzable
    v = A.alcanzar((470.0, -60.0, hi + 5.0), cal)
    assert not v.alcanzable and any('Teodoro' in m for m in v.motivos)


def test_contorno_cerrado_y_dentro_del_brazo():
    c = A.contorno_planta(A.Calibracion())
    r = np.hypot(c[:, 0], c[:, 1])
    assert np.allclose(c[0], c[-1])
    assert r.max() <= R.ROBOT.a1 + R.ROBOT.a2 + 1e-6
    # Simon en 0 (el home, desde el 23/10 en (0, -270)): no se acerca mas a la base
    assert math.isclose(r.min(), 270.0, rel_tol=0.01)
    # el home de Sergio es un punto del borde (Alvin y Simon en 0)
    assert np.min(np.hypot(c[:, 0] - 0.0, c[:, 1] + 270.0)) < 15.0
    # Simon cruza el codo recto: solo la rama down, hasta 10 grados antes de recto
    assert math.isclose(r.max(), 2 * 250.0 * math.cos(math.radians(10.0) / 2), rel_tol=0.01)


def test_el_vector_lleva_las_velocidades_fijas():
    """[pos_alvin, 14, pos_simon, 30, pos_teodoro, 60]; sin paso, Teodoro se queda."""
    cal = A.Calibracion()
    v = A.alcanzar((430.0, -160.0, 125.0), cal)
    vec = A.vector_placa(v.motor, [0.0, 0.0, -12.5], cal)
    assert vec[1::2] == [14.0, 30.0, 60.0]
    assert vec[0] == pytest.approx(v.motor[0], abs=1e-3)
    assert vec[2] == pytest.approx(v.motor[1], abs=1e-3)
    assert vec[4] == pytest.approx(v.motor[2], abs=1e-3)     # mm
    sin_paso = A.Calibracion(paso_husillo_mm=0.0)
    v = A.alcanzar((430.0, -160.0, 125.0), sin_paso)
    assert A.vector_placa(v.motor, [0.0, 0.0, -12.5], sin_paso)[4] == -12.5


def test_cuanto_tarda_cada_motor():
    """RPM de junta (1 RPM = 6 grados/s) y mm/min de Teodoro."""
    assert A.segundos(84.0, 14.0, 0) == pytest.approx(1.0)
    assert A.segundos(-60.0, 10.0, 1) == pytest.approx(1.0)
    assert A.segundos(2.0, 60.0, 2) == pytest.approx(2.0)
    assert A.segundos(5.0, 0.0, 2) == 0.0
