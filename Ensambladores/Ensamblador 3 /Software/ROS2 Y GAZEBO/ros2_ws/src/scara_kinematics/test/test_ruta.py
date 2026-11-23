"""Comprueba ruta.py: que es el robot de la mesa y que sus rutas caben en el.

ruta.py escribe la cinematica a la manera de la guia (en mm, con numpy), aparte
de model.py. Estos tests son lo que garantiza que las dos no se separan.

    pytest src/scara_kinematics/test/test_ruta.py          (desde ~/ros2_ws)
"""
import math
import os
import random

import numpy as np
import pytest
import yaml

from scara_kinematics import ruta as R
from scara_kinematics.model import ScaraModel

HERE = os.path.dirname(__file__)


@pytest.fixture(scope='module')
def m():
    cfg = os.path.join(os.path.dirname(HERE), 'config', 'scara_urdf3.yaml')
    with open(cfg) as f:
        return ScaraModel.from_params(yaml.safe_load(f)['/**']['ros__parameters'])


def test_las_constantes_son_las_del_yaml(m):
    """Las medidas copiadas en ruta.py son las de config/scara_urdf3.yaml."""
    assert R.ROBOT == R.Robot.desde_modelo(m)


def test_fk_y_jacobiano_como_model(m):
    rnd = random.Random(4)
    for _ in range(300):
        q = [rnd.uniform(lo, hi) for lo, hi in m.limits()]
        qmm = [q[0], q[1], q[2] * 1e3]
        assert np.allclose(R.fk_scara_rrp(qmm), np.array(m.fk(q)[:3]) * 1e3, atol=1e-9)
        jm = np.array(m.jacobian(q))[:3, :3]
        jm[:2, :2] *= 1e3                    # m/rad -> mm/rad; la fila de z es adimensional
        assert np.allclose(R.jacobiano_scara_rrp(qmm), jm, atol=1e-9)


@pytest.mark.parametrize('codo, i', [('up', 0), ('down', -1)])
def test_ik_como_model(m, codo, i):
    """Misma rama y mismos angulos que model.ik_all ('up' primero)."""
    rnd = random.Random(5)
    n = 0
    for _ in range(400):
        q = [rnd.uniform(lo, hi) for lo, hi in m.limits()]
        x, y, z, _ = m.fk(q)
        esperada = m.ik_all(x, y, z)[i]
        if not m.in_limits(esperada):
            with pytest.raises(R.RutaError):
                R.ik_scara_rrp([x * 1e3, y * 1e3, z * 1e3], codo=codo)
            continue
        qi = R.ik_scara_rrp([x * 1e3, y * 1e3, z * 1e3], codo=codo)
        assert np.allclose(qi, [esperada[0], esperada[1], esperada[2] * 1e3], atol=1e-7)
        n += 1
    assert n > 200


@pytest.mark.parametrize('nombre', list(R.RUTAS))
@pytest.mark.parametrize('perfil', R.PERFILES)
def test_las_rutas_caben_en_el_brazo(nombre, perfil):
    """Cada ruta, con cada perfil: dentro de limites, sin saltos, y llega."""
    plan = R.planificar(R.RUTAS[nombre], nombre_perfil=perfil)
    assert all(R.ROBOT.fuera(q) is None for q in plan.q)
    assert np.all(np.diff(plan.t) > 0)
    assert np.max(np.abs(np.diff(plan.q[:, :2], axis=0))) < 0.01   # sin vueltas de 2*pi
    assert np.max(np.abs(plan.qd), axis=0)[2] <= R.QDOT_MAX[2] + 1e-9
    assert plan.err.max() < 0.02                                 # mm
    for k in range(plan.ruta.shape[1]):                          # pasa por cada punto
        assert np.min(np.linalg.norm(plan.p - plan.ruta[:, k], axis=1)) < R.TOL_POS
    assert np.linalg.norm(plan.p[-1] - plan.ruta[:, -1]) < R.TOL_POS


def test_perfil_lineal_es_la_ley_de_la_guia():
    """Con 'lineal' cada tramo dura distancia/velocidad, como en la guia."""
    plan = R.planificar(R.RUTAS['perforado'], nombre_perfil='lineal')
    for tr in plan.tramos:
        assert tr.duracion == pytest.approx(tr.distancia / tr.velocidad)


def test_velocidad_por_tipo_de_tramo():
    plan = R.planificar(R.RUTAS['contorno'])
    tipos = [tr.tipo for tr in plan.tramos]
    assert tipos == ['desplazamiento', 'bajada', 'trazado', 'trazado', 'trazado',
                     'trazado', 'subida', 'desplazamiento']
    for tr in plan.tramos:
        v = np.linalg.norm(plan.p[1:] - plan.p[:-1], axis=1) / np.diff(plan.t)
        en = plan.tramo[1:] == tr.n - 1
        assert v[en].max() == pytest.approx(tr.velocidad, rel=0.02)


def test_fuera_de_alcance_se_sabe_antes():
    ruta = np.column_stack([R.HOME, [480.0, 200.0, R.Z_SEGURO]])
    with pytest.raises(R.RutaError):
        R.planificar(ruta)


def test_limite_de_junta_a_mitad_de_camino():
    """Los dos extremos valen, pero la recta entre ellos saca al hombro de +-120."""
    a = [69.46, 393.92, R.Z_SEGURO]          # r = 400 a +80 deg: theta1 = 104 deg
    b = [69.46, -393.92, R.Z_SEGURO]         # r = 400 a -80 deg: theta1 = -56 deg
    R.ik_scara_rrp(a)
    R.ik_scara_rrp(b)
    with pytest.raises(R.RutaError, match='theta1'):
        R.planificar(np.column_stack([a, b]))


def test_en_interpola_y_satura():
    plan = R.planificar(R.RUTAS['perforado'])
    q, qd, pd, i = plan.en(-1.0)
    assert np.allclose(q, plan.q[0]) and i in (-1, 0)
    q, qd, pd, i = plan.en(plan.duracion + 5.0)
    assert np.allclose(q, plan.q[-1]) and i == len(plan.tramos) - 1
    tm = 0.5 * (plan.t[100] + plan.t[101])
    q, _, _, _ = plan.en(tm)
    assert np.allclose(q, 0.5 * (plan.q[100] + plan.q[101]))


def test_a_ros_pasa_d3_a_metros():
    assert R.a_ros([0.1, -0.2, 84.49]) == pytest.approx([0.1, -0.2, 0.08449])
    assert math.isclose(R.fk_scara_rrp([0, 0, 0])[2], R.Z_REF)
