"""Comprueba el modelo cinematico contra la cadena real del URDF.

Es lo que sostiene a cc: si model.fk() no reprodujera la cadena del URDF, la
punta que publica el nodo estaria mal y nadie se enteraria, porque no hay con
que compararla en el brazo de verdad.

    colcon test --packages-select scara_kinematics
    pytest src/scara_kinematics/test          (desde ~/ros2_ws)
"""
import math
import os
import random

import pytest
import yaml

from scara_kinematics.model import IKError, ScaraModel
from scara_kinematics.urdf_fk import UrdfChain

HERE = os.path.dirname(__file__)
SRC = os.path.dirname(os.path.dirname(HERE))          # .../ros2_ws/src

# El brazo montado: 3 GDL, sin muneca. 'manita' se queda en su cero.
YAML = 'scara_urdf3.yaml'
URDF = 'scara_urdf/urdf/scara_urdf_gz.urdf'
TIP = 'Ian'
TOOL = (0.118, 0.0, 0.0)          # la punta de la pinza en el sistema de Ian


@pytest.fixture(scope='module')
def m():
    cfg = os.path.join(os.path.dirname(HERE), 'config', YAML)
    with open(cfg) as f:
        return ScaraModel.from_params(yaml.safe_load(f)['/**']['ros__parameters'])


@pytest.fixture(scope='module')
def chain():
    path = os.path.join(SRC, URDF)
    if not os.path.exists(path):
        pytest.skip(f'no encuentro {path}')
    return UrdfChain(path)


def test_fk_coincide_con_urdf(m, chain):
    """El modelo cerrado debe reproducir la cadena completa a menos de 0.5 mm."""
    rnd = random.Random(1)
    peor = 0.0
    for _ in range(300):
        q = [rnd.uniform(lo, hi) for lo, hi in m.limits()]
        f = chain.frames(dict(zip(m.joint_names, q)))[TIP]
        ref = [f[i][3] + sum(f[i][k] * TOOL[k] for k in range(3)) for i in range(3)]
        peor = max(peor, math.dist(ref, m.fk(q)[:3]))
    assert peor < 5e-4, f'error maximo {peor * 1000:.3f} mm'


def test_ik_invierte_la_fk(m):
    """FK(IK(p)) == p para puntos generados desde configuraciones validas."""
    rnd = random.Random(2)
    n = 0
    for _ in range(500):
        q = [rnd.uniform(lo, hi) for lo, hi in m.limits()]
        x, y, z, yaw = m.fk(q)
        try:
            sols = m.ik_all(x, y, z, yaw)
        except IKError:
            continue
        assert sols, 'la IK devolvio una lista vacia'
        for s in sols:
            xs, ys, zs, _ = m.fk(s)
            assert math.dist((xs, ys, zs), (x, y, z)) < 1e-9
        n += 1
    assert n > 400, f'demasiados puntos descartados ({n}/500)'


def test_dos_ramas_de_codo(m):
    """Un punto interior debe tener codo+ y codo-, y ser distintos."""
    r = 0.5 * (m.r_min + m.r_max) if m.r_min > 1e-3 else 0.5 * m.r_max
    z = 0.5 * (m.z_min + m.z_max)
    sols = m.ik_all(r, 0.0, z, 0.0)
    assert len(sols) == 2
    assert abs(sols[0][1] - sols[1][1]) > 1e-3


def test_fuera_de_alcance(m):
    z = 0.5 * (m.z_min + m.z_max)
    with pytest.raises(IKError):
        m.ik_all(m.r_max + 0.05, 0.0, z)
    with pytest.raises(IKError):
        m.ik_all(0.5 * (m.r_min + m.r_max), 0.0, m.z_max + 0.1)


def test_jacobiano_contra_diferencias_finitas(m):
    rnd = random.Random(3)
    h = 1e-6
    for _ in range(20):
        q = [rnd.uniform(lo + 0.1, hi - 0.1) for lo, hi in m.limits()]
        j = m.jacobian(q)
        for c in range(len(q)):
            qp = list(q)
            qp[c] += h
            f0, f1 = m.fk(q), m.fk(qp)
            for fila in range(3):
                num = (f1[fila] - f0[fila]) / h
                assert abs(num - j[fila][c]) < 1e-4, (fila, c, num, j[fila][c])


def test_singularidad_con_brazo_estirado(m):
    """Con el brazo estirado del todo, manipulability() ~ 0: es lo que avisa cc."""
    q = [0.0, m.b1 - m.b2, m.d3_min]
    assert m.manipulability(q) < 1e-9
    x, y, _, _ = m.fk(q)
    assert abs(math.hypot(x, y) - m.r_max) < 1e-9


def test_reach_respeta_los_limites(m):
    """El anillo de reach() debe ser resoluble en todo su borde."""
    r_lo, r_hi = m.reach()
    assert r_lo >= m.r_min - 1e-9 and r_hi <= m.r_max + 1e-9
    z = 0.5 * (m.z_min + m.z_max)
    for r in (r_lo + 1e-4, 0.5 * (r_lo + r_hi), r_hi - 1e-4):
        m.ik(r, 0.0, z, 0.0)          # no debe levantar IKError
