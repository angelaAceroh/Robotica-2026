"""traycho: los puntos del codigo de Sergio en su brazo, y el que manda los ir
uno detras de otro (sin ROS).

    pytest src/scara_kinematics/test/test_traycho.py      (desde ~/ros2_ws)
"""
import os

import numpy as np
import pytest
import yaml

from scara_kinematics import alcance as A
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R
from scara_kinematics import traycho as T

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOME_DENTRO = (7.4, -280.8, 210.0)    # HOME (0, -270) con Simon 3 grados hacia dentro


def _robot_placa() -> R.Robot:
    """El robot con los topes que le pone control_placa.launch.py a control_nodo."""
    with open(os.path.join(PAQUETE, 'config', 'scara_urdf3.yaml')) as f:
        rp = yaml.safe_load(f)['/**']['ros__parameters']
    with open(os.path.join(PAQUETE, 'config', 'micropython.yaml')) as f:
        mp = yaml.safe_load(f)['/**']['ros__parameters']
    robot = R.Robot(rp['l1'] * 1e3, rp['b1'], rp['l2'] * 1e3, rp['b2'], rp['z_ref'] * 1e3,
                    rp['z_dir'], ((rp['q1_min'], rp['q1_max']), (rp['q2_min'], rp['q2_max']),
                                  (rp['d3_min'] * 1e3, rp['d3_max'] * 1e3)))
    cal = A.Calibracion.desde_parametros(lambda n, d: mp.get(n, d))
    return PV.robot_con_limites(robot, PV.limites_junta(cal, robot, 0.5))


def test_son_los_puntos_de_su_codigo_desplazados_sin_deformar():
    p = T.puntos_mm()
    suyos = np.array(T.PUNTOS_CM) * 10.0
    assert p.shape == (8, 3)
    # en planta, la misma figura
    assert np.allclose(p[1:, :2] - p[:-1, :2], suyos[1:, :2] - suyos[:-1, :2])
    assert np.allclose(p[0], p[-1])                                # vuelve a P1
    # las z, a ESCALA_Z (1: las suyas, 29/10), con P1 arriba: Teodoro baja 115 mm
    assert np.allclose(p[1:, 2] - p[:-1, 2], (suyos[1:, 2] - suyos[:-1, 2]) * T.ESCALA_Z)
    assert p[0, 2] == 220.0 and p[:, 2].min() == pytest.approx(105.0)
    assert _robot_placa().lim[2][1] + R.ROBOT.z_ref >= 220.0       # P1 cabe arriba del todo


def test_caben_y_ningun_tramo_gira_mas_de_20_grados():
    """Desde HOME (un poco dentro), cada recta en la rama down y lejos del codo
    recto, y de un ir al siguiente ninguna junta gira mas de 20 grados."""
    robot = _robot_placa()
    lo2, hi2 = A.lado_down(robot, *robot.lim[1])
    tramos = T.tramos(T.puntos_mm(), robot, desde=HOME_DENTRO)
    a = np.array(HOME_DENTRO)
    for nombre, b in tramos:
        for s in np.linspace(0.0, 1.0, 30):
            q = R.ik_scara_rrp(a + (b - a) * s, robot, 'down')
            assert lo2 <= q[1] <= hi2, nombre
        qa, qb = (np.degrees(R.ik_scara_rrp(p, robot, 'down')[:2]) for p in (a, b))
        assert np.all(np.abs(qb - qa) <= 20.0 + 1e-9), nombre
        a = b
    # cada punto de su codigo es el final de un tramo, en orden
    llegadas = [p for n, p in tramos if '(' not in n or n.split('(')[1] in
                (f'{k}/{k})' for k in range(1, 20))]
    assert np.allclose(llegadas, T.puntos_mm())


def test_desde_el_tope_de_simon_no_cabe_y_lo_dice():
    with pytest.raises(R.RutaError, match='theta2'):
        T.tramos(T.puntos_mm(), _robot_placa(), desde=(0.0, -270.0, 200.0))


class _Control:
    """Lo que hace la ventana: guarda lo mandado y lo dicho."""

    def __init__(self, escucha=True):
        self.mandado, self.dicho, self.escucha = [], [], escucha

    def mandar(self, texto):
        if self.escucha:
            self.mandado.append(texto)
        return self.escucha

    def decir(self, nivel, texto):
        self.dicho.append((nivel, texto))


def _recorrido(n=3, escucha=True):
    tramos = [(f'P{i}', np.array([100.0 * i, -300.0, 150.0])) for i in range(1, n + 1)]
    c = _Control(escucha)
    return T.Recorrido(tramos, c.mandar, c.decir), c


def test_manda_uno_y_espera_a_que_acabe_para_el_siguiente():
    r, c = _recorrido()
    r.paso(0.0)
    assert c.mandado == ['ir 100.0 -300.0 150.0']
    r.aviso('control', 30, 'recta: a esa velocidad alguna junta pasa de su techo')
    r.aviso('control', 20, 'ir: 4.0 s (recta 4.0 s) | rama del codo down')
    r.paso(0.1)
    r.paso(3.0)
    assert len(c.mandado) == 1 and r.fase == 'marcha'
    # el ir de 0 s: empieza y acaba en el mismo lote
    r.aviso('control', 20, 'ir terminada | punta medida frente a la deseada: medio 1.2 mm, '
                           'maximo 3.4 mm, RMS 1.5 mm')
    r.paso(4.1)
    assert c.mandado[-1] == 'ir 200.0 -300.0 150.0'
    r.aviso('control', 20, 'ir: 0.0 s (recta 0.0 s)')
    r.aviso('control', 20, 'ir terminada | medio 0.1 mm, maximo 0.2 mm')
    r.paso(4.2)
    r.aviso('control', 20, 'ir: 2.0 s')
    r.aviso('control', 20, 'ir terminada | medio 0.1 mm, maximo 0.2 mm')
    r.paso(6.3)
    assert len(c.mandado) == 3 and r.fase == 'hecho' and not r.activo()
    assert any('medio 1.2 mm, maximo 3.4 mm' in t for _, t in c.dicho)
    assert 'terminado' in c.dicho[-1][1]


@pytest.mark.parametrize('nodo, nivel, texto, dice', [
    ('control', 20, 'parado: regulo donde estoy', 'parado'),
    ('control', 40, '«ir 1 2 3» no sale: P fuera del alcance', 'no lo hace'),
])
def test_se_corta_si_control_nodo_se_para_o_no_lo_hace(nodo, nivel, texto, dice):
    r, c = _recorrido()
    r.paso(0.0)
    if 'no sale' not in texto:
        r.aviso('control', 20, 'ir: 4.0 s')
    r.aviso(nodo, nivel, texto)
    r.paso(1.0)
    r.paso(2.0)
    assert r.fase == 'cortado' and len(c.mandado) == 1
    assert dice in c.dicho[-1][1] and c.dicho[-1][0] == 40


def test_se_corta_si_la_placa_suelta_los_motores_y_dice_por_que():
    r, c = _recorrido()
    r.paso(0.0)
    r.aviso('control', 20, 'ir: 4.0 s')
    r.aviso('puente_placa', 40, 'placa: Teodoro 0.25 s empujando (de PWM 450 a 920) y no se '
                                'ha movido ni 0.012 mm')
    r.aviso('puente_placa', 30, 'placa: PARADO -2.88 -10.10 -23.36  (motores sueltos; '
                                'para seguir, «activar»)')
    r.paso(1.0)
    assert r.fase == 'cortado'
    assert 'soltado los motores' in r.porque and 'Teodoro' in r.porque


def test_se_corta_si_control_nodo_no_contesta_o_no_acaba():
    r, c = _recorrido()
    r.paso(0.0)
    r.paso(T.ESPERA_EMPIEZA_S + 0.1)
    assert r.fase == 'cortado' and 'no contesta' in r.porque
    r, c = _recorrido()
    r.paso(0.0)
    r.aviso('control', 20, 'ir: 4.0 s')
    r.paso(1.0)
    r.paso(1.0 + 2 * 4.0 + 15.0 + 0.1)                 # 29/10: con 2*dura + 15, aun no
    assert r.fase == 'marcha'
    r.paso(1.0 + T.limite_marcha(4.0) + 0.1)
    assert r.fase == 'cortado' and 'no acaba en 42 s' in r.porque


def test_sin_control_nodo_no_empieza():
    r, c = _recorrido(escucha=False)
    r.paso(0.0)
    assert r.fase == 'cortado' and 'no escucha' in r.porque


def test_cortar_dos_veces_lo_dice_una():
    r, c = _recorrido()
    r.paso(0.0)
    r.cortar('has mandado «parar»')
    r.cortar('otra')
    assert sum(1 for n, _ in c.dicho if n == 40) == 1
