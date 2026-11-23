"""Llegar mas fino (24/10, «continua mejorandolo»): con la consigna quieta, la
placa ya no se conforma con 2 bandas (0.6 grados) tras un corte o un pulso que
se pasa: lo acercan los pulsos hasta la banda (0.3). Antes, en el traycho por
juntas, Alvin acababa a 0.41-0.56 grados (2-4 mm) y juntas_nodo avisaba
«FUERA de la tolerancia» (0.4).

Sin que vuelva el temblor, y Teodoro (sin pulsos) como estaba."""
import os

import numpy as np
import pytest

from scara_kinematics import banco_lazo as B

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PASOS = [0.5, -1.0, 2.0, -5.0, 10.0, -0.7, 3.0, -15.0, 18.0, -18.0, 7.0, -2.5]


@pytest.fixture(scope='module')
def firmware():
    return B.cargar_firmware(os.path.join(PAQUETE, 'firmware', 'micropython', 'main_control.py'))


def _pasos(motor, g):
    """Lo que le falta al final de cada paso (2 s quieto), y cuanto tiembla."""
    faltas, inv, fallos = [], 0, []
    for etiqueta, fis in B.variantes(motor):
        b = B.Banco(motor, g, fis, pos=-20.0)
        for d in PASOS:
            obj = b.mover(d, 84.0 if motor == 'Alvin' else 30.0, sujetar=2.0)
            faltas.append(abs(obj - b.m.medida()))
            inv += B.inversiones(b.desde(b.t - 1.0)[:, 1], 0.2)
            if b.fallo:
                fallos.append((etiqueta, b.fallo))
                break
    return np.array(faltas), inv, fallos


@pytest.mark.parametrize('motor', ['Alvin', 'Simon'])
def test_llega_dentro_de_la_tolerancia_de_juntas(motor, firmware):
    """12 pasos con cada una de las 11 variantes: casi todos acaban a menos de
    0.4 (TOL de juntas_nodo). Con el firmware de antes, Alvin pasaba de 0.4 en
    37 de 132 y Simon en 21; ahora, 5 y 3."""
    f, inv, fallos = _pasos(motor, firmware)
    assert not fallos
    assert inv == 0
    assert f.mean() < 0.2
    assert np.sum(f > 0.4) <= 0.06 * len(f)
    assert f.max() < 0.6                       # y nunca peor que antes (2 bandas)


def test_teodoro_sin_pulsos_asienta_como_siempre(firmware):
    """Teodoro (PULSO 0) se da por asentado al entrar en la banda aunque vaya
    deprisa, como antes: lo de entrar casi parado es solo con pulsos."""
    b = B.Banco('Teodoro', firmware, pos=-20.0)
    m = b.m
    m.ref = m.posicion() + 0.5 * m.banda           # dentro de la banda
    m.v_med = 50.0                                 # y deprisa
    m.paso(10)
    assert m.asentado
    a = B.Banco('Alvin', firmware, pos=-20.0)
    m = a.m
    m.ref = m.posicion() + 0.5 * m.banda
    m.v_med = 50.0
    m.paso(10)
    assert not m.asentado                          # Alvin, no: que lo acerquen los pulsos
