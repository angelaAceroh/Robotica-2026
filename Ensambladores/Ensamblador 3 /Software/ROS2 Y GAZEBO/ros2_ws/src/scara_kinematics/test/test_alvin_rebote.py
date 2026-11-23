"""Alvin yendo a P3 (24/10): la transmision cede. El motor (donde esta el
encoder) se adelanta al brazo empujando y, sin empujar, vuelve hacia el. En el
brazo, cada pulso de 20 ms lo movia medio grado y volvia, un poco mas atras cada
vez: como «se movia», la placa ni subia el PWM ni alargaba el pulso, y se quedo
2.8 grados por detras de la consigna (P3 y P5 del traycho por juntas).

Rebote es eso, sin mas: los numeros salen de la grabacion (ir_p3, 24/10)."""
import math
import os

import numpy as np
import pytest

from scara_kinematics import banco_lazo as B

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE = os.path.join(PAQUETE, 'firmware', 'micropython', 'main_control.py')


class Rebote:
    """En grados del motor. Empujando, el motor va a `w` (por unidad de PWM) y
    el brazo no se mueve hasta que el empujon es de PWM_BRAZO o mas durante
    MS_BRAZO seguidos: entonces el brazo arranca y sigue al motor. Sin empujar,
    el motor vuelve hacia el brazo a VUELTA grados/s por grado de diferencia
    (la transmision tensa; frenado, despacio)."""

    PWM_BRAZO = 850
    MS_BRAZO = 50

    def __init__(self, theta, brazo, w=33.0, vuelta=6.0):
        self.theta = theta
        self.brazo = brazo
        self.w = w
        self.vuelta = vuelta
        self.ms_fuerte = 0.0

    def paso(self, dt, in1, in2, duty):
        if in1 != in2 and duty > 0:
            s = 1.0 if in1 else -1.0
            self.theta += s * self.w * (duty / 1023.0) * dt
            self.ms_fuerte = self.ms_fuerte + dt * 1000 if duty >= self.PWM_BRAZO else 0.0
            if self.ms_fuerte >= self.MS_BRAZO:
                self.brazo = self.theta                    # arranca y lo sigue
        else:
            self.ms_fuerte = 0.0
            d = self.brazo - self.theta
            self.theta += math.copysign(min(abs(d), abs(d) * self.vuelta * dt), d)


def _banco(firmware, motor=-30.6, brazo=-28.1, consigna=-31.0):
    """Como en la grabacion: el motor ha llegado a -30.6 con la transmision tensa
    y el brazo se ha quedado en -28.1; la consigna, quieta en -31."""
    b = B.Banco('Alvin', firmware, pos=motor, activar=False)
    b.fis = Rebote(motor * b.m.f, brazo * b.m.f)
    b._encoder()
    b.m.ref = b.m.pos_ant = b.m.posicion()
    b.m.activar()
    b.m.ref = consigna * b.m.f
    return b


def test_el_rebote_es_como_en_el_brazo():
    """Con PWM 700 de 20 ms, el motor se va medio grado y vuelve."""
    f = Rebote(-29.0, -28.1)
    for _ in range(40):                       # 20 ms a 700, hacia -
        f.paso(0.0005, 0, 1, 700)
    assert f.theta == pytest.approx(-29.45, abs=0.05)
    assert f.brazo == -28.1
    for _ in range(200):                      # 100 ms sin empujar
        f.paso(0.0005, 0, 0, 0)
    assert f.theta > -29.0


def test_con_el_firmware_de_antes_se_queda_lejos(tmp_path):
    """El de antes (pulsos que cuentan si se ha movido, no si se ha acercado, y
    siempre de 20 ms) no lleva el brazo a la consigna en 4 s."""
    viejo = tmp_path / 'main_control_antes.py'
    with open(FIRMWARE) as f:
        codigo = f.read()
    # el bloque de los pulsos tal como estaba antes del 24/10 por la tarde
    a = codigo.index('                signo = 1 if e > 0.0 else -1\n                if self.pos_pulso is None:')
    z = codigo.index('                self.ms_pulso = self.largo_pulso * min(1 + self.pulsos_quietos, '
                     'PULSO_LARGO_MAX)\n')
    z = codigo.index('\n', z) + 1
    viejo.write_text(codigo[:a] + ANTES + codigo[z:])
    b = _banco(str(viejo))
    b.correr(4.0)
    assert abs(-31.0 - b.fis.brazo) > 1.5


def test_ahora_llega():
    """Los pulsos cuentan lo que lo ACERCAN: los que no, cada vez mas fuertes y
    mas largos, hasta que el brazo arranca. Llega y se queda."""
    b = _banco(FIRMWARE)
    b.correr(4.0)
    assert b.fallo is None
    assert abs(-31.0 - b.fis.brazo / b.m.f) < 0.65
    assert abs(-31.0 - b.m.medida()) < 0.65


def test_si_no_hay_manera_deja_de_empujar_y_avisa():
    """Con un brazo que no arranca ni con TOPE y pulsos largos: antes, pulsos
    para siempre (cada uno «lo movia»). Ahora deja de empujar y lo dice. No es
    el vigilante: el motor gira y el encoder cuenta (ese, con el motor
    bloqueado: test_banco_lazo)."""
    b = _banco(FIRMWARE)
    b.fis.PWM_BRAZO = 2000
    avisos = []
    for _ in range(400):                      # 4 s
        b.correr(0.01)
        if b.m.aviso:
            avisos.append(b.m.aviso)
            b.m.aviso = None
    assert b.fallo is None
    assert b.m.rendido and len(avisos) == 1 and 'no llega' in avisos[0]
    assert (b.desde(b.t - 1.0)[:, 3] == 0).all()          # el ultimo segundo, quieto
    b.correr(0.5, -1.0)                       # otra consigna: vuelve a intentarlo
    assert not b.m.rendido


def test_en_la_frontera_de_los_pulsos_no_vuelve_a_empezar():
    """P3, 24/10 a las 12:37: el brazo atascado a unos 3 grados (PULSOS_HASTA)
    de la consigna. El error iba y venia alrededor de 3 y cada vez se salia de
    los pulsos y volvia a empezar por el mas flojo: 35 s de pulsos de 700.
    Ahora suben hasta TOPE, y con un brazo que arranca con mas de lo que tensa
    llegar a la consigna, los de TOPE se pasan PASAR_FORZANDO y lo arrancan.

    banco_elastico con transmisiones y brazos de varias durezas (SUPUESTO: no
    se sabe la del brazo). Con el firmware de antes, en el barrido entero (90
    casos): 1.09 grados de media, 58 a mas de 0.65 y 33 vigilantes; ahora
    0.72, 29 y ninguno."""
    from scara_kinematics import banco_elastico as E
    faltas = []
    for k in (0.15, 0.3):
        for arr in (0.5, 0.8):
            for hueco in (1.5, 2.5, 2.9):
                b = E.banco('Alvin', FIRMWARE, pos=-31.0 + hueco, k=k, arranque=(arr, arr))
                b.m.ref = -31.0 * b.m.f
                b.m.holgura_e = 5.0           # como yendo: aun no se ha gastado
                b.correr(5.0)
                assert b.fallo is None, (k, arr, hueco, b.fallo)
                assert abs(b.desde(b.t - 1.0)[:, 3]).max() == 0, (k, arr, hueco)   # quieto
                faltas.append(abs(-31.0 - b.fis.brazo / b.m.f))
    faltas = np.array(faltas)
    assert faltas.mean() < 0.85
    assert np.sum(faltas > 0.65) <= len(faltas) // 2


ANTES = '''                signo = 1 if e > 0.0 else -1
                if self.pos_pulso is None:
                    self.largo_pulso = self.pulso
                    self.pulsos_quietos = 0
                elif abs(pos - self.pos_pulso) < MOVIDO_MIN:
                    self.pulsos_quietos += 1
                else:
                    self.pulsos_quietos = 0
                    if signo != self.signo_pulso:          # se ha pasado
                        self.largo_pulso = max(PERIODO_MS, self.largo_pulso // 2)
                        if abs(e) < SALIDA_BANDA * self.banda:
                            self.asentado = True
                            self.frenar()
                            return
                if self.pulsos_quietos >= PULSOS_ATASCO:
                    self.fallo = ("%d pulsos (hasta PWM %d) y no se ha movido ni %.3f %s: "
                                  "atascado o sin encoder"
                                  % (PULSOS_ATASCO, self.tope, MOVIDO_MIN / self.f, self.unidad))
                    return
                self.pos_pulso = pos
                self.signo_pulso = signo
                self.ms_pulso = self.largo_pulso
'''


def test_frenado_lejos_a_pulsos_no_es_que_se_haya_ido():
    """P3, 24/10 a las 12:55: ya a pulsos, la transmision devolvio el motor
    hasta -26.97 y se quedo frenado a 3.9 grados de la consigna: el vigilante
    de «se ha ido» (3 grados, para un motor que se escapa) solto los motores y
    corto el traycho. A pulsos y con la consigna quieta, hasta PULSOS_SALE no
    es eso. Aqui el brazo no arranca ni con TOPE: acaba rendido, con AVISO y
    quieto."""
    from scara_kinematics import banco_elastico as E
    b = E.banco('Alvin', FIRMWARE, pos=-28.5, k=0.15, arranque=(0.9, 0.9))
    b.m.ref = -31.0 * b.m.f
    b.m.holgura_e = 5.0
    while b.m.pos_pulso is None:              # hasta el primer pulso
        b.correr(0.01)
    b.fis.brazo = b.fis.theta = -27.0 * b.m.f    # y el motor, de vuelta a -27 (3.9 + 0.1)
    b.fis.w = b.fis.w_brazo = 0.0
    avisos = []
    for _ in range(500):                      # 5 s
        b.correr(0.01)
        if b.m.aviso:
            avisos.append(b.m.aviso)
            b.m.aviso = None
    assert b.fallo is None, b.fallo
    assert b.m.rendido and len(avisos) == 1
    assert abs(b.desde(b.t - 1.0)[:, 3]).max() == 0


def test_mas_alla_de_pulsos_sale_si_salta():
    """Y a mas de PULSOS_SALE, aunque la consigna este quieta, el vigilante."""
    b = _banco(FIRMWARE, motor=-25.0, brazo=-25.0, consigna=-25.0)
    b.m.pos_pulso = b.m.posicion()
    b.m.ref = -31.0 * b.m.f
    b.correr(0.05)
    assert b.fallo is not None and 'se ha ido' in b.fallo[1]
