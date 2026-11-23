"""El lazo de main_control.py contra motores como los del brazo (banco_lazo.py).

24/10: parado, Alvin temblaba +-1.3 grados unas 7 veces por segundo, con el PWM
de +700 a -700 (arranca con 700 y girando roza mucho menos: se embalaba, se
pasaba la banda y volvia). El banco tiene un motor asi; con el lazo de antes
(sin freno, sin corte y sin pulsos) tiembla, y con el de ahora no.

    pytest src/scara_kinematics/test/test_banco_lazo.py     (desde ~/ros2_ws)
"""
import pytest

from scara_kinematics import banco_lazo as B

# El lazo de antes del 24/10, con las mismas ganancias de siempre: suelto en la
# banda, sin corte, sin pulsos y el mismo ARRANQUE hacia los dos lados
ANTES = {
    'Alvin': {'freno': 0, 'anticipo': 0.0, 'pulso': 0, 'arranque_menos': 700},
    'Simon': {'freno': 0, 'anticipo': 0.0, 'pulso': 0, 'arranque_menos': 450},
}


@pytest.fixture(scope='module')
def firmware():
    return B.cargar_firmware()


def test_el_banco_tiembla_como_el_brazo_con_el_lazo_de_antes(firmware):
    """Si el banco no reprodujera el temblor, que no tiemble ahora no diria nada."""
    s = B.ensayo_sujetar('Alvin', firmware, ganancias=ANTES['Alvin'])
    assert s['inversiones'] >= 10          # en 2 s: unas 7 idas y vueltas por segundo
    assert s['cambios_pwm'] >= 10
    assert s['pico'] > 0.9                 # grados


@pytest.mark.parametrize('nombre', ['Alvin', 'Simon'])
def test_parado_no_tiembla(firmware, nombre):
    for etiqueta, fis in B.variantes(nombre):
        s = B.ensayo_sujetar(nombre, firmware, fis)
        assert s['fallo'] is None, (etiqueta, s['fallo'])
        assert s['inversiones'] == 0, (etiqueta, s)
        assert s['cambios_pwm'] == 0, (etiqueta, s)
        # se queda donde ya no empuja: a menos de SALIDA_BANDA bandas (0.6 grados)
        assert s['error'] < 0.61, (etiqueta, s)


@pytest.mark.parametrize('nombre', ['Alvin', 'Simon'])
def test_despues_de_cada_paso_se_queda_quieto_y_cerca(firmware, nombre):
    for etiqueta, fis in B.variantes(nombre):
        e = B.ensayo_escalones(nombre, firmware, fis)
        assert e['fallo'] is None, (etiqueta, e['fallo'])
        assert e['inversiones'] == 0, (etiqueta, e)
        assert e['falta'] < 0.61, (etiqueta, e)


@pytest.mark.parametrize('nombre', ['Alvin', 'Simon'])
def test_despacio_no_va_y_viene(firmware, nombre):
    """Como control_nodo (3 grados/s): si se adelanta a la consigna, la espera
    frenado en vez de volver con el suelo de ARRANQUE."""
    for etiqueta, fis in B.variantes(nombre):
        if etiqueta == 'retraso0.02':
            continue                      # 20 ms de retraso: algun cambio suelto
        d = B.ensayo_despacio(nombre, firmware, fis)
        assert d['fallo'] is None, (etiqueta, d['fallo'])
        assert d['cambios_pwm'] == 0, (etiqueta, d)
        assert d['error_max'] < 1.0, (etiqueta, d)


def test_al_cambiar_de_sentido_la_rampa_sube_aunque_aun_vaya_hacia_el_otro(firmware):
    """Al cambiar de sentido aun se iba un poco hacia el otro lado por la inercia,
    y la rampa del suelo lo tomaba por «se ha movido»: el PWM se quedaba en
    ARRANQUE y a los 0.25 s saltaba el vigilante de atasco (en el banco, con un
    Alvin que pide un 10 % mas para arrancar, con el lazo de antes)."""
    fis = dict(B.FISICA['Alvin'], arranque=(0.726, 0.726))
    s = B.ensayo_sujetar('Alvin', firmware, fis, ganancias=ANTES['Alvin'])
    assert s['fallo'] is None, s['fallo']


def test_atascado_a_pulsos_salta_el_vigilante(firmware):
    """Cerca y a pulsos no se cuenta el tiempo empujando como en SEG_ATASCO: se
    cuentan los pulsos que no lo mueven. Con el motor bloqueado, salta."""
    fis = dict(B.FISICA['Alvin'], arranque=(5.0, 5.0))          # no arranca con nada
    b = B.Banco('Alvin', firmware, fis, pos=-20.0)
    b.m.ref += 1.0 * b.m.f
    b.correr(3.0)
    assert b.fallo is not None
    assert 'pulsos' in b.fallo[1]
    x = b.desde(0.0)[:, 1]
    assert x.max() - x.min() < 0.01


def test_frena_en_corto_y_soltado_va_suelto(firmware):
    """Activo y sin empujar: IN1 = IN2 = 0 con ENA = FRENO (el freno «por abajo»).
    Soltado: ENA a 0, para poder moverlo a mano."""
    b = B.Banco('Alvin', firmware, pos=-20.0)
    b.correr(0.2)
    m = b.m
    assert (m.IN1.value(), m.IN2.value(), m.ENA.duty()) == (0, 0, 1023)
    m.soltar()
    assert m.ENA.duty() == 0


def test_teodoro_como_estaba(firmware):
    """Teodoro se mira al final (24/10): ni freno, ni corte, ni pulsos, y el mismo
    ARRANQUE hacia los dos lados."""
    kp, ki, kv, arr, arr_menos, tope, banda, freno, anticipo, pulso = \
        firmware['GANANCIA_DE']['Teodoro']
    assert (ki, arr, arr_menos, tope, banda, freno, anticipo, pulso) == \
        (0.0, 450, 450, 920, 1.0, 0, 0.0, 0)
