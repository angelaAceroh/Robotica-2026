"""Comprueba como la aduana lee la telemetria y como etiqueta cada muestra.

Es lo que decide que entra en la cadena: una linea rota que pasara por buena
seria un numero falso en el brazo, y una etiqueta mal leida haria que cc juntara
piezas de muestras distintas.

    pytest src/scara_kinematics/test          (desde ~/ros2_ws)
"""
from scara_kinematics.protocolo import (Telemetria, checksum, etiqueta, leer_cero,
                                        leer_etiqueta, leer_telemetria)


def firmada(cuerpo: str) -> str:
    """Como la firma el firmware: XOR de todo lo que va antes del '*'."""
    return f'{cuerpo}*{checksum(cuerpo)}'


LINEA = '> 12040 0.20071 -0.34907 0.09000 0.150 -0.120 0.000 7'


def test_linea_buena_sale_en_vectores():
    t = leer_telemetria(firmada(LINEA), 3)
    assert t == Telemetria(12040, (0.20071, -0.34907, 0.09), (0.15, -0.12, 0.0), 7)


def test_checksum_en_minusculas_tambien_vale():
    cuerpo, firma = firmada(LINEA).split('*')
    assert isinstance(leer_telemetria(f'{cuerpo}*{firma.lower()}', 3), Telemetria)


def test_un_byte_cambiado_se_tira():
    buena = firmada(LINEA)
    rota = buena.replace('0.20071', '0.20171')
    assert leer_telemetria(rota, 3) == 'checksum incorrecto'


def test_sin_checksum_se_tira():
    assert leer_telemetria(LINEA, 3) == 'sin checksum'


def test_campos_de_mas_o_de_menos():
    # Dos lineas pegadas, o un firmware de 4 juntas con un YAML de 3.
    assert 'campos' in leer_telemetria(firmada(LINEA + ' 0.5'), 3)
    assert 'campos' in leer_telemetria(firmada(LINEA), 4)


def test_nan_no_entra():
    assert leer_telemetria(firmada(LINEA.replace('0.09000', 'nan')), 3) == \
        'un valor no es finito'


def test_etiqueta_ida_y_vuelta():
    assert leer_etiqueta(etiqueta(1234, 56780)) == (1234, 56780)


def test_sin_etiqueta():
    # La sonda del auditor no pone etiqueta, y un frame_id normal tampoco es una.
    assert leer_etiqueta('') is None
    assert leer_etiqueta('world') is None
    assert leer_etiqueta('muestra=abc placa_ms=1') is None


def test_cero_en_la_etiqueta():
    # cc y los motores solo miran muestra y placa_ms: el cero no les estorba.
    assert leer_etiqueta(etiqueta(7, 280, 2)) == (7, 280)
    assert leer_cero(etiqueta(7, 280, 2)) == 2
    assert leer_cero(etiqueta(7, 280)) == 0


def test_sin_cero():
    # La sonda del auditor no trae etiqueta, y una de antes del 29/09 no trae cero.
    assert leer_cero('') is None
    assert leer_cero('muestra=7 placa_ms=280') is None
    assert leer_cero('muestra=7 placa_ms=280 cero=x') is None
