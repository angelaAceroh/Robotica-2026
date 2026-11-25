"""Donde quedaron las palas la ultima vez (2026-11-24).

Al reiniciarse, la ESP32 pone la cuenta del gripper a CERO esten donde esten
las palas: no sabe si estan recogidas o fuera.  Y el referenciado, si no
encuentra el tope a la primera, las SACA 60 cuentas para volver a probar (por
si ya estaban en el tope).  Con las palas fuera y el motor trabado, cada
intento fallido las acerca a 580, donde se salen de la guia.

Por eso el PC apunta aqui la ultima posicion MEDIDA (ya referenciada) al
terminar cada movimiento, y antes de referenciar decide:

    quedaron junto al tope (<= CERCA_TOPE)  ->  referenciado normal
    quedaron fuera, o no se sabe            ->  referenciado SOLO HACIA DENTRO
                                                (si falla, no las saca)
"""
import json
import os
import time

import config as cfg

ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'estado_palas.json')
CERCA_TOPE = 60          # cuentas: hasta aqui sacarlas 60 mas sigue siendo seguro


def guardar(cuentas, motivo=''):
    """Apunta la ultima posicion medida de las palas (cuentas referenciadas)."""
    try:
        with open(ARCHIVO, 'w') as f:
            json.dump({'cuentas': int(round(cuentas)), 'cuando': time.strftime('%Y-%m-%d %H:%M:%S'),
                       'motivo': motivo}, f)
    except OSError:
        pass


def leer():
    """(cuentas, cuando) o (None, None) si no hay registro."""
    try:
        with open(ARCHIVO) as f:
            d = json.load(f)
        return int(d['cuentas']), d.get('cuando', '?')
    except (OSError, ValueError, KeyError):
        return None, None


def consigna_referenciar():
    """Que consigna mandar para referenciar, y por que (texto)."""
    c, cuando = leer()
    if c is not None and c <= CERCA_TOPE:
        return cfg.GRIPPER_HOME, ('quedaron en %d cuentas (%s): referenciado normal'
                                  % (c, cuando))
    if c is None:
        por = 'no se sabe donde quedaron'
    else:
        por = 'quedaron FUERA, en %d cuentas (%s)' % (c, cuando)
    return cfg.GRIPPER_HOME_SIN_RETIRADA, por + ': referenciado SOLO HACIA DENTRO'
