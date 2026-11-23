"""El main.py de la ESP32, ejecutado en el PC: que cada nodo entienda bien el vector.

Corre firmware/micropython/main.py SIN TOCAR con el `machine` de mentira de
test/micropython_falso: tres motores con inercia y encoder en cuadratura, con los
pines y los 5500 pulsos por vuelta de los programas de Sergio.

Es la respuesta a «¿como se que a cada nodo le llega lo suyo y lo entiende?»:
se le manda el vector como lo manda el PC y se lee lo que contesta la placa.

    pytest src/scara_kinematics/test/test_main_micropython.py     (desde ~/ros2_ws)
"""
import os
import queue
import re
import subprocess
import sys
import threading
import time

import pytest
import yaml

from scara_kinematics import placa_simulada as PS

AQUI = os.path.dirname(os.path.abspath(__file__))
PAQUETE = os.path.dirname(AQUI)
MAIN = os.path.join(PAQUETE, 'firmware', 'micropython', 'main.py')
MAIN_CONTROL = os.path.join(PAQUETE, 'firmware', 'micropython', 'main_control.py')
FALSO = os.path.join(AQUI, 'micropython_falso')


def checksum(texto: str) -> str:
    """La misma cuenta que main.py y el nodo `vector`."""
    return '%02X' % (sum(texto.encode()) & 0xFF)


def con_checksum(vector) -> str:
    texto = '[' + ', '.join(f'{v:g}' for v in vector) + ']'
    return f'{texto} *{checksum(texto)}'


class Placa:
    """main.py en un subproceso: se le escribe por stdin y se lee su stdout."""

    def __init__(self, carpeta, fallos=''):
        with open(MAIN) as f:
            codigo = f.read()
        with open(os.path.join(carpeta, 'main.py'), 'w') as f:
            f.write(codigo)
        env = dict(os.environ, PYTHONPATH=FALSO, FALLOS=fallos)
        self.p = subprocess.Popen([sys.executable, '-u', 'main.py'], cwd=carpeta, env=env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True)
        self.lineas: queue.Queue = queue.Queue()
        threading.Thread(target=self._leer, daemon=True).start()
        self.esperar(r'^=+$', 10.0)          # el final del saludo de arranque

    def _leer(self):
        for linea in self.p.stdout:
            self.lineas.put(linea.rstrip('\n'))

    def mandar(self, linea: str):
        self.p.stdin.write(linea + '\r\n')
        self.p.stdin.flush()

    def esperar(self, patron: str, plazo: float = 10.0) -> list[str]:
        """Lineas hasta la primera que case con `patron` (incluida)."""
        vistas, fin = [], time.monotonic() + plazo
        while time.monotonic() < fin:
            try:
                linea = self.lineas.get(timeout=0.1)
            except queue.Empty:
                continue
            vistas.append(linea)
            if re.search(patron, linea):
                return vistas
        raise AssertionError(f'no llego «{patron}». Llego:\n' + '\n'.join(vistas[-30:]))

    def cerrar(self):
        self.p.kill()


@pytest.fixture
def placa(tmp_path):
    p = Placa(str(tmp_path))
    yield p
    p.cerrar()


def posiciones(linea: str) -> list[float]:
    return [float(v) for v in linea.split()[1:4]]


def test_cada_nodo_toma_su_pareja_y_llega(placa):
    # Teodoro en mm y mm/min, como Teodoro.py: 2 mm a 60 mm/min son 2 s
    vector = [10.0, 14.0, -22.0, 30.0, -2.0, 60.0]
    placa.mandar(con_checksum(vector))
    vistas = placa.esperar(r'^LISTO', 20.0)
    assert '[10, 14, -22, 30, -2, 60] checksum ok' in ' '.join(vistas)
    recibe = {m.group(1): (float(m.group(2)), float(m.group(3)))
              for m in (re.match(r'^RECIBE (\w+) pos (\S+) vel (\S+)', ln) for ln in vistas)
              if m}
    assert recibe == {'Alvin': (10.0, 14.0), 'Simon': (-22.0, 30.0), 'Teodoro': (-2.0, 60.0)}
    final = posiciones(vistas[-1])
    for pedido, llegado in zip(vector[0::2], final):
        assert abs(llegado - pedido) < 6.0, (pedido, llegado)     # inercia de la simulacion
    assert abs(final[2] + 2.0) < 0.2                              # Teodoro va despacio
    # cada nodo dice su error al llegar
    assert sum('OBJETIVO ALCANZADO' in ln and 'error' in ln for ln in vistas) == 3


def test_velocidad_cero_ese_nodo_no_se_mueve(placa):
    placa.mandar(con_checksum([10.0, 0.0, -22.0, 30.0, 0.0, 60.0]))
    final = posiciones(placa.esperar(r'^LISTO', 20.0)[-1])
    assert final[0] == 0.0 and final[2] == 0.0 and final[1] < -15.0


def test_checksum_malo_no_mueve_nada(placa):
    texto = '[10, 14, -22, 30, 0, 60]'
    placa.mandar(f'{texto} *00' if checksum(texto) != '00' else f'{texto} *01')
    vistas = placa.esperar(r'^ERROR vector checksum', 5.0)
    assert not any(ln.startswith('RECIBE') for ln in vistas)
    placa.mandar('?')
    assert posiciones(placa.esperar(r'^POS')[-1]) == [0.0, 0.0, 0.0]


def test_si_uno_se_sale_del_recorrido_no_se_mueve_ninguno(placa):
    placa.mandar(con_checksum([-70.0, 14.0, -22.0, 30.0, 0.0, 60.0]))   # Alvin < -60
    vistas = placa.esperar(r'^RECHAZADO', 5.0)
    assert any(ln.startswith('ERROR Alvin') for ln in vistas)
    time.sleep(1.0)
    placa.mandar('?')
    assert posiciones(placa.esperar(r'^POS')[-1]) == [0.0, 0.0, 0.0]


def test_a_mano_sin_checksum_tambien_vale(placa):
    placa.mandar('[5, 14, 0, 0, 0, 0]')
    vistas = placa.esperar(r'^LISTO', 20.0)
    assert any('checksum sin' in ln for ln in vistas)


def test_los_limites_de_la_placa_son_los_del_yaml(placa):
    with open(os.path.join(PAQUETE, 'config', 'micropython.yaml')) as f:
        yaml_lim = yaml.safe_load(f)['/**']['ros__parameters']['limites_motor']
    placa.mandar('l')
    linea = placa.esperar(r'^LIMITES')[-1]
    numeros = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?', linea)]
    assert numeros == [float(v) for v in yaml_lim], (
        'main.py (tabla MOTORES) y config/micropython.yaml (limites_motor) no '
        'tienen los mismos limites')


def test_un_motor_al_reves_para_a_los_tres(tmp_path):
    placa = Placa(str(tmp_path), fallos='alvin_inv')
    try:
        placa.mandar(con_checksum([15.0, 14.0, -40.0, 30.0, 0.0, 60.0]))
        vistas = placa.esperar(r'^PARADO', 10.0)
        assert any(ln.startswith('ERROR Alvin va al reves') for ln in vistas)
    finally:
        placa.cerrar()


def test_la_h_no_a_alvin_y_fijar(placa):
    placa.mandar('h alvin')                    # Alvin.py: «NO DARLE A LA H»
    placa.esperar(r'NO DARLE A LA H', 5.0)
    placa.mandar('h')
    placa.esperar(r'uso: h <motor>', 5.0)
    placa.mandar('fijar teodoro -5')
    placa.esperar(r'Teodoro: la posici.n actual ahora es -5\.00mm', 5.0)
    placa.mandar('fijar alvin 36')     # 550 pulsos justos
    placa.esperar(r'Alvin: la posici.n actual ahora es 36\.00°', 5.0)
    placa.mandar('h teodoro')
    placa.esperar(r'Teodoro: la posici.n actual ahora es 0\.00mm', 5.0)
    placa.mandar('?')
    assert posiciones(placa.esperar(r'^POS')[-1]) == [36.0, 0.0, 0.0]


def test_main_py_y_main_control_py_tienen_la_misma_tabla():
    """Los dos programas de la placa cuentan igual (pines, pulsos, unidades y
    recorridos): se puede pasar de uno a otro sin que cambie el cero."""
    with open(MAIN) as f:
        uno = {m[0]: (m[2:]) for m in PS.tabla(f.read(), 'MOTORES')}
    with open(MAIN_CONTROL) as f:
        otro = {m[0]: m[1:10] for m in PS.tabla(f.read(), 'MOTORES')}
    assert uno == otro
    with open(MAIN) as f:
        assert [m[1] for m in PS.tabla(f.read(), 'MOTORES')] == [0, 1, 2]     # INDICE
