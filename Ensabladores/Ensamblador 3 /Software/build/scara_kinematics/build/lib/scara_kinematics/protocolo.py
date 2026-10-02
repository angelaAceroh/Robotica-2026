"""Protocolo de linea con la ESP32 y quien es quien en el brazo.

Lo comparten la aduana -el unico nodo que abre el puerto serie-, los tres
motores y cc -que leen la etiqueta de cada muestra- y la ventana brazo_hw, que
ensena los mismos bits de estado sin tocar el enlace.

Formato de las lineas que manda la placa:

    > ms q1 q2 q3 u1 u2 u3 flags*CS    telemetria (ms = reloj de la placa)
    ! texto                            aviso
    # texto                            traza

y de las que se le mandan, siempre con "*" y el XOR de la carga en hexadecimal:

    T q1 q2 q3      consigna de posicion (rad, rad, m)
    E 0|1           potencia
    Z               fija el cero en la pose actual
    R i lo hi vmax  limites de la junta i
    F hz            frecuencia de telemetria
    A i u ms        empujon en lazo abierto (prueba de motor)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import serial.tools.list_ports

# Bits del campo de estado que manda el firmware.
F_HABILITADO = 0x01
F_ENLACE = 0x02
F_CERO = 0x04
F_LIMITE = 0x08
F_SATURACION = 0x10
F_FALLO = 0x20

# Fabricantes de los conversores USB-serie que llevan las placas ESP32.
VID_CONOCIDOS = {0x10C4: 'CP210x', 0x1A86: 'CH34x', 0x0403: 'FTDI', 0x303A: 'Espressif'}


@dataclass(frozen=True)
class Motor:
    """Un motor del brazo: su nodo, su junta y como probarlo a mano."""

    clave: str           # nombre del nodo y raiz de sus topics
    nombre: str          # como lo llama Sergio
    junta: str           # la junta del URDF que mueve
    que: str             # que parte del brazo es
    pines: tuple         # IN1, IN2, ENA, encoder A, encoder B (A None = un canal)
    lineal: bool         # husillo (m) en vez de giro (rad)
    empujon: float       # ciclo de trabajo de la prueba en lazo abierto
    ms: int              # cuanto dura ese empujon

    def encoder(self) -> str:
        """Los pines del encoder tal y como los lee el firmware."""
        a, b = self.pines[3], self.pines[4]
        return f'{a}/{b}' if a is not None else f'{b} (un solo canal)'


# En el orden de joint_names del YAML: [juntura1, juntura2, vertiacal1].
# El mapeo es "motor = eslabon que lo ALOJA", decidido mirando el brazo montado.
# Los pines son los del firmware (PINES_J1..J3 de scara_esp32.ino), no los del
# esquema teorico. Simon va con UN canal de encoder: su GPIO 13 no cuenta nunca,
# asi que el firmware lleva SIN_PIN en el canal A y solo mira el 12. Poner aqui
# "12/13" mandaba a mirar un pin que el firmware ni lee.
MOTORES = (
    Motor('alvin', 'Alvin', 'juntura1', 'hombro', (16, 4, 17, 26, 25), False, 0.60, 150),
    Motor('simon', 'Simón', 'juntura2', 'codo', (19, 18, 5, None, 12), False, 0.60, 150),
    Motor('teodoro', 'Teodoro', 'vertiacal1', 'husillo', (22, 21, 23, 14, 27), True, 0.70, 300),
)

POR_CLAVE = {m.clave: m for m in MOTORES}
POR_JUNTA = {m.junta: m for m in MOTORES}


def checksum(carga: str) -> str:
    """XOR de la carga, en dos digitos hexadecimales."""
    c = 0
    for ch in carga.encode():
        c ^= ch
    return f'{c:02X}'


def linea(carga: str) -> bytes:
    """La carga ya lista para escribir en el puerto."""
    return f'{carga}*{checksum(carga)}\n'.encode()


@dataclass(frozen=True)
class Telemetria:
    """Una linea "> ms q1 q2 q3 u1 u2 u3 flags*CS" ya comprobada y en vectores."""

    ms: int              # reloj de la placa desde que arranco, en milisegundos
    q: tuple             # posiciones, en el orden de joint_names (rad, rad, m)
    u: tuple             # ciclo de trabajo con signo de cada motor (-1..1)
    flags: int           # palabra de estado


def leer_telemetria(texto: str, n: int) -> Telemetria | str:
    """Parte una linea de telemetria. Devuelve la muestra, o por que se tira.

    La placa firma cada linea con el XOR de todo lo que va antes del '*'. Si no
    cuadra, algun byte cambio por el camino, y una linea rota no se puede
    arreglar: se tira entera antes de que un numero falso entre en la cadena.
    """
    cuerpo, ast, firma = texto.rpartition('*')
    if not ast:
        return 'sin checksum'
    if firma.strip().upper() != checksum(cuerpo):
        return 'checksum incorrecto'
    campos = cuerpo[1:].split()
    # 1 reloj + n posiciones + n esfuerzos + 1 palabra de estado
    if len(campos) != 2 + 2 * n:
        return f'{len(campos)} campos en vez de {2 + 2 * n}'
    try:
        ms = int(campos[0])
        q = tuple(float(v) for v in campos[1:1 + n])
        u = tuple(float(v) for v in campos[1 + n:1 + 2 * n])
        flags = int(campos[1 + 2 * n])
    except ValueError:
        return 'un campo no es un numero'
    if not all(math.isfinite(v) for v in q + u):
        return 'un valor no es finito'
    return Telemetria(ms, q, u, flags)


# ---------------------------------------------------------------- la etiqueta
# Cada muestra de la placa viaja por la cadena partida en tres mensajes, uno por
# motor, y cc tiene que volver a juntar las tres piezas de la MISMA muestra. El
# sello de tiempo no basta para eso: con use_sim_time el reloj es el de Gazebo,
# que vale 0 hasta que arranca y se para con la simulacion, y entonces muestras
# distintas salen con el mismo sello. Por eso cada pieza lleva su etiqueta:
#
#     muestra=1234 placa_ms=56780 cero=1
#
# el numero de muestra que le pone la aduana (1, 2, 3... sin huecos), el reloj
# de la placa cuando se midio y cuantas veces se ha fijado el cero desde que la
# aduana arranco. Va en header.frame_id, que en un JointState no usa nadie: una
# junta no tiene marco de referencia.
#
# El cero va en la etiqueta porque al fijarlo la lectura salta de golpe a 0 sin
# que el brazo se haya movido. Entre dos muestras con distinto cero no hay
# movimiento que medir: ni velocidad, ni "salto imposible".

def etiqueta(muestra: int, placa_ms: int, cero: int = 0) -> str:
    """El frame_id de las tres piezas de una muestra."""
    return f'muestra={muestra} placa_ms={placa_ms} cero={cero}'


def leer_etiqueta(frame_id: str) -> tuple[int, int] | None:
    """(muestra, placa_ms) de un frame_id, o None si no trae etiqueta."""
    campos = dict(p.split('=', 1) for p in frame_id.split() if '=' in p)
    try:
        return int(campos['muestra']), int(campos['placa_ms'])
    except (KeyError, ValueError):
        return None


def leer_cero(frame_id: str) -> int | None:
    """Cuantos ceros llevaba la placa en esa muestra, o None si no lo dice."""
    campos = dict(p.split('=', 1) for p in frame_id.split() if '=' in p)
    try:
        return int(campos['cero'])
    except (KeyError, ValueError):
        return None


def buscar_puerto() -> str | None:
    """Primer puerto que parezca una ESP32, mirando el VID del conversor."""
    candidatos = [p for p in serial.tools.list_ports.comports()
                  if p.vid in VID_CONOCIDOS]
    return candidatos[0].device if candidatos else None


def formatear(clave: str, q: float) -> str:
    """El valor de una junta en unidades de taller: grados o milimetros."""
    m = POR_CLAVE[clave]
    return f'{q * 1000.0:.1f} mm' if m.lineal else f'{math.degrees(q):+.1f}°'
