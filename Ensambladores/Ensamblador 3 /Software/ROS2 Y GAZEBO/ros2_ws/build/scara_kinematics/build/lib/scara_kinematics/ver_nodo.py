"""`ver_nodo`: la terminal de UN nodo de junta. Todo lo que dice Alvin (o Simon, o Teodoro).

    ros2 run scara_kinematics ver_nodo alvin
    ros2 run scara_kinematics ver_nodo simon
    ros2 run scara_kinematics ver_nodo teodoro

En la placa (main.py) los tres nodos imprimen por el mismo USB, mezclados. El
nodo `vector` publica cada linea que dice la placa en scara/vector/placa, y
esto se queda solo con las de un nodo:

    el vector que llega      VECTOR [...]          (llega a los tres)
    lo que toma ese nodo     RECIBE Alvin pos ... vel ...  -> vector[2i], vector[2i+1]
    su arranque              [+] Alvin | Meta ... | Actual ... | Velocidad ...
    su telemetria, 10 Hz     posicion, RPM, PWM y destino (su trozo de la linea T:)
    su llegada               [!] Alvin: OBJETIVO ALCANZADO en ... (meta ..., error ...)
    sus errores              ERROR Alvin ..., avisos [!] Alvin ...
    donde se queda           LISTO / PARADO / POS: su numero

Vale igual con la ESP32 de verdad que con la placa de mentira: solo mira el topic.
"""
from __future__ import annotations

import re
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

NOMBRES = ('Alvin', 'Simon', 'Teodoro')
# colores ANSI: que lo importante salte a la vista en la terminal
AZUL, VERDE, ROJO, GRIS, NEGRITA, FIN = ('\033[34m', '\033[32m', '\033[31m', '\033[90m',
                                         '\033[1m', '\033[0m')


class VerNodo(Node):

    def __init__(self, nombre: str):
        super().__init__(f'ver_{nombre.lower()}')
        self.nombre = nombre
        self.i = NOMBRES.index(nombre)
        self.tel = re.compile(nombre + r' (-?[\d.]+)° (-?[\d.]+)rpm'
                              r'(?: pwm (-?\d+) meta (-?[\d.]+)°)?')
        self.create_subscription(String, 'scara/vector/placa', self.on_linea, 100)
        print(f'{NEGRITA}=== {nombre.upper()}: todo lo que dice el nodo {nombre} en la placa '
              f'(main.py) ==={FIN}\n'
              f'    en el vector [pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, '
              f'vel_teodoro]\n'
              f'    {nombre} es el indice {self.i}: toma vector[{2 * self.i}] (posicion) y '
              f'vector[{2 * self.i + 1}] (velocidad)\n'
              f'{GRIS}    (escuchando scara/vector/placa: arranca el nodo `vector` si no sale '
              f'nada){FIN}\n', flush=True)

    def on_linea(self, msg: String):
        linea = msg.data
        n = self.nombre
        if linea.startswith('VECTOR'):
            print(f'\n{AZUL}llega el vector   {linea[7:]}{FIN}', flush=True)
        elif linea.startswith(f'RECIBE {n} '):
            pos, vel = re.search(r'pos (\S+) vel (\S+)', linea).groups()
            print(f'{NEGRITA}{linea}{FIN}\n'
                  f'   -> {n} ha tomado vector[{2 * self.i}] = {pos} (posicion) y '
                  f'vector[{2 * self.i + 1}] = {vel} (velocidad)', flush=True)
        elif linea.startswith('T:'):
            m = self.tel.search(linea)
            if m:
                t = linea.split('|')[0].strip()
                pos, rpm, pwm, meta = m.groups()
                extra = f'   PWM {pwm:>5}   destino {meta:>7}°' if pwm is not None else ''
                print(f'{GRIS}{t:<10}{FIN} {n} {pos:>7}°   {rpm:>6} rpm{extra}', flush=True)
        elif linea.startswith(('LISTO', 'PARADO', 'POS')):
            numeros = linea.split()[1:4]
            if len(numeros) == 3:
                print(f'{VERDE if linea.startswith("LISTO") else GRIS}'
                      f'{linea.split()[0]}: {n} en {numeros[self.i]}°{FIN}', flush=True)
        elif linea.startswith('ERROR') and n in linea:
            print(f'{ROJO}{linea}{FIN}', flush=True)
        elif linea.startswith('RECHAZADO'):
            print(f'{ROJO}{linea}  (la placa no ha movido ningun nodo){FIN}', flush=True)
        elif re.search(rf'\b{n}\b', linea) and not linea.startswith('RECIBE'):
            color = VERDE if 'OBJETIVO ALCANZADO' in linea else ''
            print(f'{color}{linea}{FIN if color else ""}', flush=True)


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(args if args is not None else sys.argv)
    nombre = (argv[1] if len(argv) > 1 else 'alvin').capitalize()
    if nombre not in NOMBRES:
        print(f'no conozco el nodo {nombre!r}: hay alvin, simon y teodoro')
        return 1
    rclpy.init(args=args)
    node = VerNodo(nombre)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
