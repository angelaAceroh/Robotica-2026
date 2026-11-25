#!/usr/bin/env python3
"""
TRANSPORTADOR 1 REAL: el brazo fisico obedece al nodo maestro.

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=42
    cd ~/robot_fisico && python3 nodo_transportador1.py            # brazo fisico
    python3 nodo_transportador1.py --gazebo     # y ademas se ve en la linea de Gazebo
    python3 nodo_transportador1.py --simulado   # sin brazo: prueba del nodo

Sustituye al robot_simulado del transportador 1 y cumple el MISMO contrato que
el, que es el que esperan el maestro y los demas equipos:

    - publica su estado en /transportador1/estado cada 0.5 s
          "listo" | "ocupado" | "error"
    - al recibir una orden en /transportador1/orden publica "ocupado" de
      inmediato, la ejecuta CON EL BRAZO, y publica "listo" de inmediato
    - ignora ordenes mientras esta ocupado (el maestro las reenvia)
    - con /parada = true para el brazo donde este y queda en "error"

QUE HACE CADA ORDEN (tabla ORDENES, abajo)
    P0_a_P1   la trayectoria ensenada (trayectorias.CICLO_ENSENADO), con las
              palas: recoge la caja y la deja, y vuelve al home.
    P1_a_P2   AUN NO HAY TRAYECTORIA ENSENADA.  Con --sin-trayectoria simular
              (por defecto) el brazo NO se mueve: espera 3 s y contesta
              "listo", como el simulado, para que la linea pueda seguir.  Con
              --sin-trayectoria error, contesta "error" y para la linea.
    home      del sitio donde este al home, sin palas.

El brazo tiene que estar EN EL HOME al arrancar: al abrir el puerto la ESP32 se
reinicia y da por hecho que esta ahi.
"""
import argparse
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import config as cfg
import mover_robot_vel as mrv
import trayectorias as tray
from enlace_esp import EnlaceESP

NOMBRE = 'transportador1'
ESPERA_SIMULADA = 3.0        # s que "tarda" una orden sin trayectoria
OCUPADO_MIN = 1.5            # s que se queda en "ocupado" aunque acabe antes.
                             # Si "ocupado" y "listo" salen en el mismo instante
                             # (p.ej. "home" estando ya en el home), el maestro
                             # no llega a ver la confirmacion, cree que la orden
                             # se perdio y la reenvia sin fin.  Probado.

# orden del maestro  ->  que hace el brazo
ORDENES = {
    'P0_a_P1': 'ensenado',   # la trayectoria con la caja
    'P1_a_P2': None,         # SIN ENSENAR TODAVIA
    'home':    'home',
}


class Transportador1(Node):
    def __init__(self, enlace, args):
        super().__init__('transportador1_real')
        self.enlace = enlace
        self.args = args
        self.espejo = None
        if args.gazebo:
            from espejo_ros import EspejoROS
            self.espejo = EspejoROS()

        self.estado = 'listo'
        self.cancelar = threading.Event()
        self.hilo = None

        self.pub_estado = self.create_publisher(String, '/%s/estado' % NOMBRE, 10)
        self.create_subscription(String, '/%s/orden' % NOMBRE, self.cb_orden, 10)
        self.create_subscription(Bool, '/parada', self.cb_parada, 10)
        self.create_timer(0.5, self.publicar_estado)   # latido: "sigo vivo"

        self.get_logger().info(
            'Transportador 1 REAL listo.  Ordenes: %s'
            % ', '.join('%s (%s)' % (o, a or 'sin ensenar: ' + args.sin_trayectoria)
                        for o, a in ORDENES.items()))

    # ------------------------------------------------------------ contrato
    def publicar_estado(self):
        self.pub_estado.publish(String(data=self.estado))

    def cb_orden(self, msg):
        if self.estado != 'listo':
            return                          # ocupado o detenido: se ignora
        if msg.data not in ORDENES:
            self.get_logger().error('Orden desconocida "%s": me pongo en error'
                                    % msg.data)
            self.estado = 'error'
            self.publicar_estado()
            return
        self.estado = 'ocupado'
        self.publicar_estado()              # avisar de inmediato
        self.get_logger().info('Ejecutando "%s"...' % msg.data)
        self.hilo = threading.Thread(target=self.trabajar, args=(msg.data,),
                                     daemon=True)
        self.hilo.start()

    def cb_parada(self, msg):
        if msg.data and self.estado != 'error':
            self.cancelar.set()             # corta la trayectoria si la hay
            self.estado = 'error'
            self.publicar_estado()
            self.get_logger().warn('Parada recibida: brazo detenido donde esta')

    # ------------------------------------------------------------- trabajo
    def trabajar(self, orden):
        t0 = time.time()
        try:
            ok = self.ejecutar_orden(orden)
        except Exception as e:              # nunca dejar al maestro colgado
            self.get_logger().error('Fallo ejecutando "%s": %s' % (orden, e))
            ok = False
        while time.time() - t0 < OCUPADO_MIN and not self.cancelar.is_set():
            time.sleep(0.05)
        if self.cancelar.is_set():
            return                          # la parada ya dejo el estado en error
        self.estado = 'listo' if ok else 'error'
        self.publicar_estado()              # avisar de inmediato
        if ok:
            self.get_logger().info('"%s" terminada' % orden)
        else:
            self.get_logger().error('"%s" NO terminada: me quedo en error.  Revisa '
                                    'el brazo antes de relanzar.' % orden)

    def ejecutar_orden(self, orden):
        accion = ORDENES[orden]
        a = self.args

        if accion is None:
            if a.sin_trayectoria == 'error':
                self.get_logger().error('"%s" no tiene trayectoria ensenada' % orden)
                return False
            self.get_logger().warn('"%s" NO TIENE TRAYECTORIA ENSENADA: el brazo no '
                                   'se mueve, espero %.0f s y contesto listo'
                                   % (orden, ESPERA_SIMULADA))
            t0 = time.time()
            while time.time() - t0 < ESPERA_SIMULADA:
                if self.cancelar.is_set():
                    return False
                time.sleep(0.1)
            return True

        q0 = mrv.leer_postura(self.enlace)
        if q0 is None:
            self.get_logger().error('no llega telemetria de la ESP32')
            return False

        apertura = None
        if accion == 'home':
            home = np.radians(cfg.HOME_GRADOS)
            if np.max(np.abs(np.degrees(q0 - home))) < 1.0:
                self.get_logger().info('ya estoy en el home')
                return True
            hitos = [q0, home]
        else:
            hitos, _ = tray.ciclo_ensenado_juntas()
            if not a.sin_palas:
                apertura = dict(tray.APERTURA_POR_PUNTO)

        muestras = mrv.plan_juntas(hitos, a.perfil, a.vel, a.acel)
        avisos, _ = mrv.revisar(muestras)
        for av in avisos:
            self.get_logger().warn('plan: %s' % av)
        muestras = mrv.anadir_aproximacion(muestras, q0, a.perfil, a.vel, a.acel)

        parado = mrv.ejecutar(muestras, self.enlace, {}, 1, False, self.espejo,
                              None, apertura_por_tramo=apertura,
                              cancelar=self.cancelar)
        return not parado

    def cerrar(self):
        self.cancelar.set()
        if self.hilo is not None:
            self.hilo.join(timeout=3.0)
        if self.espejo is not None:
            self.espejo.cerrar()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--simulado', action='store_true',
                    help='sin brazo: prueba el nodo contra el maestro')
    ap.add_argument('--gazebo', action='store_true',
                    help='publica la postura medida para verla en Gazebo')
    ap.add_argument('--sin-palas', action='store_true',
                    help='no mover las palas en P0_a_P1')
    ap.add_argument('--sin-trayectoria', choices=('simular', 'error'),
                    default='simular',
                    help='que hacer con las ordenes sin trayectoria ensenada')
    ap.add_argument('--perfil', choices=('trapezoidal', 'cubico'),
                    default='trapezoidal')
    ap.add_argument('--vel', type=float, default=35.0)
    ap.add_argument('--acel', type=float, default=80.0)
    args = ap.parse_args()

    if not args.simulado:
        print('\nIMPORTANTE: el brazo tiene que estar en el HOME ahora mismo')
        print('  J1 = %.0f   J2 = %.0f   J3 = %.0f  grados' % cfg.POSE_INICIAL)
        print('  Al abrir el puerto la ESP32 se reinicia y da por hecho que esta ahi.\n')

    enlace = EnlaceESP(simulado=args.simulado)
    rclpy.init()
    nodo = None
    try:
        with enlace:
            nodo = Transportador1(enlace, args)
            rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.cerrar()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
