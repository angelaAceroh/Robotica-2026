#!/usr/bin/env python3
"""
Nodo MAESTRO de la línea de empacado.

Coordina 5 robots esclavos: transportador1, transportador2, ensamblador1,
ensamblador2 y ensamblador3. Ningún robot se mueve sin una orden de este nodo.

Contrato de comunicación (igual para todos los robots):
    /<robot>/orden    std_msgs/String   maestro -> robot   (la tarea a ejecutar)
    /<robot>/estado   std_msgs/String   robot -> maestro   "listo" | "ocupado" | "error"
    /parada           std_msgs/Bool     true = todos se detienen
    /linea/estado     std_msgs/String   paso actual de la línea (para monitorear)

Cómo avanza cada paso:
    1. El maestro envía la orden al robot.
    2. Espera a que el robot responda "ocupado" (si no responde, reenvía la orden).
    3. Espera a que el robot vuelva a "listo"  -> el paso terminó, sigue el siguiente.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


ROBOTS = ['transportador1', 'transportador2',
          'ensamblador1', 'ensamblador2', 'ensamblador3']

# Secuencia de la línea: (robot, orden).  Se ejecuta en este orden, un paso a la vez.
SECUENCIA = [
    ('transportador1', 'P0_a_P1'),     # T1 lleva la caja de la posición 0 a la 1
    ('ensamblador1',   'ensamblar'),   # E1 trabaja en la posición 1
    ('transportador1', 'P1_a_P2'),     # T1 lleva la caja de la 1 a la 2
    ('transportador1', 'home'),        # T1 vuelve a home (libera la zona de la posición 2)
    ('ensamblador2',   'ensamblar'),   # E2 trabaja en la posición 2
    ('transportador2', 'P2_a_P3'),     # T2 lleva la caja de la 2 a la 3
    ('ensamblador3',   'ensamblar'),   # E3 trabaja en la posición 3
    ('transportador2', 'P3_a_P4'),     # T2 lleva la caja de la 3 a la 4 (salida)
    ('transportador2', 'home'),        # T2 vuelve a home
]


class Maestro(Node):
    def __init__(self):
        super().__init__('maestro')

        # ---------------- Parámetros ----------------
        self.declare_parameter('repetir', False)      # True = repite el ciclo sin parar
        self.declare_parameter('timeout_paso', 60.0)  # s máximos para terminar un paso
        self.declare_parameter('timeout_vivo', 3.0)   # s sin recibir estado = robot caído
        self.declare_parameter('reenvio', 1.0)        # s entre reenvíos de una orden

        g = lambda n: self.get_parameter(n).value
        self.repetir = bool(g('repetir'))
        self.timeout_paso = float(g('timeout_paso'))
        self.timeout_vivo = float(g('timeout_vivo'))
        self.reenvio = float(g('reenvio'))

        # ---------------- Estado de cada robot ----------------
        self.estado = {r: None for r in ROBOTS}   # último estado recibido
        self.ultimo = {r: None for r in ROBOTS}   # instante en que llegó

        # ---------------- Comunicación ----------------
        self.pub_orden = {r: self.create_publisher(String, f'/{r}/orden', 10)
                          for r in ROBOTS}
        for r in ROBOTS:
            self.create_subscription(
                String, f'/{r}/estado',
                lambda msg, r=r: self.cb_estado(r, msg), 10)

        self.pub_parada = self.create_publisher(Bool, '/parada', 10)
        self.create_subscription(Bool, '/parada', self.cb_parada, 10)
        self.pub_linea = self.create_publisher(String, '/linea/estado', 10)

        # ---------------- Máquina de estados ----------------
        # Fases: ESPERANDO_ROBOTS -> EJECUTANDO -> TERMINADO   (o DETENIDO si algo falla)
        self.fase = 'ESPERANDO_ROBOTS'
        self.paso = 0
        self.ciclo_n = 1
        self.t_paso = 0.0          # cuándo empezó el paso actual
        self.t_envio = 0.0         # cuándo se envió la orden por última vez
        self.vio_ocupado = False   # el robot ya confirmó que recibió la orden
        self.t_aviso = 0.0

        self.create_timer(0.1, self.ciclo)
        self.reportar('Maestro iniciado. Esperando a que los 5 robots estén "listo"...')

    # ======================================================================
    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def reportar(self, texto, nivel='info'):
        # rclpy cachea la severidad POR SITIO DE LLAMADA: un solo
        # getattr(logger, nivel)(texto) lanza
        # "Logger severity cannot be changed between calls" la primera vez
        # que se reporta un error despues de haber reportado un info.
        # Por eso cada nivel necesita su propia linea.
        if nivel == 'error':
            self.get_logger().error(texto)
        elif nivel == 'warn':
            self.get_logger().warn(texto)
        else:
            self.get_logger().info(texto)
        self.pub_linea.publish(String(data=texto))

    # ---------------- Callbacks ----------------
    def cb_estado(self, robot, msg):
        self.estado[robot] = msg.data.strip().lower()
        self.ultimo[robot] = self.ahora()

    def cb_parada(self, msg):
        if msg.data and self.fase != 'DETENIDO':
            self.detener('Se recibió una parada de emergencia en /parada')

    # ---------------- Acciones ----------------
    def enviar_orden(self):
        robot, orden = SECUENCIA[self.paso]
        self.pub_orden[robot].publish(String(data=orden))
        self.t_envio = self.ahora()

    def iniciar_paso(self):
        robot, orden = SECUENCIA[self.paso]
        self.t_paso = self.ahora()
        self.vio_ocupado = False
        self.reportar(f'[Ciclo {self.ciclo_n} | Paso {self.paso + 1}/{len(SECUENCIA)}] '
                      f'{robot} -> "{orden}"')
        self.enviar_orden()

    def detener(self, motivo):
        self.fase = 'DETENIDO'
        self.pub_parada.publish(Bool(data=True))
        self.reportar(f'LÍNEA DETENIDA: {motivo}', 'error')

    # ---------------- Lazo principal (cada 0.1 s) ----------------
    def ciclo(self):
        if self.fase in ('DETENIDO', 'TERMINADO'):
            return
        t = self.ahora()

        # 1) Esperar a que todos los robots estén conectados y en "listo"
        if self.fase == 'ESPERANDO_ROBOTS':
            faltan = [r for r in ROBOTS if self.estado[r] != 'listo']
            if not faltan:
                self.reportar('Todos los robots están listos. Arranca la línea.')
                self.fase = 'EJECUTANDO'
                self.iniciar_paso()
            elif t - self.t_aviso > 2.0:
                self.get_logger().info(f'Esperando a: {", ".join(faltan)}')
                self.t_aviso = t
            return

        # 2) Seguridad: ningún robot en error ni desconectado
        for r in ROBOTS:
            if self.estado[r] == 'error':
                self.detener(f'{r} reportó "error"')
                return
            if t - self.ultimo[r] > self.timeout_vivo:
                self.detener(f'{r} no publica su estado hace más de {self.timeout_vivo:.0f} s')
                return

        # 3) Avanzar el paso actual
        robot, orden = SECUENCIA[self.paso]

        if t - self.t_paso > self.timeout_paso:
            self.detener(f'{robot} no terminó "{orden}" en {self.timeout_paso:.0f} s')
            return

        if not self.vio_ocupado:
            if self.estado[robot] == 'ocupado':
                self.vio_ocupado = True
                self.get_logger().info(f'{robot} confirmó y está ejecutando "{orden}"')
            elif t - self.t_envio > self.reenvio:
                self.get_logger().warn(f'{robot} no confirma "{orden}", reenviando...')
                self.enviar_orden()
            return

        if self.estado[robot] == 'listo':
            self.get_logger().info(f'{robot} terminó "{orden}"')
            self.paso += 1

            if self.paso < len(SECUENCIA):
                self.iniciar_paso()
            elif self.repetir:
                self.reportar(f'Ciclo {self.ciclo_n} terminado. Empieza el siguiente.')
                self.ciclo_n += 1
                self.paso = 0
                self.iniciar_paso()
            else:
                self.fase = 'TERMINADO'
                self.reportar(f'Ciclo {self.ciclo_n} terminado. La caja salió por la posición 4.')


def main():
    rclpy.init()
    nodo = Maestro()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
