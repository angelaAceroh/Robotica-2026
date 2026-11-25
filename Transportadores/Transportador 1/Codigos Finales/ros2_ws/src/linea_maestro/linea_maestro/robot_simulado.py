#!/usr/bin/env python3
"""
Robot SIMULADO para probar el maestro sin tener los robots reales.

Se hace pasar por cualquiera de los 5 robots (parámetro 'nombre'). No se mueve:
solo cumple el contrato de comunicación, igual que debe hacerlo el nodo real
de cada grupo:

    - Publica su estado en /<nombre>/estado cada 0.5 s ("listo" | "ocupado" | "error").
    - Al recibir una orden en /<nombre>/orden:
          publica "ocupado" de inmediato, "trabaja" 'duracion' segundos
          y luego publica "listo" de inmediato.
    - Ignora órdenes mientras está ocupado (el maestro puede reenviarlas).
    - Si llega /parada = true, se detiene y queda en "error".
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class RobotSimulado(Node):
    def __init__(self):
        super().__init__('robot_simulado')
        self.declare_parameter('nombre', 'transportador1')
        self.declare_parameter('duracion', 2.0)   # s que "tarda" cada tarea

        self.nombre = self.get_parameter('nombre').value
        self.duracion = float(self.get_parameter('duracion').value)

        self.pub_estado = self.create_publisher(String, f'/{self.nombre}/estado', 10)
        self.create_subscription(String, f'/{self.nombre}/orden', self.cb_orden, 10)
        self.create_subscription(Bool, '/parada', self.cb_parada, 10)

        self.estado = 'listo'
        self.t_fin = None

        self.create_timer(0.5, self.publicar_estado)   # latido: "sigo vivo"
        self.create_timer(0.1, self.revisar_tarea)
        self.get_logger().info(f'Robot simulado "{self.nombre}" listo '
                               f'(cada tarea dura {self.duracion:.1f} s)')

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publicar_estado(self):
        self.pub_estado.publish(String(data=self.estado))

    def cb_orden(self, msg):
        if self.estado != 'listo':
            return                      # ocupado o detenido: ignora la orden
        self.estado = 'ocupado'
        self.publicar_estado()          # avisar de inmediato
        self.t_fin = self.ahora() + self.duracion
        self.get_logger().info(f'Ejecutando "{msg.data}"...')

    def revisar_tarea(self):
        if self.estado == 'ocupado' and self.ahora() >= self.t_fin:
            self.estado = 'listo'
            self.publicar_estado()      # avisar de inmediato
            self.get_logger().info('Tarea terminada')

    def cb_parada(self, msg):
        if msg.data and self.estado != 'error':
            self.estado = 'error'
            self.publicar_estado()
            self.get_logger().warn('Parada recibida: robot detenido')


def main():
    rclpy.init()
    nodo = RobotSimulado()
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
