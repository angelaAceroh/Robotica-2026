#!/usr/bin/env python3
# Nodo: pose del TCP y de la brida desde /joint_states, y traza para RViz.
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker

import cinematica as cin
from cinematica import Cinematica


class CinematicaDirecta(Node):
    def __init__(self):
        super().__init__('cinematica_directa')
        for n, v in (('d1', cin.D1), ('a1', cin.A1), ('L2', cin.L2),
                     ('L3', cin.L3), ('Lg', cin.LG)):
            self.declare_parameter(n, v)
        self.declare_parameter('imprimir', True)
        self.declare_parameter('traza', True)
        self.declare_parameter('traza_max', 2000)
        self.declare_parameter('traza_paso', 0.004)

        g = lambda n: self.get_parameter(n).value
        self.d1, self.a1 = g('d1'), g('a1')
        self.L2, self.L3, self.Lg = g('L2'), g('L3'), g('Lg')
        self.imprimir = g('imprimir')
        self.usar_traza = g('traza')
        self.traza_max = int(g('traza_max'))
        self.traza_paso = float(g('traza_paso'))

        self.cin = Cinematica(self.d1, self.a1, self.L2, self.L3, self.Lg)

        self.pub_tcp = self.create_publisher(PoseStamped, 'tcp_pose', 10)
        self.pub_brida = self.create_publisher(PoseStamped, 'brida_pose', 10)
        self.pub_traza = self.create_publisher(Marker, 'tcp_traza', 10)
        self.create_subscription(JointState, 'joint_states', self.cb, 10)
        self.create_subscription(Empty, 'traza/limpiar', self.limpiar, 10)

        self.traza = self._nuevo_marker()
        self._t = 0
        self.get_logger().info(
            'CD lista  d1=%.5f  a1=%.5f  L2=%.5f  L3=%.5f  Lg=%.5f [m]  '
            '(la traza sigue la punta de los dedos)'
            % (self.d1, self.a1, self.L2, self.L3, self.Lg))

    def _nuevo_marker(self):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = 'tcp'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.003
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.35, 0.0, 1.0
        m.pose.orientation.w = 1.0
        return m

    def limpiar(self, _msg):
        borrar = self._nuevo_marker()
        borrar.action = Marker.DELETEALL
        self.pub_traza.publish(borrar)
        self.traza = self._nuevo_marker()
        self.get_logger().info('traza borrada')

    def punto(self, q1, q2, q3, largo3):
        return self.cin.directa((q1, q2, q3),
                                'tcp' if largo3 > self.L3 else 'brida')

    def _pose(self, xyz, stamp):
        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        p.pose.orientation.w = 1.0
        return p

    def cb(self, msg: JointState):
        try:
            q = [msg.position[msg.name.index(n)] for n in ('J1', 'J2', 'J3')]
        except (ValueError, IndexError):
            return

        stamp = self.get_clock().now().to_msg()
        p_brida = self.punto(*q, largo3=self.L3)
        p_tcp = self.punto(*q, largo3=self.L3 + self.Lg)
        self.pub_brida.publish(self._pose(p_brida, stamp))
        self.pub_tcp.publish(self._pose(p_tcp, stamp))

        if self.usar_traza:
            x, y, z = p_tcp
            pt = Point(x=x, y=y, z=z)
            d2 = self.traza_paso ** 2
            ult = self.traza.points[-1] if self.traza.points else None
            if ult is None or ((pt.x - ult.x) ** 2 + (pt.y - ult.y) ** 2
                               + (pt.z - ult.z) ** 2) > d2:
                self.traza.points.append(pt)
                while len(self.traza.points) > self.traza_max:
                    self.traza.points.pop(0)
                self.traza.header.stamp = stamp
                self.pub_traza.publish(self.traza)

        self._t += 1
        if self.imprimir and self._t % 30 == 0:
            self.get_logger().info(
                'q = [%7.2f %7.2f %7.2f] deg   ->   TCP = [%7.1f %7.1f %7.1f] mm'
                % (math.degrees(q[0]), math.degrees(q[1]), math.degrees(q[2]),
                   p_tcp[0] * 1000, p_tcp[1] * 1000, p_tcp[2] * 1000))


def main():
    rclpy.init()
    n = CinematicaDirecta()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
