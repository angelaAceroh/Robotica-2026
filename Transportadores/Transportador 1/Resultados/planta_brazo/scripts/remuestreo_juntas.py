#!/usr/bin/env python3
# Nodo: baja /joint_states de Gazebo de 1000 Hz a 60 Hz.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Remuestreo(Node):
    def __init__(self):
        super().__init__('remuestreo_juntas')
        self.declare_parameter('frecuencia', 60.0)
        f = float(self.get_parameter('frecuencia').value)

        self.ultimo = None
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(JointState, 'joint_states_gz', self.rx, 10)
        self.create_timer(1.0 / f, self.tx)
        self.get_logger().info(f'remuestreando /joint_states a {f:.0f} Hz')

    def rx(self, msg):
        self.ultimo = msg

    def tx(self):
        if self.ultimo is not None:
            self.pub.publish(self.ultimo)


def main():
    rclpy.init()
    n = Remuestreo()
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
