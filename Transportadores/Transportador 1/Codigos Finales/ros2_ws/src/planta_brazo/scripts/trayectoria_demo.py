#!/usr/bin/env python3
"""
Recorre la trayectoria articular de demostracion sin necesidad del panel.

Con interpolacion quintica (perfil suave, velocidad y aceleracion nulas en
cada punto de paso).

    ros2 run planta_brazo trayectoria_demo.py
    ros2 run planta_brazo trayectoria_demo.py --ros-args -p periodo:=2.5
    ros2 run planta_brazo trayectoria_demo.py --ros-args -p modo:=gazebo

    modo:=rviz    publica /joint_states (el robot "teletransporta", sin fisica)
    modo:=gazebo  publica consignas /planta_brazo/J{n}/cmd_pos y deja que el
                  controlador de Gazebo las siga
"""
import rclpy
from rclpy.node import Node

from enlace_juntas import EnlaceJuntas
from trayectorias import PUNTOS, PUNTOS_RAD, interpolar


class TrayectoriaDemo(Node):
    def __init__(self):
        super().__init__('trayectoria_demo')
        self.declare_parameter('periodo', 2.0)     # s por tramo
        self.declare_parameter('frecuencia', 50.0)
        self.declare_parameter('modo', 'rviz')
        self.T = float(self.get_parameter('periodo').value)
        f = float(self.get_parameter('frecuencia').value)
        modo = str(self.get_parameter('modo').value).lower()

        self.enlace = EnlaceJuntas(self, modo)
        self.puntos = PUNTOS_RAD
        self.i = 0
        self.t = 0.0
        self.dt = 1.0 / f
        self.create_timer(self.dt, self.paso)
        self.get_logger().info(
            f'Trayectoria de {len(PUNTOS)} puntos, {self.T:.1f} s por tramo '
            f'(modo {modo}).')

    def paso(self):
        self.enlace.enviar(interpolar(self.i, self.t, self.T, self.puntos))
        self.t += self.dt
        if self.t >= self.T:
            self.t = 0.0
            self.i = (self.i + 1) % len(self.puntos)
            self.get_logger().info('punto %d/%d' % (self.i, len(self.puntos)))


def main():
    rclpy.init()
    n = TrayectoriaDemo()
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
