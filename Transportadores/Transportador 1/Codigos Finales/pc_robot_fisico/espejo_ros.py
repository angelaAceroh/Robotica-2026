#!/usr/bin/env python3
"""
ESPEJO EN GAZEBO: el robot simulado repite lo que hace el fisico.

Se publica la posicion MEDIDA en los encoders como consigna de los
controladores de Gazebo, asi que el brazo simulado sigue al real.

No es un nodo aparte a proposito: el puerto serie solo lo puede abrir un
programa a la vez, y mover_robot_vel.py ya lo tiene.  Asi que es el propio
programa de control quien publica, reutilizando la telemetria que ya lee.

Desde las correcciones del 2026-10-23 el modelo de Gazebo tiene las MISMAS
medidas que el robot construido (L1=0.200, a1=0.05815, L2=0.145, L3=0.16285,
L4=0.100 provisional), asi que lo que se ve en pantalla es del mismo tamano
que el robot real, no solo los mismos angulos.
"""
MODELO = 'planta_brazo'
JUNTAS = ('J1', 'J2', 'J3')


class EspejoROS:
    """Publica las posiciones medidas en los topicos de Gazebo."""

    def __init__(self, verboso=True):
        self.ok = False
        self.nodo = None
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float64
        except ImportError:
            if verboso:
                print('[espejo] falta rclpy: no se publica a Gazebo.')
                print('         Haz primero:  source /opt/ros/jazzy/setup.bash')
            return

        self._rclpy = rclpy
        self._Float64 = Float64
        if not rclpy.ok():
            rclpy.init()
        self.nodo = Node('espejo_robot_fisico')
        self.pubs = [self.nodo.create_publisher(
            Float64, '/%s/%s/cmd_pos' % (MODELO, j), 10) for j in JUNTAS]
        self.ok = True
        if verboso:
            print('[espejo] publicando en /%s/J{1,2,3}/cmd_pos' % MODELO)
            print('         Gazebo seguira al robot real.')

    def publicar(self, q_rad):
        """q_rad: las tres posiciones MEDIDAS, en radianes."""
        if not self.ok:
            return
        for p, v in zip(self.pubs, q_rad):
            m = self._Float64()
            m.data = float(v)
            p.publish(m)

    def cerrar(self):
        if self.nodo is not None:
            self.nodo.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
