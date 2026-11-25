#!/usr/bin/env python3
# Envia posiciones a RViz (/joint_states) o a Gazebo (cmd_pos).
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JUNTAS = ('J1', 'J2', 'J3')
MODELO = 'planta_brazo'


class EnlaceJuntas:
    def __init__(self, nodo, modo='rviz', juntas=JUNTAS, modelo=MODELO):
        self.nodo = nodo
        self.modo = 'gazebo' if str(modo).lower() == 'gazebo' else 'rviz'
        self.juntas = list(juntas)
        self.consigna = [0.0] * len(self.juntas)
        self._medida = [0.0] * len(self.juntas)
        self.hay_medida = False

        if self.modo == 'gazebo':
            self.pubs = [nodo.create_publisher(Float64, f'/{modelo}/{j}/cmd_pos', 10)
                         for j in self.juntas]
            nodo.create_subscription(JointState, 'joint_states', self._rx, 10)
        else:
            self.pub = nodo.create_publisher(JointState, 'joint_states', 10)

    def enviar(self, q):
        self.consigna = list(q)
        if self.modo == 'gazebo':
            for p, v in zip(self.pubs, self.consigna):
                m = Float64()
                m.data = float(v)
                p.publish(m)
        else:
            m = JointState()
            m.header.stamp = self.nodo.get_clock().now().to_msg()
            m.name = self.juntas
            m.position = [float(v) for v in self.consigna]
            self.pub.publish(m)
            self._medida = list(self.consigna)
            self.hay_medida = True

    def _rx(self, msg):
        for i, j in enumerate(self.juntas):
            if j in msg.name:
                self._medida[i] = msg.position[msg.name.index(j)]
        self.hay_medida = True

    @property
    def real(self):
        return list(self._medida)

    @property
    def error(self):
        return [c - m for c, m in zip(self.consigna, self._medida)]
