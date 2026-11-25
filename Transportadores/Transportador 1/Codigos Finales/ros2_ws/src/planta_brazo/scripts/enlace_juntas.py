#!/usr/bin/env python3
"""
Capa fina que oculta la diferencia entre los dos modos de simulacion, para que
el panel de deslizadores y la trayectoria de demostracion sean el mismo codigo
en RViz y en Gazebo.

  modo = 'rviz'     el nodo ES la fuente de verdad: publica directamente la
                    posicion articular en /joint_states y robot_state_publisher
                    la convierte en TF.  No hay fisica: lo que se manda es lo
                    que se ve.

  modo = 'gazebo'   el nodo solo manda CONSIGNAS: un std_msgs/Float64 por junta
                    en /planta_brazo/J{n}/cmd_pos.  El puente ros_gz_bridge las
                    pasa a gz.msgs.Double y el JointPositionController de gz-sim
                    aplica el par necesario.  La posicion REAL vuelve por
                    /joint_states publicada por la fisica, y es la que hay que
                    mostrar y la que sigue la traza.

En ambos casos:
    enlace.enviar(q)   ->  manda la consigna (lista de 3 angulos en rad)
    enlace.real        ->  ultima posicion conocida (rad); en modo rviz coincide
                           con la consigna, en gazebo es la medida de Gazebo
    enlace.hay_medida  ->  True si Gazebo ya publico al menos un estado
"""
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
            # la fisica devuelve el estado real por aqui (lo remapea el launch)
            nodo.create_subscription(JointState, 'joint_states', self._rx, 10)
        else:
            self.pub = nodo.create_publisher(JointState, 'joint_states', 10)

    # ------------------------------------------------------------------ envio
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

    # ----------------------------------------------------------------- lectura
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
        """Consigna - medida, en rad."""
        return [c - m for c, m in zip(self.consigna, self._medida)]
