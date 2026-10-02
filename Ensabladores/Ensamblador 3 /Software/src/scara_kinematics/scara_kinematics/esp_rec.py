"""Nodo 6: esp_rec. El final de la cadena: recibe lo que saca la cinematica directa.

    ros2 run scara_kinematics esp_rec --ros-args --params-file .../scara_urdf3.yaml
    ros2 run scara_kinematics esp_rec --ros-args -p registro:=~/scara.csv
    ros2 run scara_kinematics esp_rec --ros-args -p reenviar:=true

Cierra el circulo: la aduana reparte lo que viene de la placa, los tres motores
lo cuidan, cc resuelve donde esta la punta y esto lo recoge.

    [cc] --scara/ee_pose--> ESTE NODO --+--> scara/esp_rec/linea  (texto legible)
         --joint_states---->            +--> un CSV, si se le da 'registro'
                                        +--> scara/hw/cmd -> aduana -> ESP32,
                                             solo con reenviar:=true

La punta y las juntas llegan por dos topics distintos, pero cc las saca de la
misma muestra con el mismo sello: aqui se emparejan por ese sello, para que cada
fila del CSV sean la punta y las juntas del mismo instante.

No abre el puerto serie: /dev/ttyUSB0 solo lo puede tener abierto un proceso, y
ese es la aduana. Si hay que mandarle algo a la placa, se le pasa a ella y ella
lo escribe.

Sobre reenviar: el firmware de esta ESP32 no tiene ninguna orden para "la punta
esta en (x, y, z)" -su protocolo es T/E/Z/R/F/A y nada mas-, asi que lo unico
que tiene sentido devolverle es una T, la consigna de posicion. Como cc mide lo
que ya hay, esa T es un "quedate donde estas": util para congelar el brazo, y
una forma rapida de ver el lazo entero cerrandose. Va apagado por defecto
porque mandar consignas a un brazo con potencia no es algo que deba pasar solo
por arrancar un nodo.

Y va por scara/hw/cmd, no escrita a mano por scara/hw/raw: la aduana ya manda la
T como un flujo a 50 Hz con la ultima consigna que le llego, y una segunda T
colada por raw a 2 Hz haria que la placa recibiera dos objetivos alternos. Asi
hay una sola T, la de la aduana, y esto solo cambia a donde apunta.
"""
from __future__ import annotations

import math
import os
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import POR_JUNTA, formatear


def yaw_de(o) -> float:
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class EspRec(Node):

    def __init__(self):
        super().__init__('esp_rec')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)

        self.reenviar = bool(self.declare_parameter('reenviar', False).value)
        self.hz = float(self.declare_parameter('hz', 2.0).value)
        ruta = str(self.declare_parameter('registro', '').value)

        self.linea_pub = self.create_publisher(String, 'scara/esp_rec/linea', 10)
        self.cmd_pub = self.create_publisher(JointState, 'scara/hw/cmd', 10)
        self.create_subscription(PoseStamped, 'scara/ee_pose', self.on_pose, 10)
        self.create_subscription(JointState, 'joint_states', self.on_juntas, 10)

        self.pose: PoseStamped | None = None
        self.q: list[float] | None = None
        # Las juntas de las ultimas muestras, por sello, para casar cada pose
        # con las suyas. Dos segundos a 25 Hz.
        self.recientes: deque[tuple[int, list[float]]] = deque(maxlen=50)
        self.n_pose = 0
        self.t_ultimo = 0.0

        self.csv = None
        if ruta:
            ruta = os.path.expanduser(ruta)
            try:
                self.csv = open(ruta, 'w', buffering=1)
                self.csv.write('t,x,y,z,yaw,' + ','.join(self.juntas) + '\n')
                self.get_logger().info(f'anotando en {ruta}')
            except OSError as exc:
                self.get_logger().error(f'no puedo escribir en {ruta}: {exc}')

        self.create_timer(1.0 / self.hz, self.resumir)
        self.get_logger().info(
            'esperando a cc por scara/ee_pose'
            + (' | reenviando la consigna a la placa por scara/hw/cmd' if self.reenviar
               else ' | sin reenviar a la placa (reenviar:=true para hacerlo)'))

    # ------------------------------------------------------------------ entrada
    def on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.n_pose += 1
        self.t_ultimo = time.monotonic()

    def on_juntas(self, msg: JointState):
        try:
            self.q = [msg.position[msg.name.index(j)] for j in self.juntas]
        except (ValueError, IndexError):
            return
        self.recientes.append((self.sello(msg.header), self.q))

    @staticmethod
    def sello(cabecera) -> int:
        return cabecera.stamp.sec * 1_000_000_000 + cabecera.stamp.nanosec

    def juntas_de(self, pose: PoseStamped) -> list[float] | None:
        """Las juntas de la misma muestra que esa pose, o None si no estan."""
        s = self.sello(pose.header)
        for sello, q in reversed(self.recientes):
            if sello == s:
                return q
        return None

    # ------------------------------------------------------------------ salida
    def resumir(self):
        if self.pose is None:
            self.get_logger().warning(
                'cc no esta publicando: sin las tres juntas no hay punta que recibir',
                throttle_duration_sec=10.0)
            return

        p = self.pose.pose.position
        yaw = yaw_de(self.pose.pose.orientation)
        # Las de esa misma muestra; si no se encuentran (no deberia pasar, cc
        # saca las dos a la vez), la linea de texto va con las ultimas y el CSV
        # se salta la fila antes que mezclar dos instantes.
        q = self.juntas_de(self.pose)
        q_texto = q if q is not None else self.q
        detalle = ''
        if q_texto is not None:
            detalle = '  |  ' + '  '.join(
                f'{POR_JUNTA[j].nombre if j in POR_JUNTA else j} {formatear(POR_JUNTA[j].clave, v)}'
                if j in POR_JUNTA else f'{j} {v:.3f}'
                for j, v in zip(self.juntas, q_texto))

        texto = (f'TCP  x {p.x * 1000:7.1f}  y {p.y * 1000:7.1f}  z {p.z * 1000:7.1f} mm'
                 f'  yaw {yaw * 57.29578:+6.1f}°{detalle}')
        self.linea_pub.publish(String(data=texto))

        if self.csv is not None and q is not None:
            t = self.pose.header.stamp.sec + self.pose.header.stamp.nanosec * 1e-9
            self.csv.write(f'{t:.3f},{p.x:.6f},{p.y:.6f},{p.z:.6f},{yaw:.6f},'
                           + ','.join(f'{v:.6f}' for v in q) + '\n')

        if self.reenviar and q is not None:
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = list(self.juntas)
            cmd.position = list(q)
            self.cmd_pub.publish(cmd)

    def destroy_node(self):
        if self.csv is not None:
            self.csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspRec()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C rclpy cierra el contexto aunque haya un callback a medias,
        # y ese callback lanza RCLError al publicar: no es un fallo, es el final.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
