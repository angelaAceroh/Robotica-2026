"""Nodo 1: cinematica directa. Recibe las tres juntas y dice donde esta la punta.

    ros2 run scara_kinematics cc --ros-args --params-file .../scara_urdf3.yaml

Es el unico de los seis que recibe y envia. Recibe de Alvin, Simon y Teodoro
-cada uno su pieza de la muestra, ya vigilada- y con las tres vuelve a armar el
vector del brazo, q = [q1, q2, d3]: publica el /joint_states que dibuja el robot
y resuelve la posicion del TCP.

    [alvin]  scara/alvin/junta   --+
    [simon]  scara/simon/junta   --+--> ESTE NODO --+--> joint_states  (TF, RViz, ventana)
    [teodoro] scara/teodoro/junta--+                +--> scara/ee_pose --> [esp_rec]
                                                    +--> scara/ee_path, scara/ee_marker
                                                    +--> Float64 a los mandos de Gazebo

Las tres piezas se juntan POR MUESTRA, no por orden de llegada. La aduana parte
cada linea de la placa en tres mensajes con la misma etiqueta (muestra=k en el
frame_id); aqui se guardan hasta tener las tres de la misma k, y entonces, y
solo entonces, sale un joint_states. Asi cada vector que se publica son tres
medidas del MISMO instante, con el sello de ese instante, y sale una vez por
muestra: ni repetidas ni mezcladas.

Si un motor se calla no se completa ninguna muestra y el brazo se queda dibujado
donde estaba: dibujarlo con la ultima posicion de ese motor seria mezclar dos
instantes y hacer pasar por medida algo que ya no lo es. Por eso lleva cuenta de
cuando llego cada pieza y avisa, con nombre, de la que falta.

Salidas:
    joint_states      sensor_msgs/JointState     el brazo entero (remapeable)
    scara/ee_pose     geometry_msgs/PoseStamped  donde esta el TCP
    scara/ee_path     nav_msgs/Path              su rastro
    scara/ee_marker   visualization_msgs/Marker  para verlo en RViz
    <cmd_prefix><junta>  std_msgs/Float64        solo con publicar_gz:=true
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import POR_JUNTA, leer_etiqueta

# Muestras a medio juntar que se guardan como mucho: un segundo de telemetria.
# Si un motor se ha callado, sus muestras no se completan nunca y sin este techo
# se acumularian sin fin.
MAX_PENDIENTES = 25


class CinematicaDirecta(Node):

    def __init__(self):
        super().__init__('cc')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)

        self.base_frame = self.declare_parameter('base_frame', 'world').value
        self.trail_len = int(self.declare_parameter('trail_len', 500).value)
        self.warn_singular = float(self.declare_parameter('singular_threshold', 5e-4).value)
        # Cuanto puede tardar una junta antes de darla por perdida.
        self.t_junta = float(self.declare_parameter('t_junta', 1.0).value)
        # Espejo en Gazebo: la simulacion sigue al brazo real.
        self.publicar_gz = bool(self.declare_parameter('publicar_gz', False).value)
        self.cmd_prefix = self.declare_parameter('cmd_prefix', '/scara_urdf/cmd/').value

        # Quien aporta cada hueco del vector. Una junta sin motor -'manita' en el
        # YAML de 4 GDL- no va a llegar nunca de la aduana: va fija a 0.
        self.motores = [(i, POR_JUNTA[j].clave) for i, j in enumerate(self.juntas)
                        if j in POR_JUNTA]
        for j in self.juntas:
            if j not in POR_JUNTA:
                self.get_logger().info(f'{j} no tiene motor: la doy por 0')

        # Muestras a medio juntar: {clave de muestra: {indice: (q, v, u)}}.
        self.pendientes: dict[tuple, dict[int, tuple[float, float, float]]] = {}
        self.t_pieza = {clave: None for _i, clave in self.motores}   # monotonic
        self.completas = 0
        self.incompletas = 0
        self.incompletas_dichas = 0
        self.t_aviso = -math.inf
        self.faltaron = {clave: 0 for _i, clave in self.motores}
        # Hasta la primera muestra completa, lo que no se completa es del
        # arranque -un motor se suscribio antes que otro- y no se cuenta.
        self.en_marcha = False

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'scara/ee_pose', 10)
        self.path_pub = self.create_publisher(Path, 'scara/ee_path', 10)
        self.mk_pub = self.create_publisher(Marker, 'scara/ee_marker', 10)

        self.gz_pub = {}
        if self.publicar_gz:
            self.gz_pub = {j: self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                           for j in self.juntas}

        # Una suscripcion por motor. El indice se cierra en el lambda para que
        # cada callback sepa que hueco del vector le toca rellenar.
        for i, clave in self.motores:
            self.create_subscription(
                JointState, f'scara/{clave}/junta',
                lambda msg, i=i, clave=clave: self.on_junta(i, clave, msg), 10)

        self.path = Path()
        self.path.header.frame_id = self.base_frame
        self.aviso_singular = False

        self.create_timer(2.0, self.vigilar)
        self.get_logger().info(
            f'cinematica directa | junta por muestra lo de '
            f'{", ".join(c for _i, c in self.motores)} | '
            f'publica joint_states y scara/ee_pose'
            + (f' | espejo en Gazebo por {self.cmd_prefix}*' if self.publicar_gz else ''))

    # ------------------------------------------------------------------ entrada
    @staticmethod
    def muestra_de(msg: JointState) -> tuple:
        """La clave que comparten las tres piezas de una muestra.

        La etiqueta de la aduana si la trae; si no (la sonda del auditor), el
        sello, que la sonda tambien pone igual en las tres.
        """
        marca = leer_etiqueta(msg.header.frame_id)
        if marca is not None:
            return (0, marca[0])
        return (1, msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def on_junta(self, i: int, clave: str, msg: JointState):
        if len(msg.position) != 1:
            return
        self.t_pieza[clave] = time.monotonic()
        pieza = (msg.position[0],
                 msg.velocity[0] if msg.velocity else 0.0,
                 msg.effort[0] if msg.effort else 0.0)

        k = self.muestra_de(msg)
        fila = self.pendientes.setdefault(k, {})
        fila[i] = pieza
        if len(fila) < len(self.motores):
            if len(self.pendientes) > MAX_PENDIENTES:
                self._descartar(min(self.pendientes))
            return

        # Completa. Las que quedan a medias y son anteriores ya no se van a
        # completar: cada motor entrega sus piezas en orden, asi que si la
        # suya de esa muestra no ha llegado antes que esta, se perdio.
        del self.pendientes[k]
        for vieja in [p for p in self.pendientes if p[0] != k[0] or p[1] < k[1]]:
            self._descartar(vieja)
        self.en_marcha = True
        self.completas += 1
        self.publicar(fila, msg)

    def _descartar(self, k: tuple):
        fila = self.pendientes.pop(k)
        if not self.en_marcha:
            return
        self.incompletas += 1
        for i, clave in self.motores:
            if i not in fila:
                self.faltaron[clave] += 1

    # ------------------------------------------------------------------ salida
    def publicar(self, fila: dict, ultima: JointState):
        n = len(self.juntas)
        q, v, u = [0.0] * n, [0.0] * n, [0.0] * n
        for i, (qi, vi, ui) in fila.items():
            q[i], v[i], u[i] = qi, vi, ui
        # El sello y la etiqueta de la muestra: son los mismos en las tres piezas.
        stamp = ultima.header.stamp

        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = ultima.header.frame_id
        js.name = list(self.juntas)
        js.position = q
        js.velocity = v
        js.effort = u
        self.js_pub.publish(js)

        for j, qj in zip(self.juntas, q):
            if j in self.gz_pub:
                self.gz_pub[j].publish(Float64(data=qj))

        x, y, z, yaw = self.model.fk(q)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.trail_len:
            del self.path.poses[:len(self.path.poses) - self.trail_len]
        self.path_pub.publish(self.path)

        mk = Marker()
        mk.header = pose.header
        mk.ns = 'tcp'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose = pose.pose
        mk.scale.x = mk.scale.y = mk.scale.z = 0.02
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.18, 0.62, 0.36, 0.9
        self.mk_pub.publish(mk)

        # Cerca de la singularidad el brazo esta estirado o plegado del todo: ahi
        # un milimetro de TCP pide muchisimo giro de codo y el lazo se vuelve loco.
        if self.model.manipulability(q) < self.warn_singular:
            if not self.aviso_singular:
                self.aviso_singular = True
                self.get_logger().warning(
                    'el brazo esta en una singularidad (estirado o plegado del todo)')
        else:
            self.aviso_singular = False

    def vigilar(self):
        """Canta la junta que se ha quedado sin dar senales, y las muestras rotas."""
        ahora = time.monotonic()
        for _i, clave in self.motores:
            t = self.t_pieza[clave]
            if t is None:
                self.get_logger().warning(
                    f'sin noticias de {clave}: sin su pieza no se completa ninguna '
                    'muestra, y sin muestra no hay punta',
                    throttle_duration_sec=10.0)
            elif ahora - t > self.t_junta:
                self.get_logger().warning(
                    f'{clave} lleva {ahora - t:.1f} s callado: sin su pieza no se '
                    'completa ninguna muestra y el brazo se queda dibujado donde estaba',
                    throttle_duration_sec=10.0)

        # Cada 10 s como mucho: con un motor muerto no se completa ninguna, y
        # el aviso de arriba ya dice cual es.
        nuevas = self.incompletas - self.incompletas_dichas
        if nuevas and ahora - self.t_aviso >= 10.0:
            self.t_aviso = ahora
            self.incompletas_dichas = self.incompletas
            faltas = ', '.join(f'{c} {n}' for c, n in self.faltaron.items() if n)
            self.get_logger().warning(
                f'{nuevas} muestra(s) incompleta(s) desde el ultimo aviso '
                f'({self.incompletas} de {self.incompletas + self.completas} en total; '
                f'piezas que faltaron: {faltas}). No las dibujo: mezclarian juntas de '
                'instantes distintos')


def main(args=None):
    rclpy.init(args=args)
    node = CinematicaDirecta()
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
