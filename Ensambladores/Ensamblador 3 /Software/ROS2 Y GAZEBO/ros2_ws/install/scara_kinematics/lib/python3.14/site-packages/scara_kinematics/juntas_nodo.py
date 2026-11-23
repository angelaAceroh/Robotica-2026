"""Nodo `juntas_nodo` (se llama `control`, como control_nodo): el brazo de verdad,
punto a punto POR JUNTAS (juntas.py). Lo lanza traycho_juntas.launch.py.

    ros2 launch scara_kinematics traycho_juntas.launch.py                 # el brazo
    ros2 launch scara_kinematics traycho_juntas.launch.py simular:=true   # sin el brazo

Ocupa el sitio de control_nodo, que no se ha tocado: las mismas ordenes por
scara/control/orden, los mismos mensajes («ir: N s», «ir terminada | ...
medio X mm, maximo Y mm») y las mismas salidas, asi que la ventana y el boton
traycho funcionan igual. Lo que cambia es como va:

    control_nodo   recta, corrigiendo todo el camino: qd = J^-1 (xd_dot + K e)
    juntas_nodo    cada junta a su velocidad fija (14 RPM, 30 RPM, 60 mm/min),
                   sin corregir por el camino; al llegar, mide y corrige

Ordenes (std_msgs/String en scara/control/orden):

    ir X Y Z      al punto (mm). Lo de detras (v=, perfil=) no cuenta: las
                  velocidades son las de cada junta. Una Z que Teodoro no
                  alcanza se cambia por la mas cercana que si (y lo dice)
    home          a P1, que es el HOME (24/10: «pon bien el home que es P1»):
                  (0, -362) arriba, con Simon en -27.5 (juntas.SIMON_HOME). Hasta
                  el 24/10 por la tarde, (0, -273): el antebrazo contra la base
    parar         se queda donde esta

Entradas:  joint_states, scara/placa/seguimiento y scara/placa/estado (de
           puente_placa: si suelta los motores, se para lo que estuviera en marcha)
Salidas:   <cmd_vel_prefix><junta>         std_msgs/Float64   (rad/s, rad/s, m/s)
           scara/control/estado            std_msgs/String
           scara/control/deseado           nav_msgs/Path      por donde va a ir la punta
           scara/control/datos/punta_real  Vector3 (mm)
           scara/control/datos/error       Vector3 (mm)       lo que falta hasta el punto
           scara/control/datos/saturacion  Float64            siempre 0
"""
from __future__ import annotations

import math
import signal
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from scara_kinematics import alcance as A
from scara_kinematics import juntas as J
from scara_kinematics import ruta as R
from scara_kinematics.model import declare_model_params


class JuntasNodo(Node):

    def __init__(self):
        super().__init__('control')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        self.juntas = list(self.model.joint_names)[:3]

        p = self.declare_parameter
        self.base_frame = str(p('base_frame', 'world').value)
        self.cmd_prefix = str(p('cmd_vel_prefix', '/scara_urdf/placa/cmd_vel/').value)
        self.dt = float(p('dt', 0.02).value)
        self.vigilante = float(p('vigilante', 0.25).value)
        # De junta a motor y las velocidades: config/micropython.yaml (lo pasa el launch)
        d = A.Calibracion()
        defectos = {'limites_motor': [x for par in d.limites for x in par],
                    'reduccion': d.reduccion, 'signo': d.signo, 'cero': d.cero,
                    'paso_husillo_mm': d.paso_husillo_mm, 'velocidades': d.velocidades}
        self.cal = A.Calibracion.desde_parametros(
            lambda nombre, _: p(nombre, defectos[nombre]).value)
        self.ir = J.Juntas(self.robot, J.velocidades(self.cal), self.dt)

        self.cmd = [self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                    for j in self.juntas]
        self.estado_pub = self.create_publisher(String, 'scara/control/estado', 10)
        self.deseado_pub = self.create_publisher(Path, 'scara/control/deseado', 10)
        self.datos = {k: self.create_publisher(Vector3, f'scara/control/datos/{k}', 10)
                      for k in ('punta_real', 'error')}
        self.sat_pub = self.create_publisher(Float64, 'scara/control/datos/saturacion', 10)

        self.create_subscription(JointState, 'joint_states', self.on_js, 1)
        self.create_subscription(String, 'scara/control/orden', self.on_orden, 10)
        self.create_subscription(Vector3, 'scara/placa/seguimiento', self.on_seg, 10)
        self.create_subscription(String, 'scara/placa/estado', self.on_placa, 10)

        self.q: np.ndarray | None = None        # grados, grados, mm
        self.t_q = 0.0
        self.seg: np.ndarray | None = None      # consigna - posicion, en junta
        self.t_seg = None
        self.placa = ''
        self.ordenes: list[str] = []
        self.tics = 0

        self.create_timer(self.dt, self.tic)
        v = self.ir.vel
        self.get_logger().info(
            f'por juntas | Alvin {v[0]:g} °/s, Simon {v[1]:g} °/s, Teodoro {v[2]:g} mm/s '
            f'(velocidades {self.cal.velocidades} de config/micropython.yaml) | corrige al '
            f'llegar hasta {J.TOL[0]:g}°, {J.TOL[1]:g}°, {J.TOL[2]:g} mm | '
            f'ordenes por scara/control/orden: ir X Y Z, home, parar')

    # ==================================================================== entrada
    def on_js(self, msg: JointState):
        donde = {n: i for i, n in enumerate(msg.name)}
        if any(j not in donde for j in self.juntas):
            return
        q = [msg.position[donde[j]] for j in self.juntas]
        self.q = J.a_grados([q[0], q[1], q[2] * 1e3])
        self.t_q = time.monotonic()

    def on_seg(self, msg: Vector3):
        self.seg = J.seguimiento_a_junta((msg.x, msg.y, msg.z), self.cal)
        self.t_seg = time.monotonic()

    def on_placa(self, msg: String):
        self.placa = msg.data
        if self.ir.activo() and not msg.data.startswith('ACTIVA'):
            self.ir.parar('la placa ha soltado los motores')

    def on_orden(self, msg: String):
        if msg.data.strip():
            self.ordenes.append(msg.data.strip())

    # ==================================================================== ordenes
    def _orden(self, texto: str, ahora: float):
        palabras = texto.replace(',', ' ').split()
        nombre = palabras[0].lower()
        log = self.get_logger()
        try:
            nums = [float(x) for x in palabras[1:] if '=' not in x]
        except ValueError:
            log.error(f'«{texto}»: los numeros no se entienden')
            return
        if nombre == 'parar':
            if self.ir.activo():
                self.ir.parar()
            else:
                log.warning('parado: quieto donde estoy')
            return
        if nombre not in ('ir', 'home'):
            log.error(f'«{texto}» no sale: por juntas solo hay ir X Y Z, home y parar')
            return
        try:
            if self.q is None:
                raise R.RutaError('no llegan los encoders (/joint_states)')
            if not self.placa.startswith('ACTIVA'):
                raise R.RutaError('los motores estan sueltos («Activar motores»)')
            if nombre == 'home':
                nums = [float(v) for v in J.puntos_mm()[0]]          # P1
            elif len(nums) != 3:
                raise R.RutaError(f'hacen falta tres numeros (mm) y hay {len(nums)}')
            # Una z que Teodoro no alcanza (la 150 de la ventana, por debajo
            # del soporte) se lleva a la mas cercana que si: arriba es seguro
            z = sorted(self.robot.z_ref + self.robot.z_dir * d for d in self.robot.lim[2])
            zz = min(max(nums[2], z[0] + 0.01), z[1] - 0.01)
            if abs(zz - nums[2]) > 0.01 and nombre != 'home':
                log.warning(f'«{texto}»: Teodoro solo va de z {z[0]:.1f} a {z[1]:.1f} mm '
                            f'(el soporte): voy a z {zz:.1f}')
            nums[2] = zz
            self.ir.ir(nums, self.q, ahora, nombre)
        except R.RutaError as e:
            log.error(f'«{texto}» no sale: {e}')
            return
        self._deseado(self.ir.camino)

    # ==================================================================== el reloj
    def tic(self):
        ahora = time.monotonic()
        if self.q is None or ahora - self.t_q > self.vigilante:
            if self.ir.activo():
                self.ir.parar('no llegan los encoders (/joint_states)')
            self._avisos()
            self._mandar(np.zeros(3))
            self._estado('sin /joint_states: juntas paradas')
            return
        while self.ordenes:
            self._orden(self.ordenes.pop(0), ahora)
        v = self.ir.paso(ahora, self.q, self.seg, self.t_seg)
        self._avisos()
        self._mandar(v)
        self._publicar(v)

    def _avisos(self):
        # Una llamada por nivel, cada una en su linea: rclpy no deja que el mismo
        # sitio del codigo escriba con niveles distintos (ValueError, y el nodo
        # se cae: le paso a la primera que salio un warning)
        log = self.get_logger()
        for nivel, texto in self.ir.avisos:
            if nivel >= 40:
                log.error(texto)
            elif nivel >= 30:
                log.warning(texto)
            else:
                log.info(texto)
        self.ir.avisos.clear()

    def _mandar(self, v):
        r = J.a_rad(v)
        for pub, x, f in zip(self.cmd, r, (1.0, 1.0, 1e-3)):
            pub.publish(Float64(data=float(x) * f))

    def parar_juntas(self):
        """Al cerrar, ceros; y se espera a que salgan (como control_nodo)."""
        self._mandar(np.zeros(3))
        for pub in self.cmd:
            pub.wait_for_all_acked(Duration(seconds=0.3))

    # ==================================================================== lo que se ve
    def _publicar(self, v):
        p = R.fk_scara_rrp(J.a_rad(self.q), J.sin_topes(self.robot))
        e = self.ir.destino - p if self.ir.destino is not None else np.zeros(3)
        self.datos['punta_real'].publish(Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        self.datos['error'].publish(Vector3(x=float(e[0]), y=float(e[1]), z=float(e[2])))
        self.sat_pub.publish(Float64(data=0.0))
        self.tics += 1
        if self.tics % max(1, round(0.2 / self.dt)) == 0:          # 5 Hz
            ir = self.ir
            if ir.activo():
                txt = (f'{ir.nombre} por juntas, {ir.fase}'
                       + (f' (correccion {ir.n_corr})' if ir.n_corr else '')
                       + f' | falta {math.hypot(*e[:2]):.1f} mm en planta, {e[2]:+.1f} en z')
            else:
                txt = f'quieto en [{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}] mm'
                if ir.destino is not None:
                    txt += f' | error {np.linalg.norm(e):.2f} mm'
            self._estado(txt)

    def _deseado(self, puntos_mm):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.base_frame
        for pt in puntos_mm:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
                float(c) * 1e-3 for c in pt)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.deseado_pub.publish(path)

    def _estado(self, texto: str):
        self.estado_pub.publish(String(data=texto))


def main(args=None):
    # Como control_nodo: sin los manejadores de rclpy, para que al cerrar aun
    # salgan los ceros
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    fin = threading.Event()
    for sen in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sen, lambda *_: fin.set())
    node = JuntasNodo()
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(node)
    try:
        while rclpy.ok() and not fin.is_set():
            ejecutor.spin_once(timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.parar_juntas()
        ejecutor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
