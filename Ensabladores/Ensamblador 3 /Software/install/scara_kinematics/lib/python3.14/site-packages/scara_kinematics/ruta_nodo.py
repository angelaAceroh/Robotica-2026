"""Nodo `ruta_nodo`: manda a Gazebo (o al brazo) las rutas de `ruta.py`.

    ros2 launch scara_kinematics ruta.launch.py                     # perforado en gz
    ros2 launch scara_kinematics ruta.launch.py ruta:=contorno
    ros2 launch scara_kinematics ruta.launch.py registro:=~/ruta.csv

`ruta.py` calcula la trayectoria entera como la guia (feed-forward + Jacobiano
inverso, integrado sobre el modelo) y este nodo la reproduce a 1/dt Hz. No
cierra el lazo con lo que mide el brazo: eso ya lo hace el control de posicion
de cada junta (el de gz o el PID de la ESP32), y un segundo lazo por encima,
con /joint_states llegando tarde, se pelearia con el. Lo que mide lo usa para
decir, al acabar, cuanto se separo la punta REAL de la deseada.

Es el mismo enganche que `tray` y no toca ni a `tray` ni a los seis nodos:

    destino:=gz     [ruta_nodo] --Float64-->  /scara_urdf/cmd/<junta> --> gz
    destino:=placa  [ruta_nodo] --JointState--> scara/hw/cmd --> la aduana

Con destino:=placa no arranca solo y no manda nada mientras scara/hw/ok sea
false (placa viva, habilitada y con el cero hecho), igual que `tray`.

Ordenes (std_msgs/String en scara/ruta/orden):

    perforado | contorno  [perfil=trapecio] [codo=down]    lanza esa ruta
    parar                                                 corta y se queda ahi

Salidas:
    <cmd_prefix><junta>  std_msgs/Float64           la consigna (destino gz)
    scara/hw/cmd         sensor_msgs/JointState     la consigna (destino placa)
    scara/ruta/plan      nav_msgs/Path              la trayectoria calculada
    scara/ruta/real      nav_msgs/Path              lo que hizo el brazo
    scara/ruta/via       visualization_msgs/Marker  los puntos de trabajo
    scara/ruta/estado    std_msgs/String            fase, tramo y porcentaje
"""
from __future__ import annotations

import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker

from scara_kinematics import ruta as R
from scara_kinematics import trayectoria as T
from scara_kinematics.model import IKError, declare_model_params


class _Quieto:
    """Mantener una pose hasta que el brazo llegue (o se acabe el plazo)."""

    def __init__(self, q, plazo: float):
        self.q, self.duracion = list(q), plazo

    def muestra(self, t):
        return self.q, [0.0] * len(self.q)


class RutaNodo(Node):

    def __init__(self):
        super().__init__('ruta')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        self.juntas = list(self.model.joint_names)[:3]

        self.base_frame = self.declare_parameter('base_frame', 'world').value
        self.cmd_prefix = self.declare_parameter('cmd_prefix', '/scara_urdf/cmd/').value
        self.dt = float(self.declare_parameter('dt', 0.02).value)
        self.destino = str(self.declare_parameter('destino', 'gz').value).lower()
        if self.destino not in ('gz', 'placa'):
            raise ValueError(f"destino tiene que ser gz o placa: {self.destino!r}")
        self.a_placa = self.destino == 'placa'
        # Los mismos techos que `tray`: en gz, el max_vel del YAML por lo que el
        # PID de la simulacion sigue de verdad; en la placa, lo medido.
        self.factor_vel = float(self.declare_parameter('factor_vel', 0.35).value)
        self.max_vel_hw = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:3]
        self.ruta = str(self.declare_parameter('ruta', 'perforado').value)
        self.perfil = str(self.declare_parameter('perfil', 'trapecio').value)
        self.codo = str(self.declare_parameter('codo', 'down').value)
        self.vel = {k: float(self.declare_parameter(f'v_{k}', v).value)
                    for k, v in R.VELOCIDADES.items()}
        self.espera = float(self.declare_parameter('espera', 6.0).value)
        if self.a_placa:
            self.espera = 0.0          # es para que gz cargue el robot
        # Antes de la ruta, el brazo tiene que HABER LLEGADO al primer punto, no
        # solo haberlo recibido: en gz el husillo se quedaba 20 mm atras al
        # acabar la aproximacion y lo arrastraba dos tramos. Tolerancia por
        # junta (rad, rad, m) y cuanto se le espera como mucho.
        self.llegada = [float(v) for v in self.declare_parameter(
            'llegada', [0.005, 0.005, 0.0005]).value]
        self.llegada_max = float(self.declare_parameter('llegada_max', 15.0).value)
        self.lazo = bool(self.declare_parameter('lazo', False).value)
        registro = str(self.declare_parameter('registro', '').value)

        # --- salidas ---
        self.cmd = ({} if self.a_placa else
                    {j: self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                     for j in self.juntas})
        self.hw_pub = (self.create_publisher(JointState, 'scara/hw/cmd', 10)
                       if self.a_placa else None)
        self.plan_pub = self.create_publisher(Path, 'scara/ruta/plan', 10)
        self.real_pub = self.create_publisher(Path, 'scara/ruta/real', 10)
        self.via_pub = self.create_publisher(Marker, 'scara/ruta/via', 10)
        self.estado_pub = self.create_publisher(String, 'scara/ruta/estado', 10)

        # --- entradas ---
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.create_subscription(String, 'scara/ruta/orden', self.on_orden, 10)
        self.hw_ok = False
        if self.a_placa:
            self.create_subscription(Bool, 'scara/hw/ok', self.on_hw_ok, 10)

        # --- estado ---
        self.q_real: list[float] | None = None
        self.q_cmd: list[float] | None = None
        # ('aproximacion', Trayectoria), ('llegada', _Quieto), ('ruta', Plan)
        self.fases: list[tuple[str, object]] = []
        self.fase: tuple[str, object] | None = None
        self.nombre = ''
        self.k = 0
        self.cola: list[str] = []
        self.errores: dict[str, list[float]] = {}
        self.real = Path()
        self.real.header.frame_id = self.base_frame
        self.arranque = None

        self.csv = None
        if registro:
            ruta_csv = os.path.expanduser(registro)
            self.csv = csv.writer(open(ruta_csv, 'w', newline=''))
            self.csv.writerow(
                ['t', 'fase', 'tramo', 'tipo', 'q1_ref', 'q2_ref', 'd3_ref_mm',
                 'q1_real', 'q2_real', 'd3_real_mm', 'xd_mm', 'yd_mm', 'zd_mm',
                 'x_real_mm', 'y_real_mm', 'z_real_mm', 'error_mm'])
            self.get_logger().info(f'anotando en {ruta_csv}')

        self.create_timer(self.dt, self.tic)
        self.get_logger().info(
            f'ruta | destino={self.destino} | perfil={self.perfil} codo={self.codo} | '
            f'velocidades {self.vel} mm/s | ordenes por scara/ruta/orden')
        if self.a_placa:
            self.get_logger().warning(
                'destino=placa: las consignas van al BRAZO REAL. No arranco solo: '
                'cuando scara/hw/ok sea true, manda la ruta por scara/ruta/orden')
        elif self.ruta:
            self.cola.append(self.ruta)
            self.get_logger().info(
                f'{self.ruta}: empieza en {self.espera:.0f} s, cuando gz haya cargado el robot')

    # ==================================================================== entrada
    def on_js(self, msg: JointState):
        donde = {n: i for i, n in enumerate(msg.name)}
        if any(j not in donde for j in self.juntas):
            return
        self.q_real = [msg.position[donde[j]] for j in self.juntas]

    def on_hw_ok(self, msg: Bool):
        if self.hw_ok and not msg.data and self.fase is not None:
            self.fase = None
            self.fases.clear()
            self.get_logger().error(
                'scara/hw/ok se ha caido a mitad de la ruta: corto y me quedo donde '
                'estoy. Mira la placa, la potencia y el cero')
        self.hw_ok = bool(msg.data)

    def on_orden(self, msg: String):
        texto = msg.data.strip()
        if not texto:
            return
        if texto.split()[0].lower() == 'parar':
            self.cola.clear()
            self.fases.clear()
            self.fase = None
            self.get_logger().warning('parado; me quedo donde estoy')
            return
        if self.a_placa and not self.hw_ok:
            # Se rechaza aqui y no se guarda: una orden mandada con la placa sin
            # preparar no puede ponerse a mover el brazo sola cuando lo este.
            self.get_logger().error(
                f'«{texto}» no se manda: scara/hw/ok esta en false. La placa tiene que '
                f'estar conectada, habilitada y con el cero hecho')
            return
        self.cola.append(texto)

    # ================================================================ planificar
    def _techo(self) -> list[float]:
        if self.a_placa:
            return list(self.max_vel_hw)
        return [v * self.factor_vel for v in self.model.max_vel[:3]]

    def _lanzar(self, texto: str) -> bool:
        if self.a_placa and not self.hw_ok:
            self.get_logger().error(
                f'«{texto}» no se manda: scara/hw/ok esta en false. La placa tiene que '
                f'estar conectada, habilitada y con el cero hecho')
            return False
        q0 = self.q_cmd or self.q_real
        if q0 is None:
            self.get_logger().warning('todavia no se donde esta el brazo: espero a '
                                      '/joint_states', throttle_duration_sec=5.0)
            return False
        palabras = texto.split()
        nombre = palabras[0].lower()
        op = dict(p.split('=', 1) for p in palabras[1:] if '=' in p)
        if nombre not in R.RUTAS:
            self.get_logger().error(f'no conozco la ruta {nombre!r}: hay {", ".join(R.RUTAS)}')
            return False
        techo = self._techo()
        try:
            plan = R.planificar(
                R.RUTAS[nombre], self.robot, op.get('perfil', self.perfil),
                op.get('codo', self.codo), self.vel,
                qdot_max=[techo[0], techo[1], techo[2] * 1e3])
        except (ValueError, R.RutaError) as e:
            self.get_logger().error(f'la ruta {nombre} no sale: {e}')
            return False

        # El brazo no esta en el primer punto de la ruta: se le lleva en
        # articular, como hace `tray` (no resuelve IK, no puede cambiar de rama).
        self.fases = []
        q_ini = R.a_ros(plan.q[0])
        salto = max(abs(a - b) for a, b in zip(q0, q_ini))
        if salto > 0.005:
            # Con el techo del hardware tambien en gz: el husillo real no pasa de
            # 8 mm/s, y el de gz, pidiendole mas, se queda atras.
            techo_ap = [min(a, b) for a, b in zip(techo, self.max_vel_hw)]
            try:
                ap = T.planificar(T.CaminoJunta(q0, q_ini), T.perfil('quintico'),
                                  q0, techo_ap)
            except (ValueError, IKError) as e:
                self.get_logger().error(f'no llego al principio de la ruta: {e}')
                return False
            self.fases.append(('aproximacion', ap))
            self.fases.append(('llegada', _Quieto(q_ini, self.llegada_max)))
            self.get_logger().info(f'aproximacion en articular: {ap.duracion:.2f} s')
        self.fases.append(('ruta', plan))

        pico = np.max(np.abs(plan.qd), axis=0)
        self.get_logger().info(
            f'{nombre}: {len(plan.tramos)} tramos, {plan.duracion:.2f} s, perfil '
            f'{plan.perfil}, codo {plan.codo} | pico {pico[0]:.3f} {pico[1]:.3f} rad/s, '
            f'husillo {pico[2]:.2f} mm/s | error del modelo {plan.err.max() * 1e3:.1f} um')
        self.nombre = nombre
        self.errores = {}
        self._dibujar(plan, nombre)
        return True

    # ==================================================================== el reloj
    def tic(self):
        ahora = self.get_clock().now()
        if self.arranque is None:
            self.arranque = ahora
            return
        if self.fase is None:
            if (ahora - self.arranque).nanoseconds * 1e-9 < self.espera:
                return
            while not self.fases and self.cola:
                self._lanzar(self.cola.pop(0))
            if not self.fases:
                self._estado('en espera')
                return
            self.fase = self.fases.pop(0)
            self.k = 0

        que, tr = self.fase
        t = self.k * self.dt
        self.k += 1
        if que in ('aproximacion', 'llegada'):
            q, qd = tr.muestra(t)
            pd, tramo = None, None
            if que == 'llegada' and self.q_real is not None:
                falta = [abs(a - b) for a, b in zip(q, self.q_real)]
                if all(f <= tol for f, tol in zip(falta, self.llegada)):
                    self.get_logger().info(f'en el primer punto tras {t:.2f} s de espera')
                    tr.duracion = t
                elif t >= tr.duracion:
                    self.get_logger().warning(
                        f'tras {t:.0f} s el brazo sigue a {falta[0]:.4f} rad, '
                        f'{falta[1]:.4f} rad, {falta[2] * 1e3:.2f} mm del primer punto: '
                        f'empiezo igual')
        else:
            qm, qdm, pdm, i = tr.en(t)
            q, qd, pd = R.a_ros(qm), R.a_ros(qdm), pdm
            tramo = tr.tramos[i] if i >= 0 else None
        self._mandar(q, qd)
        self._medir(t, que, tramo, q, pd)
        dur = tr.duracion
        paso = f' tramo {tramo.n} {tramo.tipo}' if tramo else ''
        frac = min(t / dur, 1.0) if dur > 0.0 else 1.0     # llegada puede durar 0
        self._estado(f'{self.nombre} {que}{paso} {frac * 100:3.0f}% '
                     f'({t:.1f}/{dur:.1f} s)')

        if t >= dur:
            self.fase = None
            if que == 'ruta':
                self._resumen()
                if self.lazo and not self.cola and not self.fases:
                    self.cola.append(self.nombre)

    def _mandar(self, q: list[float], qd: list[float]):
        for j, v in zip(self.juntas, q):
            if j in self.cmd:
                self.cmd[j].publish(Float64(data=float(v)))
        if self.hw_pub is not None:
            hw = JointState()
            hw.header.stamp = self.get_clock().now().to_msg()
            hw.name = list(self.juntas)
            hw.position = [float(v) for v in q]
            hw.velocity = [float(v) for v in qd]
            self.hw_pub.publish(hw)
        self.q_cmd = list(q)

    # ==================================================================== medir
    def _mm(self, q):
        return [q[0], q[1], q[2] * 1e3]

    def _medir(self, t, que, tramo, q, pd):
        """Punta real (FK de lo que mide gz o el brazo) frente a la deseada."""
        if self.q_real is None:
            return
        p_real = R.fk_scara_rrp(self._mm(self.q_real), self.robot)
        err = float('nan')
        if pd is not None:
            err = float(np.linalg.norm(pd - p_real))
            self.errores.setdefault(tramo.tipo if tramo else 'inicio', []).append(err)
        if self.csv is not None:
            pdv = pd if pd is not None else [float('nan')] * 3
            self.csv.writerow(
                [f'{t:.3f}', que, tramo.n if tramo else '', tramo.tipo if tramo else '']
                + [f'{v:.6f}' for v in self._mm(q)] + [f'{v:.6f}' for v in self._mm(self.q_real)]
                + [f'{v:.3f}' for v in pdv] + [f'{v:.3f}' for v in p_real] + [f'{err:.4f}'])

    def _resumen(self):
        todos = [e for v in self.errores.values() for e in v]
        if not todos:
            return
        a = np.array(todos)
        self.get_logger().info(
            f'{self.nombre} terminada | punta medida frente a la deseada: medio '
            f'{a.mean():.2f} mm, maximo {a.max():.2f} mm, RMS {math.sqrt(np.mean(a ** 2)):.2f} '
            f'mm ({len(a)} muestras)')
        for tipo, v in self.errores.items():
            v = np.array(v)
            self.get_logger().info(f'   {tipo:<15s} medio {v.mean():6.2f} mm   '
                                   f'maximo {v.max():6.2f} mm   ({len(v)} muestras)')

    # ==================================================================== lo que se ve
    def _pose(self, p_mm, stamp) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) * 1e-3 for v in p_mm)
        ps.pose.orientation.w = 1.0
        return ps

    def _dibujar(self, plan: R.Plan, nombre: str):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.base_frame
        paso = max(1, len(plan.t) // 1500)
        path.poses = [self._pose(p, stamp) for p in plan.p[::paso]]
        self.plan_pub.publish(path)
        self.real.poses.clear()

        mk = Marker()
        mk.header = path.header
        mk.ns = 'ruta'
        mk.id = 0
        mk.type = Marker.SPHERE_LIST
        mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = mk.scale.z = 0.010
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.92, 0.41, 0.20, 0.95
        mk.pose.orientation.w = 1.0
        mk.points = [Point(x=float(p[0]) * 1e-3, y=float(p[1]) * 1e-3, z=float(p[2]) * 1e-3)
                     for p in R.PUNTOS.get(nombre, {}).values()]
        self.via_pub.publish(mk)

    def _estado(self, texto: str):
        self.estado_pub.publish(String(data=texto))
        if self.q_real is not None and self.fase is not None:
            self.real.header.stamp = self.get_clock().now().to_msg()
            p = R.fk_scara_rrp(self._mm(self.q_real), self.robot)
            self.real.poses.append(self._pose(p, self.real.header.stamp))
            if len(self.real.poses) > 5000:
                del self.real.poses[:len(self.real.poses) - 5000]
            self.real_pub.publish(self.real)


def main(args=None):
    rclpy.init(args=args)
    node = RutaNodo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Lo mismo que en esp.py: con Ctrl+C rclpy cierra el contexto con el
        # ejecutor a medias y lanza RCLError. No es un fallo, es el final.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
