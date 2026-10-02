"""Nodo 7: planificacion de trayectorias. Manda el brazo por un camino, en Gazebo.

    ros2 launch scara_kinematics trayectoria.launch.py demo:=true

Es el primer nodo del paquete que usa la cinematica INVERSA: `model.py` traia
`ik`/`ik_all` escritos desde el principio y ningun nodo los llamaba. Aqui se le
da un punto en cartesiano y salen las tres juntas.

No toca a ninguno de los seis. Se engancha donde ya estaba el hueco:

    [tray] --Float64--> /scara_urdf/cmd/<junta> --> gz JointPositionController
                                                          |
      /joint_states <--- puente ros_gz <--- gz JointStatePublisher
           |
           +--> este nodo dibuja lo que gz hizo de verdad (scara/tray/real)
                y lo compara con lo que le pidio (scara/tray/plan)

Por eso se lanza con `brazo:=false`: sin los seis nodos no hay quien se pelee
por los topics de mando ni por /joint_states, y no se abre el puerto serie.

Como se le habla: un solo topic de texto.

    ros2 topic pub --once /scara/tray/orden std_msgs/String "data: 'recta 0.20 0.18 0.11'"

    junta      q1 q2 d3                  interpolacion articular (sin IK)
    recta      x y z                     linea recta cartesiana
    arco       cx cy r a0 a1 [z]         arco (a0, a1 en GRADOS)
    circulo    cx cy r [z] [vueltas]     circulo completo
    tres       x1 y1 x2 y2 x3 y3 [z]     arco por tres puntos
    poligonal  x1 y1 z1 x2 y2 z2 ...     segmentos rectos por N via
    spline     x1 y1 z1 x2 y2 z2 ...     spline cubica natural por N via
    home                                 vuelve a la pose de reposo
    perfiles                             la misma recta con los cinco perfiles
    caminos                              los cinco caminos con el perfil de ahora
    demo                                 las dos anteriores, seguidas
    parar                                corta y se queda donde este

En cualquier orden se pueden meter, en cualquier sitio:

    perfil=cubico|quintico|septico|trapecio|scurve
    t=3.5          duracion fija en segundos (por defecto la calcula el max_vel)
    codo=up|down|nearest|auto

Salidas:
    <cmd_prefix><junta>  std_msgs/Float64            la consigna, a 1/dt Hz
    scara/tray/plan      nav_msgs/Path               el camino planificado (TCP)
    scara/tray/real      nav_msgs/Path               lo que gz hizo de verdad
    scara/tray/via       visualization_msgs/Marker   los puntos via y el metodo
    scara/tray/estado    std_msgs/String             que corre y por donde va
"""
from __future__ import annotations

import csv
import math
import os

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker

from scara_kinematics.model import IKError, declare_model_params
from scara_kinematics import trayectoria as T

# La visita guiada. Son literalmente ordenes de las que se pueden teclear: el
# demo no tiene un camino de codigo propio, pasa por el mismo parser.
GUIA_PERFILES = [
    'home',
    '# los cinco perfiles sobre la MISMA recta: cambia el tiempo, no la forma',
    'recta 0.20 0.20 0.11 perfil=trapecio',
    'recta 0.30 0.00 0.15 perfil=trapecio',
    'recta 0.20 0.20 0.11 perfil=cubico',
    'recta 0.30 0.00 0.15 perfil=cubico',
    'recta 0.20 0.20 0.11 perfil=quintico',
    'recta 0.30 0.00 0.15 perfil=quintico',
    'recta 0.20 0.20 0.11 perfil=septico',
    'recta 0.30 0.00 0.15 perfil=septico',
    'recta 0.20 0.20 0.11 perfil=scurve',
    'recta 0.30 0.00 0.15 perfil=scurve',
]
GUIA_CAMINOS = [
    'home',
    '# los cinco caminos: cambia la forma, el perfil es el mismo',
    'junta -1.30 2.10 0.100',
    'home',
    'recta 0.20 0.20 0.11',
    'home',
    'tres 0.32 -0.10 0.38 0.00 0.32 0.10 0.15',
    'circulo 0.30 0.00 0.08 0.15 codo=down',
    'poligonal 0.26 -0.09 0.14 0.36 -0.09 0.14 0.36 0.09 0.14 0.26 0.09 0.14 0.26 -0.09 0.14',
    'spline 0.26 -0.09 0.14 0.36 -0.09 0.17 0.36 0.09 0.11 0.26 0.09 0.17 0.26 -0.09 0.14',
    'home',
]


class Planificador(Node):

    def __init__(self):
        super().__init__('tray')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)
        self.n = len(self.juntas)

        self.base_frame = self.declare_parameter('base_frame', 'world').value
        self.cmd_prefix = self.declare_parameter('cmd_prefix', '/scara_urdf/cmd/').value
        self.dt = float(self.declare_parameter('dt', 0.02).value)
        self.perfil = str(self.declare_parameter('perfil', 'quintico').value)
        self.codo = str(self.declare_parameter('codo', 'auto').value)
        self.t_fijo = float(self.declare_parameter('duracion', 0.0).value)
        self.muestras = int(self.declare_parameter('muestras', 400).value)
        # El max_vel del YAML es el techo del MODELO. El JointPositionController
        # de gz, con las ganancias del URDF, no lo sigue: pedirle 1.5 rad/s deja
        # 40 mm de retraso en la punta. Este factor es lo que la simulacion
        # aguanta de verdad; subirlo hace la trayectoria mas rapida y peor.
        self.factor_vel = float(self.declare_parameter('factor_vel', 0.35).value)
        # A donde va la consigna: 'gz' (lo probado), 'placa' (el brazo de la mesa,
        # por la aduana) o 'ambos'. Con 'gz' esto se comporta exactamente igual
        # que antes de existir el parametro.
        self.destino = str(self.declare_parameter('destino', 'gz').value).lower()
        if self.destino not in ('gz', 'placa', 'ambos'):
            raise ValueError(f"destino tiene que ser gz, placa o ambos: {self.destino!r}")
        self.a_gz = self.destino in ('gz', 'ambos')
        self.a_placa = self.destino in ('placa', 'ambos')
        # El techo REAL del hardware. Es el mismo numero que declara la aduana
        # (esp.py, max_vel_hw): el husillo de la mesa da 0.008 m/s, no los 0.10
        # del YAML, que son de la simulacion. Planificar con el del YAML y
        # mandarlo a la placa es adelantarse al brazo 12 veces.
        self.max_vel_hw = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:len(self.juntas)]
        # Salto articular a partir del cual se mete un tramo de aproximacion.
        self.aprox = float(self.declare_parameter('aprox', 0.02).value)
        self.home = [float(v) for v in
                     self.declare_parameter('home', [0.30, 0.0, 0.15]).value]
        self.demo = bool(self.declare_parameter('demo', False).value)
        self.lazo = bool(self.declare_parameter('lazo', False).value)
        self.espera = float(self.declare_parameter('espera', 6.0).value)
        self.publicar_js = bool(self.declare_parameter('publicar_js', False).value)
        registro = str(self.declare_parameter('registro', '').value)

        # --- salidas ---
        self.cmd = ({j: self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                     for j in self.juntas} if self.a_gz else {})
        # La aduana ya acepta una consigna de posicion por aqui y la convierte
        # ella sola en la linea 'T q1 q2 q3' del protocolo, a su propio ritmo.
        # No hay que escribir texto ni abrir el puerto: esp.py no se toca.
        self.hw_pub = (self.create_publisher(JointState, 'scara/hw/cmd', 10)
                       if self.a_placa else None)
        self.plan_pub = self.create_publisher(Path, 'scara/tray/plan', 10)
        self.real_pub = self.create_publisher(Path, 'scara/tray/real', 10)
        self.via_pub = self.create_publisher(Marker, 'scara/tray/via', 10)
        self.estado_pub = self.create_publisher(String, 'scara/tray/estado', 10)
        self.js_pub = (self.create_publisher(JointState, 'joint_states', 10)
                       if self.publicar_js else None)

        # --- entradas ---
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.create_subscription(String, 'scara/tray/orden', self.on_orden, 10)
        self.create_subscription(PoseStamped, 'scara/tray/meta', self.on_meta, 10)
        # El enclavamiento: la aduana solo pone esto en true si la placa contesta,
        # esta habilitada y tiene el cero hecho. Sin eso no se manda nada.
        self.hw_ok = False
        if self.a_placa:
            self.create_subscription(Bool, 'scara/hw/ok', self.on_hw_ok, 10)

        # --- estado ---
        self.q_real: list[float] | None = None
        self.q_cmd: list[float] | None = None
        self.tr: T.Trayectoria | None = None
        self.pendientes: list[tuple[str, T.Trayectoria, list]] = []
        self.etiqueta = ''
        self.k = 0                       # tics dentro de la trayectoria de ahora
        self.cola: list[str] = []
        self.real = Path()
        self.real.header.frame_id = self.base_frame
        self.arranque = None

        self.csv = None
        if registro:
            ruta = os.path.expanduser(registro)
            self.csv = csv.writer(open(ruta, 'w', newline=''))
            self.csv.writerow(
                ['t', 'metodo'] + [f'{j}_ref' for j in self.juntas]
                + [f'{j}_real' for j in self.juntas]
                + ['x_ref', 'y_ref', 'z_ref', 'x_real', 'y_real', 'z_real'])
            self.get_logger().info(f'anotando en {ruta}')

        self.create_timer(self.dt, self.tic)
        r_min, r_max = self.model.reach()
        self.get_logger().info(
            f'planificador | destino={self.destino} | perfil={self.perfil} '
            f'codo={self.codo} dt={self.dt:.3f}s | '
            f'anillo r=[{r_min:.3f}, {r_max:.3f}] m, z=[{self.model.z_min:.3f}, '
            f'{self.model.z_max:.3f}] m | ordenes por scara/tray/orden')
        if self.a_placa:
            self.get_logger().warning(
                f'destino={self.destino}: las consignas van al BRAZO REAL por '
                f'scara/hw/cmd. Techo de velocidad {["%.4f" % v for v in self._techo()]} '
                f'(husillo {self.max_vel_hw[2]:.4f} m/s). No se movera hasta que '
                f'scara/hw/ok sea true')
        if self.demo:
            self.get_logger().info(
                f'demo: empieza en {self.espera:.0f} s, cuando gz haya cargado el robot')

    # ==================================================================== entrada
    def on_js(self, msg: JointState):
        """Lo que mide Gazebo. Es el origen de la primera trayectoria."""
        donde = {n: i for i, n in enumerate(msg.name)}
        if any(j not in donde for j in self.juntas):
            return
        self.q_real = [msg.position[donde[j]] for j in self.juntas]

    def on_hw_ok(self, msg: Bool):
        if self.hw_ok and not msg.data and self.tr is not None:
            # Se cayo el enlace, o alguien quito la potencia, a mitad de un
            # movimiento. Se corta aqui: la aduana se queda con la ultima
            # consigna y el brazo no sigue persiguiendo un punto a ciegas.
            self.tr = None
            self.pendientes.clear()
            self.get_logger().error(
                'scara/hw/ok se ha caido a mitad de la trayectoria: corto y me quedo '
                'donde estoy. Mira la placa, la potencia y el cero')
        self.hw_ok = bool(msg.data)

    def on_meta(self, msg: PoseStamped):
        p = msg.pose.position
        self.on_orden(String(data=f'recta {p.x:.5f} {p.y:.5f} {p.z:.5f}'))

    def on_orden(self, msg: String):
        texto = msg.data.strip()
        if not texto or texto.startswith('#'):
            return
        cabeza = texto.split()[0].lower()
        if cabeza == 'parar':
            self.cola.clear()
            self.pendientes.clear()
            self.tr = None
            self.get_logger().warning('parado; me quedo donde estoy')
            return
        if cabeza in ('demo', 'perfiles', 'caminos'):
            guia = {'perfiles': GUIA_PERFILES, 'caminos': GUIA_CAMINOS,
                    'demo': GUIA_PERFILES + GUIA_CAMINOS}[cabeza]
            extra = ' '.join(t for t in texto.split()[1:] if '=' in t)
            self.cola = [f'{o} {extra}'.strip() if not o.startswith('#') else o
                         for o in guia]
            self.get_logger().info(f'{cabeza}: {len(self.cola)} movimientos en cola')
            return
        self.cola.append(texto)

    # ================================================================ el planificador
    def _partir(self, texto: str):
        """Separa la orden en (verbo, numeros, opciones). Las opciones son k=v."""
        verbo, nums, op = None, [], {}
        for tk in texto.split():
            if verbo is None:
                verbo = tk.lower()
            elif '=' in tk:
                k, _, v = tk.partition('=')
                op[k.lower()] = v
            else:
                nums.append(float(tk))
        return verbo, nums, op

    def _camino(self, verbo: str, v: list[float], codo: str, q0: list[float]):
        """De la orden al objeto Camino. Levanta ValueError si faltan numeros."""
        m = self.model

        def trios(x, que):
            if len(x) < 6 or len(x) % 3:
                raise ValueError(f'{que} pide grupos de 3 numeros (x y z), '
                                 f'al menos 2 puntos; me han llegado {len(x)}')
            return [tuple(x[i:i + 3]) for i in range(0, len(x), 3)]

        if verbo == 'junta':
            if len(v) != self.n:
                raise ValueError(f'junta pide {self.n} numeros: {" ".join(self.juntas)}')
            return T.CaminoJunta(q0, m.clamp(v)), []
        if verbo == 'recta':
            if len(v) != 3:
                raise ValueError('recta pide 3 numeros: x y z')
            return T.CaminoRecta(m, m.fk(q0)[:3], tuple(v), codo), [tuple(v)]
        if verbo == 'arco':
            if len(v) not in (5, 6):
                raise ValueError('arco pide cx cy r a0 a1 [z], con a0/a1 en grados')
            z = v[5] if len(v) == 6 else m.fk(q0)[2]
            cam = T.CaminoArco(m, (v[0], v[1]), v[2],
                               math.radians(v[3]), math.radians(v[4]), z, z, codo)
            return cam, cam.via()
        if verbo == 'circulo':
            if len(v) not in (3, 4, 5):
                raise ValueError('circulo pide cx cy r [z] [vueltas]')
            z = v[3] if len(v) > 3 else m.fk(q0)[2]
            cam = T.CaminoArco.circulo(m, (v[0], v[1]), v[2], z,
                                       v[4] if len(v) > 4 else 1.0, 1.0, codo)
            return cam, cam.via()
        if verbo == 'tres':
            if len(v) not in (6, 7):
                raise ValueError('tres pide x1 y1 x2 y2 x3 y3 [z]')
            z = v[6] if len(v) == 7 else m.fk(q0)[2]
            pts = [(v[0], v[1], z), (v[2], v[3], z), (v[4], v[5], z)]
            return T.CaminoArco.por_tres(m, *pts, codo=codo), pts
        if verbo in ('poligonal', 'spline'):
            pts = trios(v, verbo)
            C = T.CaminoPoligonal if verbo == 'poligonal' else T.CaminoSpline
            return C(m, pts, codo), pts
        if verbo == 'home':
            return T.CaminoRecta(m, m.fk(q0)[:3], tuple(self.home), codo), [tuple(self.home)]
        raise ValueError(f'no se que es {verbo!r}')

    def _lanzar(self, texto: str) -> bool:
        """Planifica una orden y la deja lista para el timer. False si no vale."""
        if self.a_placa and not self.hw_ok:
            self.get_logger().error(
                f'«{texto}» no se manda: scara/hw/ok esta en false. La placa tiene '
                f'que estar conectada, habilitada (scara/hw/enable) y con el cero '
                f'hecho (scara/hw/zero) antes de mover nada')
            return False
        q0 = self.q_cmd or self.q_real
        if q0 is None:
            self.get_logger().warning(
                'todavia no se donde esta el brazo: espero a /joint_states de gz',
                throttle_duration_sec=5.0)
            return False
        try:
            verbo, nums, op = self._partir(texto)
            pf = T.perfil(op.get('perfil', self.perfil))
            codo = op.get('codo', self.codo)
            dur = float(op['t']) if 't' in op else (self.t_fijo or None)
            cam, via = self._camino(verbo, nums, 'nearest' if codo == 'auto' else codo, q0)
            ramas = ('nearest', 'up', 'down') if codo == 'auto' else (codo,)
            tr = T.planificar(cam, pf, q0, self._techo(), dur, ramas,
                              muestras=self.muestras)
        except (ValueError, IKError) as e:
            self.get_logger().error(f'«{texto}» no sale: {e}')
            return False

        # Un arco, una spline o una poligonal empiezan en su primer punto via,
        # que no tiene por que ser donde esta el brazo -y puede estar ademas en
        # la otra rama del codo-. Si se mandara tal cual, la primera consigna
        # seria un escalon de varios radianes. Se cose con un tramo de
        # aproximacion en espacio articular: va exactamente a tabla[0], no
        # necesita IK y por tanto no puede equivocarse de rama.
        salto = max(abs(a - b) for a, b in zip(q0, tr.tabla[0]))
        cola = []
        if salto > self.aprox:
            ap = T.planificar(T.CaminoJunta(q0, tr.tabla[0]), pf, q0, self._techo(),
                              dur, muestras=self.muestras)
            cola.append((f'aproximacion+{pf.nombre}', ap,
                         [tuple(self.model.fk(tr.tabla[0])[:3])]))
            self.get_logger().info(
                f'aproximacion: {salto:.3f} de la pose de ahora al principio '
                f'de {cam.nombre}, {ap.duracion:.2f}s')
        cola.append((f'{cam.nombre}+{pf.nombre}', tr, via))

        rama = getattr(cam, 'codo', '-')
        pico = max(abs(v) for v in tr.muestra(tr.duracion / 2.0)[1])
        self.get_logger().info(
            f'{cam.nombre}+{pf.nombre}  T={tr.duracion:.2f}s  codo={rama}  '
            f'kv={pf.kv:.3f}  |qd|max~{pico:.3f}')
        self.pendientes = cola
        return True

    def _techo(self) -> list[float]:
        """El max_vel que se le pide de verdad, segun a quien se le mande.

        Para gz, el del YAML escalado por factor_vel (el PID de la simulacion no
        sigue el techo entero). Para la placa, el max_vel_hw, que ya es el limite
        medido del hardware. Para los dos a la vez, el menor de ambos.
        """
        gz = [v * self.factor_vel for v in self.model.max_vel]
        if not self.a_placa:
            return gz
        if not self.a_gz:
            return list(self.max_vel_hw)
        return [min(a, b) for a, b in zip(gz, self.max_vel_hw)]

    # ==================================================================== el reloj
    def tic(self):
        ahora = self.get_clock().now()
        if self.arranque is None:
            self.arranque = ahora
            return
        edad = (ahora - self.arranque).nanoseconds * 1e-9

        if self.tr is None:
            if self.demo and not self.cola and not self.pendientes and edad > self.espera:
                self.on_orden(String(data='demo'))
                self.demo = self.lazo          # sin lazo, solo una vuelta
            while not self.pendientes and self.cola:
                orden = self.cola.pop(0)
                if orden.startswith('#'):
                    self.get_logger().info(orden.lstrip('# '))
                    continue
                self._lanzar(orden)
            if not self.pendientes:
                self._estado('en espera')
                return
            self.etiqueta, self.tr, via = self.pendientes.pop(0)
            self.k = 0
            self._dibujar_plan(self.tr, via)

        t = self.k * self.dt
        q, qd = self.tr.muestra(t)
        self.k += 1
        self._mandar(q, qd)
        self._anotar(t, q)
        frac = min(t / self.tr.duracion, 1.0)
        self._estado(f'{self.etiqueta} {frac * 100:3.0f}% '
                     f'({t:.2f}/{self.tr.duracion:.2f} s)')

        if t >= self.tr.duracion:
            self.q_cmd = self.tr.fin()
            self.tr = None
            if self.lazo and not self.cola and not self.pendientes and not self.demo:
                self.cola = list(GUIA_PERFILES + GUIA_CAMINOS)

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
        if self.js_pub is not None:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.juntas)
            js.position = [float(v) for v in q]
            js.velocity = [float(v) for v in qd]
            self.js_pub.publish(js)

    # ==================================================================== lo que se ve
    def _pose(self, q, stamp) -> PoseStamped:
        x, y, z, yaw = self.model.fk(q)
        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.base_frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _dibujar_plan(self, tr: T.Trayectoria, via):
        """El camino entero de una vez, antes de moverse: se ve a donde va."""
        stamp = self.get_clock().now().to_msg()
        plan = Path()
        plan.header.stamp = stamp
        plan.header.frame_id = self.base_frame
        plan.poses = [self._pose(tr.tabla[i], stamp)
                      for i in range(0, tr.muestras + 1, 2)]
        self.plan_pub.publish(plan)
        self.real.poses.clear()

        mk = Marker()
        mk.header = plan.header
        mk.ns = 'via'
        mk.id = 0
        mk.type = Marker.SPHERE_LIST
        mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = mk.scale.z = 0.012
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.90, 0.54, 0.12, 0.95
        mk.pose.orientation.w = 1.0
        mk.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in via]
        self.via_pub.publish(mk)

        # El nombre del metodo, flotando sobre el robot: en el demo pasan once
        # trayectorias seguidas y desde fuera todas se parecen.
        txt = Marker()
        txt.header = plan.header
        txt.ns = 'metodo'
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.text = f'{self.etiqueta}  T={tr.duracion:.2f}s'
        txt.pose.position.z = self.model.z_max + 0.12
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.045
        txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.12, 0.15, 0.19, 1.0
        self.via_pub.publish(txt)

    def _estado(self, texto: str):
        self.estado_pub.publish(String(data=texto))
        if self.q_real is not None:
            self.real.header.stamp = self.get_clock().now().to_msg()
            self.real.poses.append(self._pose(self.q_real, self.real.header.stamp))
            if len(self.real.poses) > 3000:
                del self.real.poses[:len(self.real.poses) - 3000]
            self.real_pub.publish(self.real)

    def _anotar(self, t: float, q: list[float]):
        if self.csv is None:
            return
        qr = self.q_real or [float('nan')] * self.n
        xr = self.model.fk(qr)[:3] if self.q_real else (float('nan'),) * 3
        self.csv.writerow([f'{t:.4f}', self.etiqueta]
                          + [f'{v:.6f}' for v in q] + [f'{v:.6f}' for v in qr]
                          + [f'{v:.6f}' for v in self.model.fk(q)[:3]]
                          + [f'{v:.6f}' for v in xr])


def main(args=None):
    rclpy.init(args=args)
    node = Planificador()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
