"""Nodo `control_nodo`: el control cinematico del SCARA, cerrado con Gazebo.

    ros2 launch scara_kinematics control.launch.py                   # un circulo
    ros2 launch scara_kinematics control.launch.py tarea:=perforado
    ros2 launch scara_kinematics control.launch.py k:=10 rqt:=true

En cada tic (100 Hz, con el reloj de gz):

    q  = /joint_states                                    lo que MIDE gz
    qd = J(q)^-1 (xd_dot + K (xd - FK(q)))                control_cinematico.Ley
    qd --Float64--> /scara_urdf/control/cmd_vel/<junta> --> gz, juntas por velocidad

La diferencia con `ruta_nodo`: alli la trayectoria se integra entera sobre el
modelo y se manda como posiciones (el lazo lo cierra el PID de cada junta).
Aqui no hay PID de junta: el unico lazo es este, y se cierra con la punta medida.

Cuando no hay tarea, regula: mantiene la punta en el ultimo punto pedido.

Ordenes (std_msgs/String en scara/control/orden):

    home                                     a HOME (300, 0, 150) mm, en articular
    ir X Y Z [v=40] [perfil=trapecio]        recta hasta (X, Y, Z) mm: seguimiento
    salto X Y Z                              la consigna salta: regulacion, mide tau
    circulo [CX CY R] [z=150] [v=20] [vueltas=1] [perfil=trapecio]
    perforado | contorno [perfil=trapecio]   las rutas de ruta.py
    empujon DX DY DZ                         saca la punta (mm) sin control y mide la vuelta
    k=K                                      cambia la ganancia en marcha
    parar                                    se queda donde esta, regulando ahi

Una orden nueva sustituye a la que este en marcha.

Salidas:
    <cmd_vel_prefix><junta>       std_msgs/Float64             qd (rad/s, m/s)
    scara/control/estado          std_msgs/String              tarea, tramo, % y error
    scara/control/deseado         nav_msgs/Path                el camino pedido
    scara/control/real            nav_msgs/Path                lo que hizo la punta
    scara/control/objetivo        visualization_msgs/Marker    xd, ahora
    scara/control/datos/*         Vector3 / Float64            para rqt (mm, grados)

Y en la ventana de Gazebo, el camino pedido en verde y el hecho en naranja.
"""
from __future__ import annotations

import csv
import math
import os
import signal
import subprocess
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

from scara_kinematics import control_cinematico as C
from scara_kinematics import ruta as R
from scara_kinematics import trayectoria as T
from scara_kinematics.model import declare_model_params

EMPUJON_T = 0.2          # s que dura un empujon (mas si la junta no da para tanto)


class _PintorGz:
    """Dibuja en la ventana de Gazebo con `gz service -s /marker`, en otro hilo.

    Cada llamada es un proceso, asi que no se pinta a 100 Hz: el camino pedido
    una vez por tarea y el hecho una vez por segundo. Si solo se queda el
    ultimo dibujo de cada id, la cola no crece aunque gz vaya lento. Sin
    ventana de gz (gui:=false) el servicio no existe: tras tres fallos seguidos
    se deja de intentar.

    Los caminos van como cintas de triangulos y no como LINE_STRIP: las lineas
    de gz salen de un pixel y casi no se ven.
    """

    def __init__(self, logger):
        self.log = logger
        self.pendiente: dict[int, str] = {}
        self.cerrojo = threading.Lock()
        self.aviso = threading.Event()
        self.fallos = 0
        self.vivo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    @staticmethod
    def _color(r, g, b):
        c = f'{{r: {r} g: {g} b: {b} a: 1}}'
        return f'material: {{ambient: {c} diffuse: {c} emissive: {c}}}'

    # Linux no deja pasar a un programa un argumento de mas de 128 KB
    MAX_REQ = 120_000

    def linea(self, id_: int, puntos_mm, rgb, ancho_mm: float, tramos: int = 150,
              canto: bool = False):
        """Un camino como una cinta de `ancho_mm`, por las dos caras: plana (en
        horizontal) o de canto (en vertical). La pedida va plana y la hecha de
        canto, asi la naranja asoma por encima de la verde aunque coincidan.
        Con `tramos` tramos como mucho, y menos si no cabe en MAX_REQ."""
        P0 = np.asarray(puntos_mm, dtype=float) * 1e-3
        while tramos >= 10:
            P = P0[np.unique(np.linspace(0, len(P0) - 1, tramos + 1).round().astype(int))]
            vert = []
            for a, b in zip(P[:-1], P[1:]):
                d = b - a
                n = np.linalg.norm(d)
                if n < 1e-6:
                    continue
                d /= n
                w1 = np.cross(d, [0.0, 0.0, 1.0])
                w1 = (w1 / np.linalg.norm(w1) if np.linalg.norm(w1) > 1e-6
                      else np.array([1.0, 0.0, 0.0]))
                for w in ((np.cross(d, w1),) if canto else (w1,)):
                    h = w * ancho_mm * 0.5e-3
                    v = (a - h, a + h, b + h, b - h)
                    for i, j, k in ((0, 1, 2), (0, 2, 3), (0, 2, 1), (0, 3, 2)):
                        vert += [v[i], v[j], v[k]]
            if not vert:
                return
            req = (f'ns:"control" id:{id_} action:ADD_MODIFY type:TRIANGLE_LIST '
                   f'{self._color(*rgb)} '
                   + ' '.join(f'point{{x:{p[0]:.4f} y:{p[1]:.4f} z:{p[2]:.4f}}}' for p in vert))
            if len(req) <= self.MAX_REQ:
                self._pedir(id_, req)
                return
            tramos = int(tramos * 0.7)

    def bola(self, id_: int, p_mm, rgb, d_mm: float = 12.0):
        s = d_mm * 1e-3
        self._pedir(id_, f'ns: "control" id: {id_} action: ADD_MODIFY type: SPHERE '
                         f'{self._color(*rgb)} scale: {{x: {s} y: {s} z: {s}}} '
                         f'pose: {{position: {{x: {p_mm[0] * 1e-3:.4f} '
                         f'y: {p_mm[1] * 1e-3:.4f} z: {p_mm[2] * 1e-3:.4f}}}}}')

    def borrar(self, id_: int):
        self._pedir(id_, f'ns: "control" id: {id_} action: DELETE_MARKER')

    def _pedir(self, id_: int, req: str):
        if not self.vivo:
            return
        with self.cerrojo:
            self.pendiente[id_] = req
        self.aviso.set()

    def _bucle(self):
        while self.vivo:
            self.aviso.wait()
            with self.cerrojo:
                trabajo = list(self.pendiente.values())
                self.pendiente.clear()
                self.aviso.clear()
            for req in trabajo:
                try:
                    r = subprocess.run(
                        ['gz', 'service', '-s', '/marker', '--reqtype', 'gz.msgs.Marker',
                         '--reptype', 'gz.msgs.Empty', '--timeout', '5000', '--req', req],
                        capture_output=True, text=True, timeout=10.0)
                    ok = r.returncode == 0 and 'timed out' not in (r.stdout + r.stderr)
                except (OSError, subprocess.TimeoutExpired):
                    ok = False
                self.fallos = 0 if ok else self.fallos + 1
                if self.fallos >= 3:
                    self.vivo = False
                    self.log.info('gz no contesta en /marker (sin ventana?): '
                                  'no pinto el camino en Gazebo')
                    return


class ControlNodo(Node):

    def __init__(self):
        super().__init__('control')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        self.juntas = list(self.model.joint_names)[:3]

        self.base_frame = self.declare_parameter('base_frame', 'world').value
        self.cmd_prefix = self.declare_parameter(
            'cmd_vel_prefix', '/scara_urdf/control/cmd_vel/').value
        self.dt = float(self.declare_parameter('dt', 0.01).value)
        k = float(self.declare_parameter('k', C.K_DEF).value)
        # Los techos del brazo REAL, tambien en gz, para que lo que se mida aqui
        # valga para la mesa: el husillo no pasa de 8 mm/s.
        techo = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:3]
        self.qdot_max = np.array([techo[0], techo[1], techo[2] * 1e3])
        self.margen = float(self.declare_parameter('margen', C.MARGEN_VEL).value)
        self.vigilante = float(self.declare_parameter('vigilante', 0.25).value)
        self.espera = float(self.declare_parameter('espera', 6.0).value)
        self.tarea_ini = str(self.declare_parameter('tarea', 'circulo').value).strip()
        # ros2 launch no acepta `tarea:=` vacio: para no hacer nada, «nada».
        if self.tarea_ini.lower() == 'nada':
            self.tarea_ini = ''
        self.perfil = str(self.declare_parameter('perfil', 'trapecio').value)
        self.v_ir = float(self.declare_parameter('v_ir', 40.0).value)
        self.v_circulo = float(self.declare_parameter('v_circulo', 20.0).value)
        self.circulo = [float(v) for v in self.declare_parameter(
            'circulo', [330.0, 0.0, 40.0]).value]
        self.z_circulo = float(self.declare_parameter('z_circulo', R.Z_SEGURO).value)
        self.vel_ruta = {k_: float(self.declare_parameter(f'v_{k_}', v).value)
                         for k_, v in R.VELOCIDADES.items()}
        pintar = bool(self.declare_parameter('pintar_gz', True).value)
        registro = str(self.declare_parameter('registro', '').value)

        self.ley = C.Ley(self.robot, k, self.qdot_max)

        # --- salidas ---
        self.cmd = [self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                    for j in self.juntas]
        self.estado_pub = self.create_publisher(String, 'scara/control/estado', 10)
        self.deseado_pub = self.create_publisher(Path, 'scara/control/deseado', 10)
        self.real_pub = self.create_publisher(Path, 'scara/control/real', 10)
        self.objetivo_pub = self.create_publisher(Marker, 'scara/control/objetivo', 10)
        # Sin header a proposito, como en ruta_nodo: rqt_plot usaria la hora del
        # header y con la de gz las graficas salen corridas.
        self.datos = {k_: self.create_publisher(Vector3, f'scara/control/datos/{k_}', 10)
                      for k_ in ('punta_ref', 'punta_real', 'error', 'qd')}
        self.norma_pub = self.create_publisher(Float64, 'scara/control/datos/error_norma', 10)
        self.sat_pub = self.create_publisher(Float64, 'scara/control/datos/saturacion', 10)

        # --- entradas ---
        # Cola de UNO: gz manda /joint_states cada milisegundo y lo que importa es
        # el ultimo. Con cola larga el lazo lee medidas atrasadas, y el retraso
        # dentro del lazo es lo que limita la K.
        self.create_subscription(JointState, 'joint_states', self.on_js, 1)
        self.create_subscription(String, 'scara/control/orden', self.on_orden, 10)

        # --- estado ---
        self.q: np.ndarray | None = None        # medida [rad, rad, mm]
        self.t_js = None
        self.sin_medida = None                  # desde cuando la medida es vieja
        self.tic_ant = None                     # el tic anterior, para ver si hubo parón
        self.paron = 0.0                        # s que el nodo estuvo parado, sin recolocar
        self.t_ref_ant = 0.0                    # tiempo de referencia del ultimo tic bueno
        self.p_fija: np.ndarray | None = None    # a donde regula sin tarea
        self.tarea: C.Secuencia | None = None
        self.t0 = None
        self.t0_pendiente = False               # la tarea empieza a contar en el tic siguiente
        self.errores: dict[str, list[float]] = {}
        self.empuje = None                      # (t_fin, qd)
        self.medida = None                      # dict: lo que se mide de un salto/empujon
        self.ordenes: list[str] = []
        self.arranque = None
        self.tics = 0
        self.real = Path()
        self.real.header.frame_id = self.base_frame
        self.traza: list[np.ndarray] = []
        self.traza_pintada = False
        self.pintor = _PintorGz(self.get_logger()) if pintar else None

        self.csv = None
        if registro:
            ruta_csv = os.path.expanduser(registro)
            self._csv_f = open(ruta_csv, 'w', newline='')
            self.csv = csv.writer(self._csv_f)
            self.csv.writerow(
                ['t_medida', 'tarea', 'tramo', 'xd_mm', 'yd_mm', 'zd_mm', 'x_mm', 'y_mm', 'z_mm',
                 'ex_mm', 'ey_mm', 'ez_mm', 'error_mm', 'q1_rad', 'q2_rad', 'd3_mm',
                 'qd1_rad_s', 'qd2_rad_s', 'qd3_mm_s', 'saturacion', 'amortiguamiento', 'k'])
            self.get_logger().info(f'anotando en {ruta_csv}')

        self.create_timer(self.dt, self.tic)
        self.get_logger().info(
            f'control cinematico | K = {self._k_txt()} 1/s (tau = {1.0 / self.ley.k[0]:.3f} s) '
            f'| techo {self.qdot_max[0]} rad/s, {self.qdot_max[1]} rad/s, '
            f'{self.qdot_max[2]} mm/s | {1.0 / self.dt:.0f} Hz | ordenes por scara/control/orden')
        if self.tarea_ini:
            self.ordenes.append(self.tarea_ini)
            self.get_logger().info(f'«{self.tarea_ini}»: empieza en {self.espera:.0f} s, '
                                   f'cuando gz haya cargado el robot')

    def _k_txt(self) -> str:
        k = self.ley.k
        return f'{k[0]:g}' if np.all(k == k[0]) else ', '.join(f'{v:g}' for v in k)

    # ==================================================================== entrada
    def on_js(self, msg: JointState):
        donde = {n: i for i, n in enumerate(msg.name)}
        if any(j not in donde for j in self.juntas):
            return
        q = [msg.position[donde[j]] for j in self.juntas]
        self.q = np.array([q[0], q[1], q[2] * 1e3])
        # Cuando MIDIO gz (su reloj), no cuando ha llegado aqui: el error se
        # calcula contra donde tenia que estar la punta en ese instante.
        sello = Time.from_msg(msg.header.stamp)
        self.t_js = sello if sello.nanoseconds > 0 else self.get_clock().now()

    def on_orden(self, msg: String):
        if msg.data.strip():
            self.ordenes.append(msg.data.strip())

    # ==================================================================== ordenes
    def _orden(self, texto: str):
        palabras = texto.replace(',', ' ').split()
        nombre = palabras[0].lower()
        op = dict(p.split('=', 1) for p in palabras[1:] if '=' in p)
        try:
            nums = [float(p) for p in palabras[1:] if '=' not in p]
        except ValueError:
            self.get_logger().error(f'«{texto}»: los numeros no se entienden')
            return
        if nombre.startswith('k='):
            op['k'], nombre = nombre[2:], 'k'
        log = self.get_logger()
        try:
            if nombre == 'parar':
                self._fijar(R.fk_scara_rrp(self.q, self.robot))
                log.warning('parado: regulo donde estoy')
            elif nombre == 'k':
                # k=5 | k 5 | k=5,5,2 (x, y, z)
                k = ([float(op['k'])] if 'k' in op else []) + nums
                if len(k) not in (1, 3) or not all(0.0 < v <= 100.0 for v in k):
                    raise C.ControlError(f'K es uno o tres numeros entre 0 y 100: {k}')
                self.ley.k = k if len(k) == 3 else k[0]
                log.info(f'K = {self._k_txt()} 1/s (tau = {1.0 / self.ley.k[0]:.3f} s)')
            elif nombre == 'salto':
                self._salto(self._punto(nums, texto))
            elif nombre == 'empujon':
                self._empujon(self._punto(nums, texto))
            else:
                self._lanzar(self._preparar(nombre, nums, op))
        except (C.ControlError, ValueError) as e:
            log.error(f'«{texto}» no sale: {e}')

    @staticmethod
    def _punto(nums, texto) -> np.ndarray:
        if len(nums) != 3:
            raise C.ControlError(f'hacen falta tres numeros (mm) y hay {len(nums)}')
        return np.array(nums, dtype=float)

    def _perfil(self, op) -> T.Perfil:
        return R.perfil(op.get('perfil', self.perfil))

    def _aprox(self, q1) -> list[C.Referencia]:
        """En articular desde donde esta el brazo, si no esta ya ahi."""
        if np.max(np.abs(np.asarray(q1) - self.q) / self.qdot_max) < 0.002:
            return []
        return [C.Articular(self.q, q1, self.robot, self.qdot_max, self.margen)]

    def _a_tiempo(self, ref, codo):
        f = C.a_tiempo(ref, self.robot, codo, self.qdot_max, self.margen)
        if f > 1.0:
            self.get_logger().warning(
                f'{ref.tipo}: a esa velocidad alguna junta pasa de su techo; '
                f'la hago {f:.2f} veces mas lenta ({ref.duracion:.1f} s)')

    def _preparar(self, nombre, nums, op) -> C.Secuencia:
        """La tarea entera, comprobada antes de mover nada."""
        codo = C.rama(self.q, self.robot)
        p_ahora = (self.p_fija if self.tarea is None and self.p_fija is not None
                   else R.fk_scara_rrp(self.q, self.robot))
        if nombre == 'home':
            return C.Secuencia('home', [C.Articular(
                self.q, R.ik_scara_rrp(R.HOME, self.robot, codo), self.robot,
                self.qdot_max, self.margen)])
        if nombre == 'ir':
            destino = self._punto(nums, 'ir')
            C.revisar(C.Fija(destino), self.robot, codo, self.qdot_max)  # primero, el final
            rc = C.Recta(p_ahora, destino, float(op.get('v', self.v_ir)),
                         self._perfil(op), tipo='recta')
            self._a_tiempo(rc, codo)
            return C.Secuencia('ir', [rc])
        if nombre == 'circulo':
            if nums and len(nums) != 3:
                raise C.ControlError('circulo lleva CX CY R (mm) o nada')
            cx, cy, r = nums or self.circulo
            circ = C.Circulo((cx, cy), r, float(op.get('z', self.z_circulo)),
                             float(op.get('v', self.v_circulo)),
                             float(op.get('vueltas', 1.0)), self._perfil(op))
            self._a_tiempo(circ, codo)
            q1 = R.ik_scara_rrp(circ.inicio(), self.robot, codo)
            return C.Secuencia('circulo', self._aprox(q1) + [circ])
        if nombre in R.RUTAS:
            ruta = R.RUTAS[nombre]
            partes = []
            for i in range(ruta.shape[1] - 1):
                tipo = R.tipo_de_tramo(ruta[:, i], ruta[:, i + 1])
                rc = C.Recta(ruta[:, i], ruta[:, i + 1], self.vel_ruta[tipo],
                             self._perfil(op), tipo=tipo)
                self._a_tiempo(rc, codo)
                partes.append(rc)
            q1 = R.ik_scara_rrp(ruta[:, 0], self.robot, codo)
            return C.Secuencia(nombre, self._aprox(q1) + partes)
        raise C.ControlError(
            f'no conozco «{nombre}». Hay: home, ir, salto, circulo, '
            f'{", ".join(R.RUTAS)}, empujon, k=, parar')

    # ==================================================================== tareas
    def _lanzar(self, tarea: C.Secuencia):
        self.tarea = tarea
        self.t0 = self.get_clock().now()
        self.t0_pendiente = True
        self.t_ref_ant = 0.0
        self.medida = None
        self.errores = {}
        partes = ', '.join(f'{p.tipo} {p.duracion:.1f} s' for p in tarea.partes)
        self.get_logger().info(
            f'{tarea.nombre}: {tarea.duracion:.1f} s ({partes}) | rama del codo '
            f'{C.rama(self.q, self.robot)} | K = {self._k_txt()} 1/s')
        self._nuevo_dibujo(tarea.muestras(600), tarea.fin())

    def _fijar(self, p):
        """Sin tarea: regular en p."""
        self.tarea = None
        self.p_fija = np.asarray(p, dtype=float).copy()

    def _salto(self, p):
        """La consigna salta de golpe: es la respuesta al escalon de la teoria."""
        codo = C.rama(self.q, self.robot)
        C.revisar(C.Fija(p), self.robot, codo, self.qdot_max)
        p0 = R.fk_scara_rrp(self.q, self.robot)
        C.revisar(C.Recta(p0, p, 1.0), self.robot, codo, self.qdot_max, n=100)  # va recto
        self._fijar(p)
        self.errores = {}
        self._medir_desde(f'salto de {np.linalg.norm(p - p0):.1f} mm')
        self._nuevo_dibujo(np.array([p0, p]), p)

    def _empujon(self, d):
        """Durante EMPUJON_T s se manda una velocidad sin control que saca la punta
        d mm. Luego vuelve el lazo y se mide como recupera."""
        qd, _ = self.ley.inversa(self.q, d / EMPUJON_T)
        sat = float(np.max(np.abs(qd) / self.qdot_max))
        dur = EMPUJON_T * max(1.0, sat)
        self.empuje = (self.get_clock().now().nanoseconds * 1e-9 + dur, qd * EMPUJON_T / dur)
        self.medida = None
        self.get_logger().info(f'empujon de {np.linalg.norm(d):.1f} mm en {dur:.2f} s, '
                               f'sin control')

    def _medir_desde(self, que: str):
        self.medida = {'que': que, 't': [], 'e': [], 'sat': 1.0}

    def _cerrar_medida(self, razon: str):
        m, self.medida = self.medida, None
        t, e = np.array(m['t']), np.array(m['e'])
        k = self.ley.k[0]
        tau = C.tau(t, e)
        t5 = next((ti - t[0] for ti, ei in zip(t, e) if ei <= 0.05 * e[0]), None)
        txt = (f'{m["que"]}: de {e[0]:.2f} mm a {e[-1]:.3f} mm ({razon}) | tau medido '
               + (f'{tau:.3f} s' if tau is not None else '--')
               + f' (teoria 1/K = {1.0 / k:.3f} s) | al 5 % en '
               + (f'{t5:.3f} s' if t5 is not None else '--')
               + f' (teoria 3/K = {3.0 / k:.3f} s)')
        if m['sat'] > 1.0:
            txt += (f' | SATURO (pidio {m["sat"]:.1f} veces el techo): ahi cae en rampa, '
                    f'no en exponencial')
        self.get_logger().info(txt)

    # ==================================================================== el reloj
    def tic(self):
        ahora = self.get_clock().now()
        if self.arranque is None:
            self.arranque = ahora
        if self.tic_ant is not None:
            hueco = (ahora - self.tic_ant).nanoseconds * 1e-9
            if hueco > 5.0 * self.dt:
                self.paron += hueco
        self.tic_ant = ahora
        edad = math.inf if self.q is None else (ahora - self.t_js).nanoseconds * 1e-9
        if edad > self.vigilante:
            # Una medida vieja suelta no es que se haya caido gz: si el proceso se
            # para un momento (pasa al entrar otro participante de DDS), el primer
            # tic llega antes que la medida nueva. Ese tic no manda nada y gz sigue
            # con la ultima orden. Solo si sigue sin llegar `vigilante` segundos
            # MAS se paran las juntas: gz no las para solo.
            if self.sin_medida is None:
                self.sin_medida = ahora
            if self.q is None or (ahora - self.sin_medida).nanoseconds * 1e-9 > self.vigilante:
                self._mandar(np.zeros(3))
                self._estado('sin /joint_states: juntas paradas')
                if self.q is not None:
                    self.get_logger().warning(
                        f'/joint_states no llega desde hace {edad:.2f} s: paro las juntas',
                        throttle_duration_sec=5.0)
            return
        self.sin_medida = None
        if self.p_fija is None:
            self.p_fija = R.fk_scara_rrp(self.q, self.robot)
        if self.t0_pendiente:
            self.t0, self.t0_pendiente = ahora, False
        if self.ordenes and (ahora - self.arranque).nanoseconds * 1e-9 >= self.espera:
            self._orden(self.ordenes.pop(0))
            # Preparar una tarea lleva su tiempo (~0.1 s) y mientras tanto el reloj
            # de gz no se lee: llega por este mismo ejecutor, asi que pedirlo aqui
            # da la hora de antes. Ese tiempo aparece de golpe en el tic siguiente:
            # no es un paron (no se recoloca nada) y la tarea nueva empieza a
            # contar desde ese tic (t0_pendiente), no desde este.
            self.tic_ant = None

        # --- la referencia ---
        # El error, contra donde tenia que estar la punta CUANDO SE MIDIO; la
        # velocidad, la de ahora, que es cuando se va a aplicar. Comparar la
        # medida (de hace un tic o dos) con la consigna de ahora hace creer que
        # la punta va v*retraso por detras, y el lazo la adelantaria eso mismo:
        # el fallo de la guia que ya se corrigio en ruta.py.
        tramo, t = '', 0.0
        if self.tarea is not None and self.paron > 0.0:
            self._recolocar()
        self.paron = 0.0
        if self.tarea is not None:
            t = (ahora - self.t0).nanoseconds * 1e-9
            tm = (self.t_js - self.t0).nanoseconds * 1e-9
            self.t_ref_ant = max(tm, 0.0)
            pd, vd = self.tarea.en(max(tm, 0.0))[0], self.tarea.en(t)[1]
            tramo = self.tarea.partes[self.tarea.donde(t)[0]].tipo
        else:
            pd, vd = self.p_fija, np.zeros(3)

        # --- la ley ---
        s = self.ley.paso(self.q, pd, vd)
        qd = s.qd
        seg = ahora.nanoseconds * 1e-9
        if self.empuje is not None:
            if seg < self.empuje[0]:
                qd = self.empuje[1]
                tramo = 'empujon'
            else:
                self.empuje = None
                self._medir_desde('empujon')
        self._mandar(qd)

        # --- medir (en el instante de la medida) ---
        en = float(np.linalg.norm(s.e))
        t_med = self.t_js.nanoseconds * 1e-9
        if self.tarea is not None and tramo != 'empujon':
            self.errores.setdefault(tramo, []).append(en)
        if self.medida is not None:
            self.medida['t'].append(t_med)
            self.medida['e'].append(en)
            self.medida['sat'] = max(self.medida['sat'], s.sat)
            dura = t_med - self.medida['t'][0]
            if en < max(0.01, 0.002 * self.medida['e'][0]):
                self._cerrar_medida(f'{dura:.2f} s')
            elif dura > 8.0 / self.ley.k.min() + 10.0:
                self._cerrar_medida(f'no llega en {dura:.0f} s')
        if self.tarea is not None and t >= self.tarea.duracion:
            self._resumen()
            self._fijar(self.tarea.fin())
        self._publicar(s, pd, qd, tramo, t_med)

    def _recolocar(self):
        """El nodo ha estado parado (el ordenador va justo: pasa al entrar o salir
        otro participante de DDS) y gz no espera. Durante el paron las juntas han
        seguido con la ultima velocidad mandada; la referencia, con el reloj.

        Se pone la referencia en el punto del camino, entre donde iba antes del
        paron y ahora, mas cercano a la punta medida. Si el paron pillo al brazo
        siguiendo el camino, ese punto es el de ahora y no cambia nada; si lo
        pillo quieto (justo al arrancar una tarea), la referencia le espera en vez
        de irse 50 mm por delante.
        """
        tm = (self.t_js - self.t0).nanoseconds * 1e-9
        if tm <= self.t_ref_ant:
            return
        ts = np.linspace(self.t_ref_ant, tm, 80)
        p = R.fk_scara_rrp(self.q, self.robot)
        t_mejor = ts[int(np.argmin([np.linalg.norm(self.tarea.en(t)[0] - p) for t in ts]))]
        atraso = tm - t_mejor
        if atraso > 2.0 * self.dt:
            self.t0 += Duration(nanoseconds=int(atraso * 1e9))
            self.get_logger().info(
                f'el nodo estuvo parado {self.paron:.2f} s (gz no espera): la referencia '
                f'se recoloca donde esta la punta, {atraso:.2f} s mas tarde')

    def _mandar(self, qd):
        for pub, v, f in zip(self.cmd, qd, (1.0, 1.0, 1e-3)):
            pub.publish(Float64(data=float(v) * f))

    def parar_juntas(self):
        """gz se queda con la ULTIMA velocidad que recibio: al cerrar, ceros. Y
        se espera a que el puente los confirme: si el proceso acaba antes de
        que salgan, es como no haberlos mandado."""
        self._mandar(np.zeros(3))
        for pub in self.cmd:
            pub.wait_for_all_acked(Duration(seconds=0.3))

    def _resumen(self):
        todos = [e for v in self.errores.values() for e in v]
        if not todos:
            return
        a = np.array(todos)
        log = self.get_logger()
        log.info(f'{self.tarea.nombre} terminada | punta medida frente a la deseada: '
                 f'medio {a.mean():.3f} mm, maximo {a.max():.3f} mm, '
                 f'RMS {math.sqrt(np.mean(a ** 2)):.3f} mm ({len(a)} muestras, '
                 f'K = {self._k_txt()} 1/s)')
        for tipo, v in self.errores.items():
            v = np.array(v)
            log.info(f'   {tipo:<15s} medio {v.mean():7.3f} mm   maximo {v.max():7.3f} mm'
                     f'   ({len(v)} muestras)')

    # ==================================================================== lo que se ve
    def _publicar(self, s: C.Salida, pd, qd, tramo, t_med):
        def vec(v):
            return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        self.datos['punta_ref'].publish(vec(pd))
        self.datos['punta_real'].publish(vec(s.p))
        self.datos['error'].publish(vec(s.e))
        self.datos['qd'].publish(vec([math.degrees(qd[0]), math.degrees(qd[1]), qd[2]]))
        self.norma_pub.publish(Float64(data=float(np.linalg.norm(s.e))))
        self.sat_pub.publish(Float64(data=float(s.sat)))
        if self.csv is not None:
            self.csv.writerow(
                [f'{t_med:.3f}', self.tarea.nombre if self.tarea else 'fija', tramo]
                + [f'{v:.4f}' for v in (*pd, *s.p, *s.e)]
                + [f'{np.linalg.norm(s.e):.4f}']
                + [f'{v:.6f}' for v in self.q] + [f'{v:.6f}' for v in qd]
                + [f'{s.sat:.3f}', f'{s.amort:.2f}', self._k_txt()])

        self.tics += 1
        activo = self.tarea is not None or self.medida is not None or self.empuje is not None
        if self.tics % 5 == 0:                       # 20 Hz
            self._objetivo(pd)
            if activo:
                self.real.header.stamp = self.get_clock().now().to_msg()
                self.real.poses.append(self._pose(s.p, self.real.header.stamp))
                del self.real.poses[:-5000]
                self.real_pub.publish(self.real)
                self.traza.append(s.p.copy())
        if self.tics % 20 == 0:                      # 5 Hz
            if self.tarea is not None:
                t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
                dur = self.tarea.duracion
                txt = (f'{self.tarea.nombre} {tramo} {min(t / dur, 1.0) * 100:3.0f}% '
                       f'({t:.1f}/{dur:.1f} s)')
            else:
                txt = f'regulando en [{s.p[0]:.1f}, {s.p[1]:.1f}, {s.p[2]:.1f}] mm'
            txt += f' | error {np.linalg.norm(s.e):.3f} mm | K {self._k_txt()}'
            if s.sat > 1.0:
                txt += f' | saturado x{s.sat:.1f}'
            if s.topes:
                txt += f' | en el tope: {", ".join(s.topes)}'
            self._estado(txt)
        # Cada `gz service` es un proceso (y de ruby): cada 2 s basta.
        if self.tics % 200 == 0 and activo and self.pintor and len(self.traza) > 1:
            self.pintor.linea(2, self.traza, (1.0, 0.45, 0.1), 6.0, tramos=100, canto=True)
            self.traza_pintada = True

    def _nuevo_dibujo(self, deseado_mm, fin_mm):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.base_frame
        path.poses = [self._pose(p, stamp) for p in deseado_mm]
        self.deseado_pub.publish(path)
        self.real.poses.clear()
        self.traza = []
        if self.pintor:
            # Verde y ancha la pedida; naranja y fina la hecha, que va por dentro.
            self.pintor.linea(1, deseado_mm, (0.1, 0.8, 0.2), 6.0, tramos=200)
            if self.traza_pintada:
                self.pintor.borrar(2)
                self.traza_pintada = False
            self.pintor.bola(3, fin_mm, (0.1, 0.8, 0.2))

    def _pose(self, p_mm, stamp) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) * 1e-3 for v in p_mm)
        ps.pose.orientation.w = 1.0
        return ps

    def _objetivo(self, pd):
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = self.base_frame
        mk.ns, mk.id = 'control', 0
        mk.type, mk.action = Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (
            float(v) * 1e-3 for v in pd)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.012
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.8, 0.2, 0.9
        self.objetivo_pub.publish(mk)

    def _estado(self, texto: str):
        self.estado_pub.publish(String(data=texto))


def main(args=None):
    # Sin los manejadores de senales de rclpy. Con ellos, Ctrl+C apaga el
    # contexto ANTES de llegar al finally, los ceros ya no salen y gz sigue
    # moviendo el brazo con la ultima velocidad hasta el tope (medido: 0.09 rad
    # en 2 s despues de cerrar el nodo). Aqui Ctrl+C y el SIGTERM del launch
    # solo levantan una bandera; el bucle sale entre dos callbacks, con rclpy
    # aun vivo para mandar los ceros. Los dos a mano: si el proceso nace con
    # SIGINT ignorado (lanzado en segundo plano desde un script), Python no
    # pone el suyo.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    fin = threading.Event()
    for sen in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sen, lambda *_: fin.set())
    node = ControlNodo()
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(node)
    try:
        # Con timeout: la espera vuelve a Python cada 0.1 s aunque gz no mande
        # /clock, y la bandera se mira.
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
