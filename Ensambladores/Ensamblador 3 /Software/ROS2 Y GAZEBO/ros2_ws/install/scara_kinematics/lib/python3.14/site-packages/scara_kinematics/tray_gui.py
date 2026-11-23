"""La ventana de las trayectorias: el nodo `tray`, con Gazebo delante.

    ros2 launch scara_kinematics tray_gui.launch.py

Es la cara de `tray`. No planifica ni mueve nada por su cuenta: compone la misma
orden de texto que se teclearia a mano en scara/tray/orden y la manda por ahi.
Lo que anade es poder VER, antes y despues de mandarla:

    antes    el borrador (azul discontinuo). La orden se pasa por el MISMO
             parser de tray.py y por `trayectoria.planificar`, asi que es el
             camino que va a seguir tray, con su duracion y su rama del codo. Si
             la IK no llega, el error sale aqui y el camino se pinta en rojo,
             antes de haber mandado nada.
    durante  el plan que publico tray (naranja), la punta que mide Gazebo
             (verde) y cuanto se separa la una del otro.
    despues  los mensajes de tray, que llegan por /rosout.

    Enter        manda la orden
    Esc          parar
    clic         en la planta pone el punto; lo que signifique depende del camino
    clic dcho.   deshace el ultimo clic
    clic en z    pone la altura

Entradas y salidas:

    scara/tray/orden   std_msgs/String             lo que se manda (lo unico)
    joint_states       sensor_msgs/JointState      el brazo que lee tray
    /clock             rosgraph_msgs/Clock         Gazebo corre, esta en pausa o no esta
    scara/tray/estado  std_msgs/String             que hace tray y por donde va
    scara/tray/plan    nav_msgs/Path               el camino que planifico tray
    scara/tray/via     visualization_msgs/Marker   sus puntos via y el metodo
    /rosout            rcl_interfaces/Log          los mensajes de tray

Y a Gazebo, por fuera de ROS, `gz service /world/<mundo>/control` para pausarlo y
seguir. Con Gazebo en pausa tray se para tambien: corre con el reloj de la
simulacion.
"""
from __future__ import annotations

import math
import queue
import re
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from scara_kinematics import trayectoria as T
from scara_kinematics.estilo import (AZUL, BORDE, FONDO, NARANJA, PAPEL, REJILLA,
                                     ROJO, SUAVE, TINTA, VERDE)
from scara_kinematics.model import IKError, declare_model_params
from scara_kinematics.tray import Planificador

VIVO_S = 1.0             # sin noticias en este tiempo, esa parte se da por caida
PAUSA_S = 0.5            # /clock llega pero no avanza en este tiempo: pausa

VISTA = 0.55             # la planta ensena x, y en [-VISTA, VISTA] m
LADO_MAX = 460           # px del lienzo de la planta, si cabe en la pantalla
CELDA = 2                # px por celda del mapa de alcance
REDONDEO = 0.005         # m: un pixel de la planta son ~2.5 mm; el clic se redondea
ANCHO_Z = 76             # la regla de z, a la derecha, igual de alta que la planta

# El mapa de alcance: fuera, con una sola rama del codo, o con las dos.
TONO_FUERA = PAPEL
TONO_UNA = '#edf1f7'
TONO_DOS = '#d3ddec'
BRAZO = '#9aa3b2'

PERFILES = ('cubico', 'quintico', 'septico', 'trapecio', 'scurve')
CODOS = ('auto', 'nearest', 'up', 'down')

# (camino, lo que se ve en el boton)
CAMINOS = (('recta', 'Recta'), ('junta', 'Junta'), ('arco', 'Arco'),
           ('circulo', 'Círculo'), ('tres', 'Tres puntos'),
           ('poligonal', 'Poligonal'), ('spline', 'Spline'))

# Los campos de cada camino: (clave, unidad, valor de partida). Los de partida
# son los del demo de tray.py, que se sabe que salen.
CAMPOS = {
    'recta': [('x', 'm', '0.200'), ('y', 'm', '0.200'), ('z', 'm', '0.110')],
    'junta': [('q1', '°', '-74.5'), ('q2', '°', '120.3'), ('d3', 'm', '0.100')],
    'arco': [('cx', 'm', '0.300'), ('cy', 'm', '0.000'), ('r', 'm', '0.080'),
             ('a0', '°', '-90'), ('a1', '°', '90'), ('z', 'm', '0.150')],
    'circulo': [('cx', 'm', '0.300'), ('cy', 'm', '0.000'), ('r', 'm', '0.080'),
                ('z', 'm', '0.150'), ('vueltas', '', '1')],
    'tres': [('x1', 'm', '0.320'), ('y1', 'm', '-0.100'), ('x2', 'm', '0.380'),
             ('y2', 'm', '0.000'), ('x3', 'm', '0.320'), ('y3', 'm', '0.100'),
             ('z', 'm', '0.150')],
    # En las de N puntos los campos son el punto SIGUIENTE, el que se anade.
    'poligonal': [('x', 'm', '0.300'), ('y', 'm', '0.000'), ('z', 'm', '0.140')],
    'spline': [('x', 'm', '0.300'), ('y', 'm', '0.000'), ('z', 'm', '0.140')],
}
PUNTOS = {
    'poligonal': [(0.26, -0.09, 0.14), (0.36, -0.09, 0.14), (0.36, 0.09, 0.14),
                  (0.26, 0.09, 0.14), (0.26, -0.09, 0.14)],
    'spline': [(0.26, -0.09, 0.14), (0.36, -0.09, 0.17), (0.36, 0.09, 0.11),
               (0.26, 0.09, 0.17), (0.26, -0.09, 0.14)],
}
AYUDA = {
    'recta': 'La punta va en línea recta desde donde está hasta (x, y, z).',
    'junta': 'Cada junta va de su valor de ahora al pedido, sin IK. Nunca falla, '
             'pero la punta dibuja una curva.',
    'arco': 'Arco de centro (cx, cy) y radio r, del ángulo a0 al a1. Si a0 y a1 '
            'se pasan de una vuelta, da más de una.',
    'circulo': 'Círculo entero de centro (cx, cy) y radio r. Tiene que caber en la '
               'zona sombreada: q1 no pasa de ±120°.',
    'tres': 'El arco que pasa por tres puntos, en el sentido que toca el del medio.',
    'poligonal': 'Tramos rectos que pasan por todos los puntos, con una esquina en '
                 'cada uno. Clic dcho.: quita el último.',
    'spline': 'Curva suave (spline cúbica natural) por todos los puntos, sin esquinas. '
              'Clic dcho.: quita el último.',
}
# Lo que pone cada clic en la planta, por turno.
PASOS = {
    'recta': ['el destino'],
    'junta': ['a dónde llevar la punta (se sacan q1 y q2 por IK)'],
    'arco': ['el centro', 'el inicio (radio y a0)', 'el final (a1)'],
    'circulo': ['el centro', 'un punto del borde (el radio)'],
    'tres': ['el primer punto', 'el del medio', 'el último'],
    'poligonal': ['un punto más'],
    'spline': ['un punto más'],
}
ORDEN_T = re.compile(r'(\d+)\s*%')


class _Sombra:
    """Lo poco de `Planificador` que miran `_partir`, `_camino` y `_techo`.

    La ventana no reescribe el parser de tray.py: lo llama con esto en el sitio
    de `self`. Asi el borrador sale del MISMO codigo que va a ejecutar la orden,
    y no hay dos traducciones de 'circulo 0.30 0 0.08' que puedan divergir.
    """

    def __init__(self, node: 'TrayGuiNode'):
        self.model = node.model
        self.juntas = list(node.model.joint_names)
        self.n = len(self.juntas)
        self.home = node.home
        self.factor_vel = node.factor_vel
        self.max_vel_hw = node.max_vel_hw
        self.a_gz = node.destino in ('gz', 'ambos')
        self.a_placa = node.destino in ('placa', 'ambos')


class Borrador:
    """Lo que saldria si la orden se mandase ahora mismo."""

    def __init__(self, texto: str, ok: bool, puntos: list, via: list,
                 aprox: list | None = None, detalle: str = ''):
        self.texto = texto
        self.ok = ok
        self.puntos = puntos          # (x, y, z) de la punta, en orden
        self.via = via
        self.aprox = aprox or []      # el tramo en juntas que mete tray antes
        self.detalle = detalle


class TrayGuiNode(Node):

    def __init__(self):
        super().__init__('tray_gui')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)
        # Los mismos parametros que declara tray, con los mismos valores por
        # defecto: el borrador tiene que planificar con los numeros de tray. El
        # launch le pasa a los dos los mismos argumentos.
        self.destino = str(self.declare_parameter('destino', 'gz').value).lower()
        self.perfil = str(self.declare_parameter('perfil', 'quintico').value)
        self.codo = str(self.declare_parameter('codo', 'auto').value)
        self.t_fijo = float(self.declare_parameter('duracion', 0.0).value)
        self.factor_vel = float(self.declare_parameter('factor_vel', 0.35).value)
        self.muestras = int(self.declare_parameter('muestras', 400).value)
        self.aprox = float(self.declare_parameter('aprox', 0.02).value)
        self.home = [float(v) for v in
                     self.declare_parameter('home', [0.30, 0.0, 0.15]).value]
        self.max_vel_hw = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:len(self.juntas)]
        # vacio = lo busca solo en `gz service -l` la primera vez que hace falta
        self.mundo = str(self.declare_parameter('mundo', '').value)

        self.q: list[float] | None = None
        self.t_js = 0.0
        self.reloj: float | None = None
        self.t_reloj = 0.0            # cuando llego el ultimo /clock
        self.t_avanza = 0.0           # cuando cambio de valor por ultima vez
        self.estado = ''
        self.t_estado = 0.0
        self.plan: list[tuple] = []
        self.plan_n = 0               # sube con cada plan nuevo
        self.via: list[tuple] = []
        self.metodo = ''
        self.rastro: list[tuple] = []
        self.log: queue.Queue = queue.Queue()
        self.pausa_pedida: bool | None = None

        self.orden_pub = self.create_publisher(String, 'scara/tray/orden', 10)
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(String, 'scara/tray/estado', self.on_estado, 10)
        self.create_subscription(Path, 'scara/tray/plan', self.on_plan, 10)
        self.create_subscription(Marker, 'scara/tray/via', self.on_via, 10)
        # /rosout se publica transient_local: asi llegan tambien los ultimos
        # mensajes que tray dijo antes de abrirse la ventana.
        self.create_subscription(
            Log, '/rosout', self.on_rosout,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    # -------------------------------------------------------------- entradas
    def on_js(self, msg: JointState):
        donde = {n: i for i, n in enumerate(msg.name)}
        if any(j not in donde for j in self.juntas):
            return
        q = [msg.position[donde[j]] for j in self.juntas]
        self.q = q
        self.t_js = time.monotonic()
        p = self.model.fk(q)[:3]
        r = self.rastro
        if not r or math.dist(r[-1], p) > 5e-4:
            r.append(p)
            if len(r) > 4000:
                del r[:1000]

    def on_clock(self, msg: Clock):
        v = msg.clock.sec + msg.clock.nanosec * 1e-9
        ahora = time.monotonic()
        if v != self.reloj:
            self.t_avanza = ahora
        self.reloj = v
        self.t_reloj = ahora

    def on_estado(self, msg: String):
        self.estado = msg.data
        self.t_estado = time.monotonic()

    def on_plan(self, msg: Path):
        self.plan = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                     for p in msg.poses]
        self.plan_n += 1
        self.rastro = []              # el rastro verde es el de este plan

    def on_via(self, msg: Marker):
        if msg.ns == 'via':
            self.via = [(p.x, p.y, p.z) for p in msg.points]
        elif msg.ns == 'metodo':
            self.metodo = msg.text

    def on_rosout(self, msg: Log):
        if msg.name == 'tray':
            self.log.put((int(msg.level), msg.msg))

    # --------------------------------------------------------------- estados
    @staticmethod
    def vivo(t: float, margen: float = VIVO_S) -> bool:
        return time.monotonic() - t < margen

    def gazebo(self) -> str:
        """'corriendo', 'pausa' o 'no' (sin /clock)."""
        if not self.vivo(self.t_reloj):
            return 'no'
        return 'corriendo' if self.vivo(self.t_avanza, PAUSA_S) else 'pausa'

    def ocupado(self) -> bool:
        return (self.vivo(self.t_estado) or self.gazebo() == 'pausa') \
            and bool(self.estado) and self.estado != 'en espera'

    # --------------------------------------------------------------- ordenes
    def mandar(self, texto: str):
        self.orden_pub.publish(String(data=texto))
        self.log.put(('tu', texto))

    def pausar(self, si: bool):
        """Pausa o sigue Gazebo. En un hilo: `gz service` tarda un par de segundos."""
        self.pausa_pedida = si
        threading.Thread(target=self._pausar, args=(si,), daemon=True).start()

    def _pausar(self, si: bool):
        gz = shutil.which('gz')
        if gz is None:
            self.log.put((40, 'no encuentro el comando gz: no puedo pausar Gazebo'))
            return
        try:
            if not self.mundo:
                lista = subprocess.run([gz, 'service', '-l'], capture_output=True,
                                       text=True, timeout=10).stdout
                m = re.search(r'^/world/([^/\s]+)/control$', lista, re.M)
                if m is None:
                    self.log.put((40, 'Gazebo no ofrece /world/<mundo>/control: '
                                      '¿está arrancado?'))
                    return
                self.mundo = m.group(1)
            r = subprocess.run(
                [gz, 'service', '-s', f'/world/{self.mundo}/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000', '--req', f'pause: {"true" if si else "false"}'],
                capture_output=True, text=True, timeout=10)
        except subprocess.TimeoutExpired:
            self.log.put((40, 'gz service no contesta'))
            return
        if 'data: true' in r.stdout:
            self.log.put(('gz', 'Gazebo en pausa (tray se para con él)' if si
                          else 'Gazebo sigue'))
        else:
            self.log.put((40, f'Gazebo no ha hecho caso: {(r.stdout + r.stderr).strip()}'))


class VentanaTray:

    def __init__(self, node: TrayGuiNode):
        self.n = node
        self.model = node.model
        self.sombra = _Sombra(node)
        self.root = tk.Tk()
        self.root.title('SCARA - trayectorias (Gazebo)')
        self.root.configure(bg=FONDO)
        self.root.protocol('WM_DELETE_WINDOW', self.cerrar)
        # En una pantalla de 1366x768 la ventana entera tiene que caber sin que
        # GNOME la maximice: la planta encoge para dejar sitio a lo demas.
        self.lado = max(320, min(LADO_MAX, self.root.winfo_screenheight() - 330,
                                 self.root.winfo_screenwidth() - 830))

        self.vars = {c: {k: tk.StringVar(value=v) for k, _, v in campos}
                     for c, campos in CAMPOS.items()}
        self.puntos = {c: list(p) for c, p in PUNTOS.items()}
        self.var_camino = tk.StringVar(value='recta')
        self.var_perfil = tk.StringVar(value=node.perfil if node.perfil in PERFILES
                                       else 'quintico')
        self.var_codo = tk.StringVar(value=node.codo if node.codo in CODOS else 'auto')
        self.var_t = tk.StringVar(value='')
        self.var_orden = tk.StringVar(value='')
        self.var_mano = tk.StringVar(value='')
        self.entradas: dict[str, ttk.Entry] = {}
        self.clic = 0                  # que clic toca en los caminos de varios

        self.borrador: Borrador | None = None
        self.q_borrador: list[float] | None = None
        self.t_enviada = 0.0
        self._pendiente = None
        self.plan_visto = -1
        self.desvio_max = 0.0

        self._estilos()
        self._construir()
        self._fondo()
        for c, vs in self.vars.items():
            for v in vs.values():
                v.trace_add('write', lambda *a: self._pedir_comprobar())
        for v in (self.var_perfil, self.var_codo, self.var_t):
            v.trace_add('write', lambda *a: self._pedir_comprobar())
        self._poner_camino()
        self.root.bind('<Return>', lambda ev: self.ejecutar())
        self.root.bind('<KP_Enter>', lambda ev: self.ejecutar())
        self.root.bind('<Escape>', lambda ev: self.parar())
        self._tarea = self.root.after(50, self._refrescar)

    # ---------------------------------------------------------------- aspecto
    def _estilos(self):
        s = ttk.Style()
        if 'clam' in s.theme_names():
            s.theme_use('clam')
        s.configure('.', background=FONDO, foreground=TINTA)
        s.configure('TFrame', background=FONDO)
        s.configure('TLabel', background=FONDO, foreground=TINTA)
        s.configure('Suave.TLabel', foreground=SUAVE)
        s.configure('Titulo.TLabel', font=('TkDefaultFont', 13, 'bold'))
        s.configure('Dato.TLabel', font=('TkFixedFont', 10))
        s.configure('Estado.TLabel', font=('TkDefaultFont', 11, 'bold'))
        s.configure('Metodo.TLabel', font=('TkDefaultFont', 10, 'bold'))
        s.configure('TLabelframe', background=FONDO, bordercolor=BORDE)
        s.configure('TLabelframe.Label', background=FONDO, foreground=SUAVE)
        s.configure('Mal.TEntry', fieldbackground='#fbe3e1')
        s.configure('Toolbutton', padding=(8, 3), background='#e2e6ee')
        s.map('Toolbutton', background=[('selected', AZUL), ('active', '#dde3ec')],
              foreground=[('selected', 'white')])
        s.configure('Ir.TButton', font=('TkDefaultFont', 11, 'bold'),
                    foreground='white', background=AZUL, padding=6)
        s.map('Ir.TButton', background=[('active', '#2558a8'), ('disabled', '#a8b0bd')])
        s.configure('Parar.TButton', font=('TkDefaultFont', 11, 'bold'),
                    foreground='white', background=ROJO, padding=6)
        s.map('Parar.TButton', background=[('active', '#a8322a'), ('disabled', '#a8b0bd')])
        s.configure('Tray.Horizontal.TProgressbar', background=AZUL, troughcolor=PAPEL,
                    bordercolor=BORDE, lightcolor=AZUL, darkcolor=AZUL)

    def _construir(self):
        # Tres columnas: la planta | la orden nueva | atajos, estado y mensajes.
        # Es lo que cabe en 1366x768.
        raiz = ttk.Frame(self.root, padding=12)
        raiz.pack(fill='both', expand=True)
        raiz.columnconfigure(2, weight=1)
        raiz.rowconfigure(1, weight=1)

        cab = ttk.Frame(raiz)
        cab.grid(row=0, column=0, columnspan=3, sticky='ew', pady=(0, 8))
        ttk.Label(cab, text='Trayectorias', style='Titulo.TLabel').pack(side='left')
        via = ('   tray → aduana → brazo real   (Gazebo de espejo)'
               if self.n.destino == 'placa' else '   tray → Gazebo')
        ttk.Label(cab, text=via, style='Suave.TLabel').pack(side='left')
        self.lbl_tray = ttk.Label(cab, text='', style='Estado.TLabel')
        self.lbl_tray.pack(side='right')
        self.lbl_gz = ttk.Label(cab, text='', style='Estado.TLabel')
        self.lbl_gz.pack(side='right', padx=(14, 14))
        self.b_gz = ttk.Button(cab, text='Pausar Gazebo', width=14,
                               command=self._alternar_gz)
        self.b_gz.pack(side='right')

        self._construir_planta(raiz).grid(row=1, column=0, sticky='nw')
        self._construir_orden(raiz).grid(row=1, column=1, sticky='nw', padx=(12, 0))
        col = ttk.Frame(raiz)
        col.grid(row=1, column=2, sticky='nsew', padx=(12, 0))
        col.columnconfigure(0, weight=1)
        col.rowconfigure(2, weight=1)
        self._construir_atajos(col).grid(row=0, column=0, sticky='ew')
        self._construir_ahora(col).grid(row=1, column=0, sticky='ew', pady=(10, 0))
        self._construir_mensajes(col).grid(row=2, column=0, sticky='nsew', pady=(10, 0))

    def _construir_planta(self, padre):
        marco = ttk.Labelframe(padre, text=' Planta, vista desde arriba ', padding=8)
        self.lienzo = tk.Canvas(marco, width=self.lado, height=self.lado, bg=PAPEL,
                                highlightthickness=1, highlightbackground=BORDE, bd=0,
                                cursor='crosshair')
        self.lienzo.grid(row=0, column=0)
        self.regla = tk.Canvas(marco, width=ANCHO_Z, height=self.lado, bg=PAPEL,
                               highlightthickness=1, highlightbackground=BORDE, bd=0,
                               cursor='sb_v_double_arrow')
        self.regla.grid(row=0, column=1, padx=(6, 0))
        self.lienzo.bind('<Button-1>', self._clic)
        self.lienzo.bind('<Button-3>', self._deshacer)
        self.lienzo.bind('<Motion>', self._encima)
        self.lienzo.bind('<Leave>', lambda ev: self.lbl_encima.configure(text=''))
        self.regla.bind('<Button-1>', self._clic_z)

        ley = ttk.Frame(marco)
        ley.grid(row=1, column=0, columnspan=2, sticky='w', pady=(6, 0))
        for texto, color in (('━ plan de tray', NARANJA), ('━ la punta en Gazebo', VERDE),
                             ('┅ borrador', AZUL), ('┅ no sale', ROJO)):
            tk.Label(ley, text=texto, fg=color, bg=FONDO,
                     font=('TkDefaultFont', 9, 'bold')).pack(side='left', padx=(0, 12))
        ley2 = ttk.Frame(marco)
        ley2.grid(row=2, column=0, columnspan=2, sticky='w')
        tk.Label(ley2, text='■', fg=TONO_DOS, bg=FONDO,
                 font=('TkDefaultFont', 12)).pack(side='left')
        ttk.Label(ley2, text='alcanzable; más oscuro, con los dos codos (up y down)',
                  style='Suave.TLabel').pack(side='left')
        self.lbl_encima = ttk.Label(marco, text='', style='Dato.TLabel')
        self.lbl_encima.grid(row=3, column=0, columnspan=2, sticky='w', pady=(2, 0))
        return marco

    def _construir_mensajes(self, padre):
        marco = ttk.Labelframe(padre, text=' Mensajes de tray, y órdenes a mano ',
                               padding=8)
        marco.columnconfigure(0, weight=1)
        marco.rowconfigure(0, weight=1)
        self.registro = tk.Text(marco, height=8, width=40, font=('TkFixedFont', 9), bg=PAPEL,
                                fg=TINTA, relief='flat', highlightthickness=1,
                                highlightbackground=BORDE, wrap='word', state='disabled')
        self.registro.grid(row=0, column=0, columnspan=2, sticky='nsew')
        for tag, color in (('error', ROJO), ('aviso', NARANJA), ('tu', AZUL),
                           ('gz', VERDE), ('hora', SUAVE)):
            self.registro.tag_configure(tag, foreground=color)
        self.e_mano = ttk.Entry(marco, textvariable=self.var_mano)
        self.e_mano.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        self.e_mano.bind('<Return>', self._mandar_mano)
        self.e_mano.bind('<KP_Enter>', self._mandar_mano)
        ttk.Button(marco, text='Enviar', command=self._mandar_mano).grid(
            row=1, column=1, pady=(6, 0), padx=(6, 0))
        return marco

    def _construir_orden(self, padre):
        marco = ttk.Labelframe(padre, text=' Nueva trayectoria ', padding=10)
        marco.columnconfigure(0, weight=1)

        botones = ttk.Frame(marco)
        botones.grid(row=0, column=0, sticky='w')
        for k, (c, texto) in enumerate(CAMINOS):
            ttk.Radiobutton(botones, text=texto, value=c, variable=self.var_camino,
                            style='Toolbutton', command=self._poner_camino).grid(
                row=k // 4, column=k % 4, sticky='ew', padx=(0, 4), pady=(0, 4))

        self.lbl_ayuda = ttk.Label(marco, text='', style='Suave.TLabel', wraplength=380,
                                   justify='left')
        self.lbl_ayuda.grid(row=1, column=0, sticky='ew', pady=(4, 2))
        self.lbl_paso = ttk.Label(marco, text='', foreground=AZUL, wraplength=380)
        self.lbl_paso.grid(row=2, column=0, sticky='ew', pady=(0, 6))

        self.f_campos = ttk.Frame(marco)
        self.f_campos.grid(row=3, column=0, sticky='w')

        self.f_lista = ttk.Frame(marco)
        self.f_lista.grid(row=4, column=0, sticky='ew', pady=(6, 0))
        self.f_lista.columnconfigure(0, weight=1)
        self.lista = tk.Listbox(self.f_lista, height=4, font=('TkFixedFont', 9), bg=PAPEL,
                                fg=TINTA, relief='flat', highlightthickness=1,
                                highlightbackground=BORDE, activestyle='none')
        self.lista.grid(row=0, column=0, rowspan=3, sticky='ew')
        ttk.Button(self.f_lista, text='Añadir (x, y, z)', command=self._anadir).grid(
            row=0, column=1, sticky='ew', padx=(6, 0))
        ttk.Button(self.f_lista, text='Quitar el último', command=self._quitar).grid(
            row=1, column=1, sticky='ew', padx=(6, 0), pady=3)
        ttk.Button(self.f_lista, text='Vaciar', command=self._vaciar).grid(
            row=2, column=1, sticky='ew', padx=(6, 0))

        op = ttk.Frame(marco)
        op.grid(row=5, column=0, sticky='w', pady=(10, 0))
        ttk.Label(op, text='Perfil').pack(side='left')
        ttk.Combobox(op, textvariable=self.var_perfil, values=PERFILES, width=9,
                     state='readonly').pack(side='left', padx=(4, 12))
        ttk.Label(op, text='Codo').pack(side='left')
        ttk.Combobox(op, textvariable=self.var_codo, values=CODOS, width=8,
                     state='readonly').pack(side='left', padx=(4, 12))
        ttk.Label(op, text='t').pack(side='left')
        self.e_t = ttk.Entry(op, textvariable=self.var_t, width=6)
        self.e_t.pack(side='left', padx=(4, 2))
        ttk.Label(op, text='s', style='Suave.TLabel').pack(side='left')

        linea = ttk.Frame(marco)
        linea.grid(row=6, column=0, sticky='ew', pady=(10, 0))
        linea.columnconfigure(1, weight=1)
        ttk.Label(linea, text='Orden', style='Suave.TLabel').grid(row=0, column=0,
                                                                  padx=(0, 6))
        ttk.Entry(linea, textvariable=self.var_orden, state='readonly',
                  font=('TkFixedFont', 9)).grid(row=0, column=1, sticky='ew')

        # Alto fijo: un error largo no empuja al resto de la ventana fuera de la
        # pantalla. Lo que no cabe se corta en _veredicto.
        self.lbl_veredicto = tk.Label(marco, text='', bg=FONDO, fg=SUAVE, anchor='nw',
                                      justify='left', wraplength=380, height=3,
                                      font=('TkDefaultFont', 9, 'bold'))
        self.lbl_veredicto.grid(row=7, column=0, sticky='ew', pady=(6, 0))

        mandos = ttk.Frame(marco)
        mandos.grid(row=8, column=0, sticky='ew', pady=(10, 0))
        self.b_ir = ttk.Button(mandos, text='Ejecutar  (Enter)', style='Ir.TButton',
                               command=self.ejecutar)
        self.b_ir.pack(side='left')
        ttk.Button(mandos, text='PARAR  (Esc)', style='Parar.TButton',
                   command=self.parar).pack(side='right')
        return marco

    def _construir_atajos(self, padre):
        marco = ttk.Labelframe(padre, text=' Atajos ', padding=8)
        for k, (texto, cmd) in enumerate((
                ('Home', self.home), ('Perfiles', lambda: self._guia('perfiles')),
                ('Caminos', lambda: self._guia('caminos')),
                ('Demo', lambda: self._guia('demo')))):
            marco.columnconfigure(k, weight=1)
            ttk.Button(marco, text=texto, width=7, command=cmd).grid(
                row=0, column=k, sticky='ew', padx=(0 if k == 0 else 4, 0))
        return marco

    def _construir_ahora(self, padre):
        marco = ttk.Labelframe(padre, text=' Ahora ', padding=10)
        marco.columnconfigure(1, weight=1)
        self.lbl_metodo = ttk.Label(marco, text='--', style='Metodo.TLabel')
        self.lbl_metodo.grid(row=0, column=0, columnspan=2, sticky='w')
        self.barra = ttk.Progressbar(marco, maximum=100, style='Tray.Horizontal.TProgressbar')
        self.barra.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(4, 2))
        self.lbl_estado = ttk.Label(marco, text='', style='Suave.TLabel')
        self.lbl_estado.grid(row=2, column=0, columnspan=2, sticky='w', pady=(0, 6))
        self.datos = {}
        for k, (clave, texto) in enumerate((('juntas', 'Juntas'), ('punta', 'Punta'),
                                            ('desvio', 'Desvío'))):
            ttk.Label(marco, text=texto, style='Suave.TLabel').grid(
                row=3 + k, column=0, sticky='w', padx=(0, 10))
            self.datos[clave] = ttk.Label(marco, text='--', style='Dato.TLabel')
            self.datos[clave].grid(row=3 + k, column=1, sticky='w')
        return marco

    # ----------------------------------------------------------- la planta
    def _px(self, x: float, y: float) -> tuple[float, float]:
        L = self.lado
        s = L / (2.0 * VISTA)
        return L / 2.0 + x * s, L / 2.0 - y * s

    def _m(self, u: float, v: float) -> tuple[float, float]:
        L = self.lado
        s = L / (2.0 * VISTA)
        return (u - L / 2.0) / s, (L / 2.0 - v) / s

    def _ramas(self, x: float, y: float) -> int:
        """Cuantas ramas del codo llegan a (x, y) sin salirse de los limites."""
        m = self.model
        try:
            sols = m.ik_all(x, y, (m.z_min + m.z_max) / 2.0)
        except IKError:
            return 0
        lim = m.limits()[:2]
        return sum(1 for q in sols if all(lo <= v <= hi for v, (lo, hi) in zip(q, lim)))

    def _fondo(self):
        """El mapa de alcance, la rejilla y los ejes. Se pinta una vez."""
        c = self.lienzo
        n = self.lado // CELDA
        tonos = (TONO_FUERA, TONO_UNA, TONO_DOS)
        filas = []
        for i in range(n):
            fila = []
            for j in range(n):
                x, y = self._m((j + 0.5) * CELDA, (i + 0.5) * CELDA)
                fila.append(tonos[self._ramas(x, y)])
            filas.append('{' + ' '.join(fila) + '}')
        img = tk.PhotoImage(width=n, height=n)
        img.put(' '.join(filas))
        self.mapa = img.zoom(CELDA)
        c.create_image(0, 0, image=self.mapa, anchor='nw', tags='fondo')

        paso = 0.1
        k = -int(VISTA / paso)
        while k * paso <= VISTA + 1e-9:
            v = k * paso
            (u0, w0), (u1, w1) = self._px(v, -VISTA), self._px(v, VISTA)
            color = BORDE if k == 0 else REJILLA
            c.create_line(u0, w0, u1, w1, fill=color, tags='fondo')
            (u0, w0), (u1, w1) = self._px(-VISTA, v), self._px(VISTA, v)
            c.create_line(u0, w0, u1, w1, fill=color, tags='fondo')
            if k and k % 2 == 0:
                u, w = self._px(v, -VISTA)
                c.create_text(u, w - 8, text=f'{v:+.1f}', fill=SUAVE,
                              font=('TkDefaultFont', 7), tags='fondo')
                u, w = self._px(-VISTA, v)
                c.create_text(u + 16, w, text=f'{v:+.1f}', fill=SUAVE,
                              font=('TkDefaultFont', 7), tags='fondo')
            k += 1
        u, w = self._px(VISTA, 0.0)
        c.create_text(u - 10, w - 10, text='x', fill=SUAVE, tags='fondo')
        u, w = self._px(0.0, VISTA)
        c.create_text(u + 10, w + 10, text='y', fill=SUAVE, tags='fondo')
        hx, hy = self.n.home[0], self.n.home[1]
        u, w = self._px(hx, hy)
        c.create_line(u - 5, w, u + 5, w, fill=SUAVE, tags='fondo')
        c.create_line(u, w - 5, u, w + 5, fill=SUAVE, tags='fondo')
        c.create_text(u + 6, w + 9, text='home', fill=SUAVE, anchor='w',
                      font=('TkDefaultFont', 7), tags='fondo')
        c.create_text(8, 8, anchor='nw', fill=SUAVE, font=('TkDefaultFont', 8),
                      text='rejilla cada 0.1 m', tags='fondo')

        # La regla de z: la barra es el recorrido del husillo; a su derecha, lo
        # que ocupa en z el borrador (azul) y el plan (naranja); a su izquierda,
        # la punta.
        r = self.regla
        m = self.model
        for z in (m.z_min, m.z_max):
            y = self._pz(z)
            r.create_line(30, y, 58, y, fill=BORDE, tags='fondo')
            r.create_text(ANCHO_Z - 3, y + (10 if z == m.z_min else -10), anchor='e',
                          text=f'{z:.3f}', fill=SUAVE, font=('TkDefaultFont', 7),
                          tags='fondo')
        r.create_rectangle(34, self._pz(m.z_max), 40, self._pz(m.z_min), fill=TONO_DOS,
                           outline=BORDE, tags='fondo')
        r.create_text(ANCHO_Z / 2, 10, text='z (m)', fill=SUAVE,
                      font=('TkDefaultFont', 8), tags='fondo')
        r.create_text(ANCHO_Z / 2, self.lado - 10, text='clic: z', fill=SUAVE,
                      font=('TkDefaultFont', 7), tags='fondo')

    def _pz(self, z: float) -> float:
        """z (m) a px de la regla. Con margen arriba y abajo para las etiquetas."""
        m = self.model
        a, b = 40.0, self.lado - 40.0
        return b - (z - m.z_min) / max(m.z_max - m.z_min, 1e-9) * (b - a)

    def _zp(self, v: float) -> float:
        m = self.model
        a, b = 40.0, self.lado - 40.0
        return m.z_min + (b - v) / (b - a) * (m.z_max - m.z_min)

    def _linea(self, pts, **kw):
        if len(pts) < 2:
            return
        paso = max(1, len(pts) // 600)
        xy = []
        for p in pts[::paso] + [pts[-1]]:
            xy.extend(self._px(p[0], p[1]))
        self.lienzo.create_line(*xy, tags='dyn', **kw)

    def _marca(self, p, color, texto='', lleno=False):
        u, w = self._px(p[0], p[1])
        self.lienzo.create_oval(u - 5, w - 5, u + 5, w + 5, outline=color, width=2,
                                fill=color if lleno else '', tags='dyn')
        if texto:
            self.lienzo.create_text(u + 8, w - 8, text=texto, fill=color, anchor='w',
                                    font=('TkDefaultFont', 8, 'bold'), tags='dyn')

    def _pintar(self):
        c = self.lienzo
        c.delete('dyn')
        self.regla.delete('dyn')
        n = self.n
        b = self.borrador

        if b is not None:
            color = AZUL if b.ok else ROJO
            self._linea(b.aprox, fill=color, width=2, dash=(2, 4))
            self._linea(b.puntos, fill=color, width=2, dash=(6, 4))
            numerar = self.var_camino.get() in ('tres', 'poligonal', 'spline')
            for k, p in enumerate(b.via):
                self._marca(p, color, str(k + 1) if numerar else '')
        self._marcas_formulario()

        self._linea(list(n.plan), fill=NARANJA, width=3)
        for p in list(n.via):
            self._marca(p, NARANJA, lleno=True)
        self._linea(list(n.rastro), fill=VERDE, width=2)

        if n.q is not None:
            m = self.model
            q = n.q
            codo = (m.l1 * math.cos(q[0] + m.b1), m.l1 * math.sin(q[0] + m.b1))
            punta = m.fk(q)
            xy = [*self._px(0.0, 0.0), *self._px(*codo), *self._px(punta[0], punta[1])]
            c.create_line(*xy, fill=BRAZO, width=7, capstyle='round', joinstyle='round',
                          tags='dyn')
            for (x, y), r in (((0.0, 0.0), 7), (codo, 5)):
                u, w = self._px(x, y)
                c.create_oval(u - r, w - r, u + r, w + r, fill=PAPEL, outline=BRAZO,
                              width=2, tags='dyn')
            u, w = self._px(punta[0], punta[1])
            c.create_oval(u - 4, w - 4, u + 4, w + 4, fill=VERDE, outline=TINTA, tags='dyn')
            self._marca_z(punta[2], VERDE)

        # En la regla: por donde va a ir z (borrador y plan) y donde esta.
        if b is not None and b.puntos:
            zs = [p[2] for p in b.puntos]
            self._tramo_z(min(zs), max(zs), AZUL if b.ok else ROJO, 48)
        if n.plan:
            zs = [p[2] for p in n.plan]
            self._tramo_z(min(zs), max(zs), NARANJA, 55)

    def _tramo_z(self, z0, z1, color, u):
        y0, y1 = self._pz(z0), self._pz(z1)
        self.regla.create_line(u, y0, u, y1 if abs(y1 - y0) > 1 else y0 - 2, fill=color,
                               width=4, capstyle='round', tags='dyn')

    def _marca_z(self, z, color):
        y = self._pz(z)
        self.regla.create_polygon(20, y - 5, 20, y + 5, 31, y, fill=color, outline='',
                                  tags='dyn')
        self.regla.create_text(2, y - 11, anchor='w', text=f'{z:.3f}', fill=color,
                               font=('TkDefaultFont', 7), tags='dyn')

    def _marcas_formulario(self):
        """Lo del formulario que no es camino: el centro del arco, sobre todo."""
        c = self.var_camino.get()
        try:
            if c in ('arco', 'circulo'):
                u, w = self._px(self._num(c, 'cx', marcar=False),
                                self._num(c, 'cy', marcar=False))
                self.lienzo.create_line(u - 6, w - 6, u + 6, w + 6, fill=AZUL, width=2,
                                        tags='dyn')
                self.lienzo.create_line(u - 6, w + 6, u + 6, w - 6, fill=AZUL, width=2,
                                        tags='dyn')
        except ValueError:
            pass

    # ---------------------------------------------------------------- ratón
    def _encima(self, ev):
        x, y = self._m(ev.x, ev.y)
        k = self._ramas(x, y)
        que = ('fuera de alcance', 'alcanzable con un codo',
               'alcanzable con los dos codos')[k]
        self.lbl_encima.configure(
            text=f'x {x:+.3f}  y {y:+.3f} m   r {math.hypot(x, y):.3f} m   {que}',
            foreground=SUAVE if k else ROJO)

    def _clic(self, ev):
        x, y = (_redondear(v) for v in self._m(ev.x, ev.y))
        c = self.var_camino.get()
        V = self.vars[c]

        def pon(kx, ky):
            V[kx].set(f'{x:.3f}')
            V[ky].set(f'{y:.3f}')

        if c == 'recta':
            pon('x', 'y')
        elif c == 'junta':
            self._clic_junta(x, y)
        elif c == 'circulo':
            if self.clic == 0:
                pon('cx', 'cy')
            else:
                cx, cy = self._num(c, 'cx'), self._num(c, 'cy')
                V['r'].set(f'{_redondear(math.hypot(x - cx, y - cy)):.3f}')
            self.clic = (self.clic + 1) % 2
        elif c == 'arco':
            if self.clic == 0:
                pon('cx', 'cy')
            else:
                cx, cy = self._num(c, 'cx'), self._num(c, 'cy')
                a = math.degrees(math.atan2(y - cy, x - cx))
                if self.clic == 1:
                    V['r'].set(f'{_redondear(math.hypot(x - cx, y - cy)):.3f}')
                    V['a0'].set(f'{a:.0f}')
                else:
                    # El giro corto desde a0; para dar la vuelta larga, a mano.
                    a0 = self._num(c, 'a0')
                    giro = math.degrees(math.remainder(math.radians(a - a0), 2.0 * math.pi))
                    V['a1'].set(f'{a0 + giro:.0f}')
            self.clic = (self.clic + 1) % 3
        elif c == 'tres':
            k = self.clic + 1
            pon(f'x{k}', f'y{k}')
            self.clic = (self.clic + 1) % 3
        else:                                              # poligonal, spline
            pon('x', 'y')
            self._anadir()
        self._poner_paso()

    def _clic_junta(self, x: float, y: float):
        """Las q1, q2 que ponen la punta en (x, y), con la rama del codo elegida."""
        V = self.vars['junta']
        m = self.model
        try:
            d3 = self._num('junta', 'd3')
        except ValueError:
            d3 = (m.d3_min + m.d3_max) / 2.0
        z = m.z_ref + m.z_dir * min(max(d3, m.d3_min), m.d3_max)
        codo = self.var_codo.get()
        try:
            q = m.ik(x, y, z, elbow='nearest' if codo == 'auto' else codo, q_ref=self.n.q)
        except IKError as e:
            self._al_registro(f'ese punto no: {e}', 'aviso')
            return
        V['q1'].set(f'{math.degrees(q[0]):.1f}')
        V['q2'].set(f'{math.degrees(q[1]):.1f}')

    def _deshacer(self, ev=None):
        c = self.var_camino.get()
        if c in self.puntos:
            self._quitar()
        else:
            self.clic = 0
            self._poner_paso()

    def _clic_z(self, ev):
        z = min(max(self._zp(ev.y), self.model.z_min), self.model.z_max)
        c = self.var_camino.get()
        V = self.vars[c]
        if 'z' in V:
            V['z'].set(f'{z:.3f}')
        elif c == 'junta':
            m = self.model
            V['d3'].set(f'{(z - m.z_ref) / m.z_dir:.3f}')

    # ------------------------------------------------------------ formulario
    def _poner_camino(self):
        c = self.var_camino.get()
        for w in self.f_campos.winfo_children():
            w.destroy()
        self.entradas = {}
        cols = 2 if c == 'tres' else 3
        for k, (clave, unidad, _) in enumerate(CAMPOS[c]):
            fila, col = divmod(k, cols)
            ttk.Label(self.f_campos, text=clave).grid(
                row=fila, column=3 * col, sticky='e', padx=(0 if col == 0 else 12, 4), pady=2)
            e = ttk.Entry(self.f_campos, textvariable=self.vars[c][clave], width=8)
            e.grid(row=fila, column=3 * col + 1, sticky='w', pady=2)
            ttk.Label(self.f_campos, text=unidad, style='Suave.TLabel').grid(
                row=fila, column=3 * col + 2, sticky='w', padx=(2, 0))
            self.entradas[clave] = e
        if c in self.puntos:
            self.f_lista.grid()
            self._llenar_lista()
        else:
            self.f_lista.grid_remove()
        self.clic = 0
        self.lbl_ayuda.configure(text=AYUDA[c])
        self._poner_paso()
        self._pedir_comprobar()

    def _poner_paso(self):
        pasos = PASOS[self.var_camino.get()]
        n = f' ({self.clic + 1}/{len(pasos)})' if len(pasos) > 1 else ''
        self.lbl_paso.configure(text=f'Clic en la planta{n}: {pasos[self.clic]}.')

    def _llenar_lista(self):
        c = self.var_camino.get()
        self.lista.delete(0, 'end')
        for k, (x, y, z) in enumerate(self.puntos.get(c, [])):
            self.lista.insert('end', f'{k + 1:2d}   x {x:+.3f}   y {y:+.3f}   z {z:.3f}')
        self.lista.see('end')

    def _anadir(self):
        c = self.var_camino.get()
        try:
            p = (self._num(c, 'x'), self._num(c, 'y'), self._num(c, 'z'))
        except ValueError:
            return
        self.puntos[c].append(p)
        self._llenar_lista()
        self._pedir_comprobar()

    def _quitar(self):
        c = self.var_camino.get()
        if self.puntos.get(c):
            self.puntos[c].pop()
            self._llenar_lista()
            self._pedir_comprobar()

    def _vaciar(self):
        c = self.var_camino.get()
        if c in self.puntos:
            self.puntos[c].clear()
            self._llenar_lista()
            self._pedir_comprobar()

    def _num(self, c: str, clave: str, marcar: bool = True) -> float:
        txt = self.vars[c][clave].get().strip().replace(',', '.')
        e = self.entradas.get(clave) if c == self.var_camino.get() else None
        try:
            v = float(txt)
            if not math.isfinite(v):
                raise ValueError
        except ValueError:
            if marcar and e is not None:
                e.configure(style='Mal.TEntry')
            raise ValueError(f'{clave}: «{txt}» no es un número') from None
        if marcar and e is not None:
            e.configure(style='TEntry')
        return v

    def _opciones(self) -> str:
        op = f' perfil={self.var_perfil.get()} codo={self.var_codo.get()}'
        txt = self.var_t.get().strip().replace(',', '.')
        if txt:
            try:
                t = float(txt)
                if not t > 0.0:
                    raise ValueError
            except ValueError:
                self.e_t.configure(style='Mal.TEntry')
                raise ValueError(f't: «{txt}» tiene que ser un número de segundos, '
                                 f'o nada para que la calcule tray') from None
            op += f' t={t:g}'
        self.e_t.configure(style='TEntry')
        return op

    def _orden(self) -> str:
        """La orden de texto que se manda, tal cual se teclearia a mano."""
        c = self.var_camino.get()
        errores = []

        def n(clave):
            try:
                return self._num(c, clave)
            except ValueError as e:
                errores.append(str(e))
                return 0.0

        if c == 'recta':
            cuerpo = f'recta {n("x"):.3f} {n("y"):.3f} {n("z"):.3f}'
        elif c == 'junta':
            cuerpo = (f'junta {math.radians(n("q1")):.4f} {math.radians(n("q2")):.4f} '
                      f'{n("d3"):.3f}')
        elif c == 'arco':
            cuerpo = (f'arco {n("cx"):.3f} {n("cy"):.3f} {n("r"):.3f} '
                      f'{n("a0"):g} {n("a1"):g} {n("z"):.3f}')
        elif c == 'circulo':
            cuerpo = (f'circulo {n("cx"):.3f} {n("cy"):.3f} {n("r"):.3f} {n("z"):.3f} '
                      f'{n("vueltas"):g}')
        elif c == 'tres':
            cuerpo = 'tres ' + ' '.join(f'{n(k):.3f}' for k in
                                        ('x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'z'))
        else:
            pts = self.puntos[c]
            if len(pts) < 2:
                errores.append(f'la {c} necesita al menos dos puntos: clic en la planta')
            cuerpo = c + ' ' + '  '.join(f'{x:.3f} {y:.3f} {z:.3f}' for x, y, z in pts)
        try:
            op = self._opciones()
        except ValueError as e:
            errores.append(str(e))
            op = ''
        if errores:
            raise ValueError(' · '.join(errores))
        return cuerpo + op

    # --------------------------------------------------------------- borrador
    def _pedir_comprobar(self):
        if self._pendiente is not None:
            self.root.after_cancel(self._pendiente)
        self._pendiente = self.root.after(150, self._comprobar)

    def _comprobar(self):
        self._pendiente = None
        q0 = self.n.q
        self.q_borrador = list(q0) if q0 is not None else None
        try:
            texto = self._orden()
        except ValueError as e:
            self.var_orden.set('')
            self.borrador = None
            self._veredicto(f'✗ {e}', ROJO)
            return
        self.var_orden.set(texto)
        if q0 is None:
            self.borrador = None
            self._veredicto('Sin /joint_states todavía: no puedo comprobarla. tray la '
                            'mirará cuando llegue.', NARANJA)
            return
        self.borrador = self.planificar(texto, list(q0))
        self._veredicto(('✓ ' if self.borrador.ok else '✗ ') + self.borrador.detalle,
                        VERDE if self.borrador.ok else ROJO)

    def planificar(self, texto: str, q0: list[float]) -> Borrador:
        """Lo mismo que `Planificador._lanzar`, pero sin mandar nada.

        Parser, eleccion de camino, techo de velocidad y tramo de aproximacion
        son los de tray.py, llamados con la sombra en el sitio de `self`.
        """
        s, n, m = self.sombra, self.n, self.model
        cam = None
        try:
            verbo, nums, op = Planificador._partir(s, texto)
            pf = T.perfil(op.get('perfil', n.perfil))
            codo = op.get('codo', n.codo)
            dur = float(op['t']) if 't' in op else (n.t_fijo or None)
            cam, via = Planificador._camino(s, verbo, nums,
                                            'nearest' if codo == 'auto' else codo, q0)
            ramas = ('nearest', 'up', 'down') if codo == 'auto' else (codo,)
            techo = Planificador._techo(s)
            tr = T.planificar(cam, pf, q0, techo, dur, ramas, muestras=n.muestras)
        except (ValueError, IKError) as e:
            return Borrador(texto, False, self._intencion(cam, q0),
                            cam.via() if cam is not None else [],
                            detalle=f'no sale: {_resumir(str(e))}')

        puntos = [m.fk(q)[:3] for q in tr.tabla[::2]]
        detalle = (f'sale: {tr.camino.nombre} con perfil {pf.nombre}, '
                   f'T = {tr.duracion:.2f} s ({"la pones tú" if dur else "la calcula tray"}), '
                   f'codo {getattr(tr.camino, "codo", "-")}')
        if verbo == 'junta' and m.clamp(nums) != nums:
            # tray no se queja: recorta en silencio (m.clamp en _camino).
            topes = [f'd3 a {v:.3f} m' if i == 2 else f'q{i + 1} a {math.degrees(v):+.1f}°'
                     for i, (v, w) in enumerate(zip(m.clamp(nums), nums)) if v != w]
            detalle += ('. Ojo: se sale de los límites y tray la deja en el tope: '
                        + ', '.join(topes))
        aprox = []
        salto = max(abs(a - b) for a, b in zip(q0, tr.tabla[0]))
        if salto > n.aprox:
            try:
                ap = T.planificar(T.CaminoJunta(q0, tr.tabla[0]), pf, q0, techo, dur,
                                  muestras=n.muestras)
                aprox = [m.fk(q)[:3] for q in ap.tabla[::4]]
                detalle += f', y antes {ap.duracion:.2f} s en juntas hasta el principio'
            except (ValueError, IKError):
                pass
        if n.ocupado():
            detalle += '. tray está ocupado: irá a la cola, y saldrá de donde acabe'
        return Borrador(texto, True, puntos, via, aprox, detalle)

    def _intencion(self, cam, q0) -> list:
        """Por donde iria el camino si la IK llegase: para ver donde se sale."""
        if cam is None:
            return []
        m = self.model
        if hasattr(cam, 'p'):
            return [cam.p(k / 200.0) for k in range(201)]
        if isinstance(cam, T.CaminoJunta):
            return [m.fk(cam.q(k / 100.0, q0))[:3] for k in range(101)]
        return []

    def _veredicto(self, texto: str, color: str):
        if len(texto) > 170:
            texto = texto[:168].rstrip() + '…'
        self.lbl_veredicto.configure(text=texto, fg=color)

    # ---------------------------------------------------------------- ordenes
    def ejecutar(self):
        self._comprobar()
        b = self.borrador
        if b is not None and not b.ok:
            return
        texto = self.var_orden.get()
        if not texto:
            return
        self.n.mandar(texto)
        self.borrador = None
        self.q_borrador = None         # que se rehaga en cuanto tray acabe
        self.t_enviada = time.monotonic()
        self._veredicto('Mandada. Cuando tray acabe, aquí sale el borrador de la '
                        'siguiente.', SUAVE)

    def parar(self):
        self.n.mandar('parar')

    def home(self):
        try:
            op = self._opciones()
        except ValueError:
            op = f' perfil={self.var_perfil.get()} codo={self.var_codo.get()}'
        self.n.mandar('home' + op)
        self.t_enviada = time.monotonic()

    def _guia(self, cual: str):
        # Sin opciones: tray se las pegaria a cada orden de la guia y el tour de
        # perfiles dejaria de ensenar los cinco.
        self.n.mandar(cual)
        self.t_enviada = time.monotonic()

    def _mandar_mano(self, ev=None):
        texto = self.var_mano.get().strip()
        if texto:
            self.n.mandar(texto)
            self.var_mano.set('')
            self.t_enviada = time.monotonic()
        return 'break'                 # que el Enter no ejecute tambien el formulario

    def _alternar_gz(self):
        self.n.pausar(self.n.gazebo() == 'corriendo')

    def cerrar(self):
        self.root.after_cancel(self._tarea)
        self.root.destroy()

    # --------------------------------------------------------------- refresco
    def _refrescar(self):
        # Ctrl+C en el launch apaga rclpy, pero eso no saca al mainloop de Tk.
        if not rclpy.ok():
            self.root.destroy()
            return
        self._tarea = self.root.after(50, self._refrescar)
        n = self.n
        while True:
            try:
                nivel, texto = n.log.get_nowait()
            except queue.Empty:
                break
            if nivel == 'tu':
                self._al_registro(f'tú: {texto}', 'tu')
            elif nivel == 'gz':
                self._al_registro(texto, 'gz')
            else:
                self._al_registro(texto, 'error' if nivel >= 40 else
                                  'aviso' if nivel >= 30 else None)

        gz = n.gazebo()
        vivo = n.vivo(n.t_estado)
        ocupado = n.ocupado()
        self.lbl_gz.configure(
            text={'corriendo': 'Gazebo: corriendo', 'pausa': 'Gazebo: en pausa',
                  'no': 'Gazebo: sin reloj'}[gz],
            foreground={'corriendo': VERDE, 'pausa': NARANJA, 'no': ROJO}[gz])
        if gz == 'pausa' and n.destino != 'placa':
            texto, color = 'tray: parado con Gazebo', NARANJA
        elif not vivo:
            texto, color = 'tray: no está', ROJO
        elif ocupado:
            texto, color = 'tray: moviendo', AZUL
        else:
            texto, color = 'tray: en espera', VERDE
        self.lbl_tray.configure(text=texto, foreground=color)
        puede_gz = gz != 'no' and n.destino != 'placa'
        self.b_gz.configure(text='Seguir Gazebo' if gz == 'pausa' else 'Pausar Gazebo')
        self.b_gz.state(['!disabled'] if puede_gz else ['disabled'])

        # El borrador sale de la pose de ahora: se rehace cuando el brazo se ha
        # quedado quieto en otra. Mientras tray se mueve, se deja como estaba.
        if (not ocupado and time.monotonic() - self.t_enviada > 1.0
                and self._pendiente is None and n.q is not None
                and (self.q_borrador is None
                     or max(abs(a - b) for a, b in zip(n.q, self.q_borrador)) > 2e-3)):
            self._comprobar()

        b = self.borrador
        mal = b is not None and not b.ok or not self.var_orden.get()
        self.b_ir.state(['disabled'] if mal else ['!disabled'])
        self.b_ir.configure(text='A la cola  (Enter)' if ocupado else 'Ejecutar  (Enter)')

        estado = n.estado if (vivo or gz == 'pausa') else ''
        self.lbl_metodo.configure(text=n.metodo or '--')
        m = ORDEN_T.search(estado)
        self.barra['value'] = int(m.group(1)) if m and ocupado else (100 if n.plan else 0)
        self.lbl_estado.configure(text=estado or 'sin noticias de tray')

        if n.q is not None:
            q = n.q
            p = self.model.fk(q)
            self.datos['juntas'].configure(
                text=f'q1 {math.degrees(q[0]):+6.1f}°  q2 {math.degrees(q[1]):+6.1f}°  '
                     f'd3 {q[2] * 1000:5.1f} mm')
            self.datos['punta'].configure(
                text=f'x {p[0] * 1000:+6.1f}  y {p[1] * 1000:+6.1f}  '
                     f'z {p[2] * 1000:5.1f} mm')
            self._desvio(p[:3])
        self._pintar()

    def _desvio(self, p):
        """Lo que se separa la punta de Gazebo del plan de tray, en mm."""
        n = self.n
        if n.plan_n != self.plan_visto:
            self.plan_visto = n.plan_n
            self.desvio_max = 0.0
        plan = n.plan
        if len(plan) < 2:
            self.datos['desvio'].configure(text='--')
            return
        d = min(_a_segmento(p, a, b) for a, b in zip(plan, plan[1:]))
        if n.ocupado():
            self.desvio_max = max(self.desvio_max, d)
        self.datos['desvio'].configure(
            text=f'{d * 1000:5.1f} mm del plan  (máx. {self.desvio_max * 1000:.1f})')

    def _al_registro(self, texto: str, tag: str | None = None):
        r = self.registro
        r.configure(state='normal')
        r.insert('end', time.strftime('%H:%M:%S  '), 'hora')
        r.insert('end', texto.strip() + '\n', tag or ())
        if int(r.index('end-1c').split('.')[0]) > 400:
            r.delete('1.0', '100.0')
        r.see('end')
        r.configure(state='disabled')


def _redondear(v: float) -> float:
    return round(v / REDONDEO) * REDONDEO


def _resumir(error: str) -> str:
    """El 'ninguna rama sirve' de planificar, en corto.

    Trae un motivo por rama ('codo=nearest: ... | codo=up: ... | ...'), y casi
    siempre es el mismo tres veces: basta con decirlo una.
    """
    cabeza, _, resto = error.partition('. ')
    partes = resto.split(' | ')
    if len(partes) < 2:
        return error
    motivos = [p.partition(': ')[2] or p for p in partes]
    if all(m == motivos[0] for m in motivos):
        return f'{cabeza}; con las {len(partes)} ramas: {motivos[0]}'
    return f'{cabeza}. {partes[0]} (y {len(partes) - 1} ramas más)'


def _a_segmento(p, a, b) -> float:
    """Distancia de p al segmento a-b, en 3D."""
    ab = [bb - aa for aa, bb in zip(a, b)]
    ap = [pp - aa for aa, pp in zip(a, p)]
    L = sum(v * v for v in ab)
    t = 0.0 if L <= 0.0 else min(max(sum(u * v for u, v in zip(ap, ab)) / L, 0.0), 1.0)
    return math.dist(p, [aa + t * v for aa, v in zip(a, ab)])


def _girar(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:             # noqa: B902
        # Al cerrar la ventana, rclpy.shutdown() pilla al spin a mitad de una
        # espera y rcl protesta; eso no es un fallo.
        if rclpy.ok():
            raise


def main(args=None):
    rclpy.init(args=args)
    node = TrayGuiNode()
    hilo = threading.Thread(target=_girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaTray(node)
    except tk.TclError as exc:
        node.get_logger().error(f'no puedo abrir la ventana ({exc}). ¿Hay DISPLAY?')
        rclpy.shutdown()
        return 1

    try:
        ventana.root.mainloop()
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    hilo.join(timeout=2.0)
    node.destroy_node()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
