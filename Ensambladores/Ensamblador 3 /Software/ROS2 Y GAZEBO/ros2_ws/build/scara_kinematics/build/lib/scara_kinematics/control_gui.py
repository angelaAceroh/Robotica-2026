"""La ventana del control cinematico: `control_nodo`, con Gazebo delante.

    ros2 launch scara_kinematics control_gui.launch.py
    ros2 launch scara_kinematics control_gui.launch.py tarea:=perforado k:=10

Es la cara de `control_nodo`. No calcula ni mueve nada: compone la misma orden
de texto que se mandaria a mano por scara/control/orden y la publica. Lo que
anade es VER el lazo mientras trabaja:

    la planta   el camino pedido (verde), lo que hace la punta (naranja), la
                consigna de ahora y hasta donde llega el brazo (sombreado)
    z           la regla del husillo, con la consigna y la punta
    el error    |xd - x| de los ultimos segundos, en rojo donde la ley saturo
    mensajes    lo que dice control_nodo: la tau medida, el error al acabar...

Y quita el problema de `ros2 topic pub --once`: cada uno es un participante de
DDS que entra y sale (control_nodo se queda parado 0.3-0.4 s cada vez) y alguna
orden se pierde. La ventana entra una vez y se queda, y no manda nada hasta que
control_nodo la escucha.

    Esc          parar
    clic         en la planta: X, Y del punto de «Ir / Salto»
    clic dcho.   en la planta: el centro del circulo

Entradas y salidas:

    scara/control/orden              std_msgs/String       lo que se manda (lo unico)
    scara/control/estado             std_msgs/String       que hace y por donde va
    scara/control/deseado            nav_msgs/Path         el camino pedido
    scara/control/datos/punta_real   geometry_msgs/Vector3 la punta medida [mm]
    scara/control/datos/error        geometry_msgs/Vector3 xd - x [mm]
    scara/control/datos/saturacion   std_msgs/Float64      > 1: la ley recorto
    /rosout                          rcl_interfaces/Log    los mensajes de control

Y a Gazebo, por fuera de ROS, `gz service /world/<mundo>/control` para pausarlo y
seguir. Con Gazebo en pausa el control se para tambien: corre con su reloj.
"""
from __future__ import annotations

import collections
import math
import queue
import re
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from scara_kinematics import control_cinematico as C
from scara_kinematics import ruta as R
from scara_kinematics.estilo import (AZUL, BORDE, FONDO, NARANJA, PAPEL, REJILLA,
                                     ROJO, SUAVE, TINTA, VERDE)
from scara_kinematics.model import declare_model_params

VIVO_S = 1.0             # sin estado en este tiempo, control_nodo se da por caido

# La planta ensena la zona de trabajo, no el anillo entero: las rutas y el
# circulo caben y un milimetro es algo mas de un pixel.
VISTA_X = (140.0, 510.0)  # mm
VISTA_Y = (-185.0, 185.0)  # mm
LADO_MAX = 400           # px del lienzo de la planta, si cabe en la pantalla
CELDA = 2                # px por celda del mapa de alcance
ANCHO_Z = 64             # la regla de z, a la derecha de la planta
ALTO_ERROR = 112         # px de la grafica del error
VENTANA_S = 15.0         # s (de reloj) que ensena la grafica del error
E_MIN = 1e-3             # mm: el suelo de la grafica (escala logaritmica)

TONO_FUERA = PAPEL
TONO_OTRA = '#edf1f7'    # solo con la otra rama del codo: el control no cambia de rama
TONO_DOWN = '#d3ddec'    # con la rama del reposo (down)
VERDE_CLARO = '#9fd8b9'

ORDEN_T = re.compile(r'(\d+)\s*%')
K_T = re.compile(r'\| K ([\d.,\s]+?)(?:\s*\||$)')
AYUDA_PLANTA = 'clic: X, Y de «Ir a un punto»   ·   clic dcho.: centro del círculo'


class ControlGuiNode(Node):

    def __init__(self):
        super().__init__('control_gui')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        self.k = float(self.declare_parameter('k', C.K_DEF).value)
        self.perfil = str(self.declare_parameter('perfil', 'trapecio').value)
        # vacio = lo busca en `gz service -l` la primera vez que hace falta
        self.mundo = str(self.declare_parameter('mundo', '').value)

        self.estado = ''
        self.t_estado = 0.0
        self.deseado: list[tuple] = []         # el camino pedido [mm]
        self.deseado_n = 0                      # sube con cada camino nuevo
        self.traza: list[tuple] = []           # lo que hizo la punta en esta tarea [mm]
        self.punta: tuple | None = None
        self.error: tuple | None = None
        self.sat = 0.0
        self.err_max = 0.0
        self.serie: collections.deque = collections.deque(maxlen=4000)  # (t, |e|, sat)
        self.log: queue.Queue = queue.Queue()
        self.pausa: bool | None = None          # lo ultimo que se le pidio a gz

        self.orden_pub = self.create_publisher(String, 'scara/control/orden', 10)
        self.create_subscription(String, 'scara/control/estado', self.on_estado, 10)
        self.create_subscription(Path, 'scara/control/deseado', self.on_deseado, 10)
        self.create_subscription(Vector3, 'scara/control/datos/punta_real', self.on_punta, 10)
        self.create_subscription(Vector3, 'scara/control/datos/error', self.on_error, 10)
        self.create_subscription(Float64, 'scara/control/datos/saturacion', self.on_sat, 10)
        # /rosout se publica transient_local: asi llegan tambien los ultimos
        # mensajes que control dijo antes de abrirse la ventana.
        self.create_subscription(
            Log, '/rosout', self.on_rosout,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    # -------------------------------------------------------------- entradas
    def on_estado(self, msg: String):
        self.estado = msg.data
        self.t_estado = time.monotonic()

    def on_deseado(self, msg: Path):
        self.deseado = [(p.pose.position.x * 1e3, p.pose.position.y * 1e3,
                         p.pose.position.z * 1e3) for p in msg.poses]
        self.deseado_n += 1
        self.traza = []                 # la naranja es la de esta tarea
        self.err_max = 0.0

    def on_punta(self, msg: Vector3):
        p = (msg.x, msg.y, msg.z)
        self.punta = p
        t = self.traza
        if not t or math.dist(t[-1], p) > 0.3:
            t.append(p)
            if len(t) > 8000:
                del t[:2000]

    def on_error(self, msg: Vector3):
        self.error = (msg.x, msg.y, msg.z)
        e = math.sqrt(msg.x ** 2 + msg.y ** 2 + msg.z ** 2)
        self.err_max = max(self.err_max, e)
        self.serie.append((time.monotonic(), e, self.sat))

    def on_sat(self, msg: Float64):
        self.sat = msg.data

    def on_rosout(self, msg: Log):
        if msg.name == 'control':
            self.log.put((int(msg.level), msg.msg))

    # --------------------------------------------------------------- estados
    def vivo(self) -> bool:
        return time.monotonic() - self.t_estado < VIVO_S

    def escucha(self) -> bool:
        """control_nodo esta suscrito a las ordenes: lo que se mande, le llega."""
        return self.orden_pub.get_subscription_count() > 0

    # --------------------------------------------------------------- ordenes
    def mandar(self, texto: str) -> bool:
        if not self.escucha():
            self.log.put((40, f'«{texto}» no sale: control_nodo no escucha todavía'))
            return False
        self.orden_pub.publish(String(data=texto))
        self.log.put(('tu', texto))
        if texto != 'parar' and not texto.startswith('k'):
            self.err_max = 0.0          # el maximo es el de esta orden
        return True

    def pausar(self, si: bool):
        """Pausa o sigue Gazebo. En un hilo: `gz service` tarda un par de segundos."""
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
            self.pausa = si
            self.log.put(('gz', 'Gazebo en pausa (el control se para con él)' if si
                          else 'Gazebo sigue'))
        else:
            self.log.put((40, f'Gazebo no ha hecho caso: {(r.stdout + r.stderr).strip()}'))


class VentanaControl:

    def __init__(self, node: ControlGuiNode):
        self.n = node
        self.robot = node.robot
        self.root = tk.Tk()
        self.root.title('SCARA - control cinemático (Gazebo)')
        self.root.configure(bg=FONDO)
        self.root.protocol('WM_DELETE_WINDOW', self.cerrar)
        # 1366x768: la ventana entera tiene que caber sin que GNOME la maximice.
        self.lado = max(300, min(LADO_MAX, self.root.winfo_screenheight() - 410))
        self.esc = self.lado / max(VISTA_X[1] - VISTA_X[0], VISTA_Y[1] - VISTA_Y[0])
        z = [self.robot.z_ref + self.robot.z_dir * d for d in self.robot.lim[2]]
        self.z_lim = (min(z), max(z))

        self.v = {}
        for clave, valor in (('x', '330'), ('y', '60'), ('z', '150'), ('v', '40'),
                             ('cx', '330'), ('cy', '0'), ('r', '40'), ('cz', '150'),
                             ('cv', '20'), ('vueltas', '1'),
                             ('dx', '0'), ('dy', '15'), ('dz', '0'),
                             ('k', f'{node.k:g}')):
            self.v[clave] = tk.StringVar(value=valor)
        self.var_perfil = tk.StringVar(
            value=node.perfil if node.perfil in R.PERFILES else 'trapecio')
        self.var_mano = tk.StringVar(value='')
        self.entradas: dict[str, ttk.Entry] = {}
        self.botones: list[ttk.Button] = []
        self.deseado_visto = -1

        self._estilos()
        self._construir()
        self._fondo()
        self.v['k'].trace_add('write', lambda *a: self._tau())
        self._tau()
        self.root.bind('<Escape>', lambda ev: self.n.mandar('parar'))
        self._tarea = self.root.after(100, self._refrescar)

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
        s.configure('TLabelframe', background=FONDO, bordercolor=BORDE)
        s.configure('TLabelframe.Label', background=FONDO, foreground=SUAVE)
        s.configure('Mal.TEntry', fieldbackground='#fbe3e1')
        s.configure('Ir.TButton', font=('TkDefaultFont', 10, 'bold'),
                    foreground='white', background=AZUL, padding=4)
        s.map('Ir.TButton', background=[('active', '#2558a8'), ('disabled', '#a8b0bd')])
        s.configure('Parar.TButton', font=('TkDefaultFont', 11, 'bold'),
                    foreground='white', background=ROJO, padding=6)
        s.map('Parar.TButton', background=[('active', '#a8322a'), ('disabled', '#a8b0bd')])
        s.configure('Ctrl.Horizontal.TProgressbar', background=AZUL, troughcolor=PAPEL,
                    bordercolor=BORDE, lightcolor=AZUL, darkcolor=AZUL)

    def _construir(self):
        # Tres columnas: la planta y el error | las ordenes | ahora y mensajes.
        raiz = ttk.Frame(self.root, padding=10)
        raiz.pack(fill='both', expand=True)
        raiz.columnconfigure(2, weight=1)
        raiz.rowconfigure(1, weight=1)

        cab = ttk.Frame(raiz)
        cab.grid(row=0, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        ttk.Label(cab, text='Control cinemático', style='Titulo.TLabel').pack(side='left')
        ttk.Label(cab, text='   q̇ = J⁻¹(ẋd + K·e)  →  Gazebo, juntas por velocidad',
                  style='Suave.TLabel').pack(side='left')
        b = ttk.Button(cab, text='Parar  (Esc)', style='Parar.TButton', width=14,
                       command=lambda: self.n.mandar('parar'))
        b.pack(side='right')
        self.botones.append(b)
        self.b_gz = ttk.Button(cab, text='Pausar Gazebo', width=14,
                               command=self._alternar_gz)
        self.b_gz.pack(side='right', padx=(0, 10))
        self.lbl_ctrl = ttk.Label(cab, text='', style='Estado.TLabel', width=30, anchor='e')
        self.lbl_ctrl.pack(side='right', padx=(0, 14))

        col = ttk.Frame(raiz)
        col.grid(row=1, column=0, sticky='nw')
        self._construir_planta(col).grid(row=0, column=0, sticky='nw')
        self._construir_error(col).grid(row=1, column=0, sticky='nw', pady=(6, 0))
        self._construir_ordenes(raiz).grid(row=1, column=1, sticky='nw', padx=(10, 0))
        der = ttk.Frame(raiz)
        der.grid(row=1, column=2, sticky='nsew', padx=(10, 0))
        der.columnconfigure(0, weight=1)
        der.rowconfigure(2, weight=1)
        self._construir_ahora(der).grid(row=0, column=0, sticky='ew')
        self._construir_regulacion(der).grid(row=1, column=0, sticky='ew', pady=(6, 0))
        self._construir_mensajes(der).grid(row=2, column=0, sticky='nsew', pady=(6, 0))

    def _construir_planta(self, padre):
        marco = ttk.Labelframe(padre, text=' Planta, desde arriba (mm) ', padding=6)
        self.lienzo = tk.Canvas(marco, width=self.lado, height=self.lado, bg=PAPEL,
                                highlightthickness=1, highlightbackground=BORDE, bd=0,
                                cursor='crosshair')
        self.lienzo.grid(row=0, column=0)
        self.regla = tk.Canvas(marco, width=ANCHO_Z, height=self.lado, bg=PAPEL,
                               highlightthickness=1, highlightbackground=BORDE, bd=0)
        self.regla.grid(row=0, column=1, padx=(6, 0))
        self.lienzo.bind('<Button-1>', lambda ev: self._clic(ev, 'x', 'y'))
        self.lienzo.bind('<Button-3>', lambda ev: self._clic(ev, 'cx', 'cy'))
        self.lienzo.bind('<Motion>', self._encima)
        self.lienzo.bind('<Leave>', lambda ev: self.lbl_encima.configure(
            text=AYUDA_PLANTA, style='Suave.TLabel'))

        ley = ttk.Frame(marco)
        ley.grid(row=1, column=0, columnspan=2, sticky='w', pady=(2, 0))
        for texto, color in (('━ pedido', VERDE), ('━ la punta en Gazebo', NARANJA),
                             ('○ consigna', VERDE), ('┅ lo que mandarías', AZUL)):
            tk.Label(ley, text=texto, fg=color, bg=FONDO,
                     font=('TkDefaultFont', 9, 'bold')).pack(side='left', padx=(0, 10))
        self.lbl_encima = ttk.Label(marco, text=AYUDA_PLANTA, style='Suave.TLabel')
        self.lbl_encima.grid(row=2, column=0, columnspan=2, sticky='w')
        return marco

    def _construir_error(self, padre):
        marco = ttk.Labelframe(padre, text=f' |error| de la punta, últimos {VENTANA_S:.0f} s '
                                           f'(log.: una caída exponencial es una recta) ',
                               padding=6)
        self.graf = tk.Canvas(marco, width=self.lado + ANCHO_Z + 6, height=ALTO_ERROR,
                              bg=PAPEL, highlightthickness=1, highlightbackground=BORDE, bd=0)
        self.graf.grid(row=0, column=0)
        return marco

    def _campo(self, padre, fila, col, clave, etiqueta, ancho=6):
        ttk.Label(padre, text=etiqueta).grid(row=fila, column=col, sticky='e', padx=(6, 2))
        e = ttk.Entry(padre, textvariable=self.v[clave], width=ancho, justify='right')
        e.grid(row=fila, column=col + 1, sticky='w', pady=1)
        self.entradas[clave] = e
        return e

    def _boton(self, padre, texto, accion, estilo='TButton', **grid):
        b = ttk.Button(padre, text=texto, command=accion, style=estilo)
        b.grid(**grid)
        self.botones.append(b)
        return b

    def _construir_ordenes(self, padre):
        col = ttk.Frame(padre)

        t = ttk.Labelframe(col, text=' Tareas ', padding=6)
        t.grid(row=0, column=0, sticky='ew')
        for i, (texto, orden) in enumerate((('Home', 'home'), ('Perforado', 'perforado'),
                                            ('Contorno', 'contorno'))):
            self._boton(t, texto, lambda o=orden: self._tarea_ruta(o),
                        row=0, column=i, padx=2, sticky='ew')
        fp = ttk.Frame(t)
        fp.grid(row=1, column=0, columnspan=3, sticky='w', pady=(6, 0))
        ttk.Label(fp, text='perfil').pack(side='left', padx=(0, 4))
        ttk.Combobox(fp, textvariable=self.var_perfil, values=R.PERFILES, width=10,
                     state='readonly').pack(side='left')
        ttk.Label(fp, text='  (ir, círculo y rutas)', style='Suave.TLabel').pack(side='left')

        p = ttk.Labelframe(col, text=' Ir a un punto (mm) ', padding=6)
        p.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        self._campo(p, 0, 0, 'x', 'X')
        self._campo(p, 0, 2, 'y', 'Y')
        self._campo(p, 0, 4, 'z', 'Z')
        self._campo(p, 1, 0, 'v', 'v', 5)
        ttk.Label(p, text='mm/s', style='Suave.TLabel').grid(row=1, column=2, sticky='w')
        f = ttk.Frame(p)
        f.grid(row=2, column=0, columnspan=6, sticky='ew', pady=(6, 0))
        self._boton(f, 'Ir en recta', self._ir, 'Ir.TButton', row=0, column=0, padx=(0, 4))
        self._boton(f, 'Salto (escalón)', self._salto, row=0, column=1)
        ttk.Label(p, text='Ir: sigue una recta a v (seguimiento).\n'
                          'Salto: la consigna salta de golpe y se mide τ.',
                  style='Suave.TLabel', justify='left').grid(
            row=3, column=0, columnspan=6, sticky='w', pady=(4, 0))

        c = ttk.Labelframe(col, text=' Círculo (mm) ', padding=6)
        c.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        self._campo(c, 0, 0, 'cx', 'CX')
        self._campo(c, 0, 2, 'cy', 'CY')
        self._campo(c, 0, 4, 'r', 'R')
        self._campo(c, 1, 0, 'cz', 'z')
        self._campo(c, 1, 2, 'cv', 'v')
        self._campo(c, 1, 4, 'vueltas', 'vueltas', 4)
        self._boton(c, 'Círculo', self._circulo, 'Ir.TButton',
                    row=2, column=0, columnspan=2, sticky='w', pady=(6, 0))
        ttk.Label(c, text='v en mm/s', style='Suave.TLabel').grid(
            row=2, column=2, columnspan=4, sticky='w', pady=(6, 0))
        return col

    def _construir_regulacion(self, padre):
        marco = ttk.Labelframe(padre, text=' Regulación: empujón y ganancia ', padding=6)
        self._campo(marco, 0, 0, 'dx', 'DX', 5)
        self._campo(marco, 0, 2, 'dy', 'DY', 5)
        self._campo(marco, 0, 4, 'dz', 'DZ', 5)
        ttk.Label(marco, text='mm').grid(row=0, column=6, sticky='w')
        self._boton(marco, 'Empujar', self._empujon, row=0, column=7, padx=(8, 0))
        self._campo(marco, 1, 0, 'k', 'K', 5)
        ttk.Label(marco, text='1/s').grid(row=1, column=2, sticky='w')
        self._boton(marco, 'Aplicar', self._k, row=1, column=7, padx=(8, 0), pady=(4, 0))
        ttk.Label(marco, text='Empujón: 0.2 s sin control; luego mide cómo vuelve.',
                  style='Suave.TLabel').grid(row=2, column=0, columnspan=8, sticky='w',
                                             pady=(4, 0))
        self.lbl_tau = ttk.Label(marco, text='', style='Suave.TLabel')
        self.lbl_tau.grid(row=3, column=0, columnspan=8, sticky='w')
        return marco

    def _construir_ahora(self, padre):
        marco = ttk.Labelframe(padre, text=' Ahora ', padding=8)
        marco.columnconfigure(1, weight=1)
        self.lbl_estado = ttk.Label(marco, text='', style='Estado.TLabel', wraplength=420)
        self.lbl_estado.grid(row=0, column=0, columnspan=2, sticky='w')
        self.barra = ttk.Progressbar(marco, style='Ctrl.Horizontal.TProgressbar',
                                     maximum=100, mode='determinate')
        self.barra.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(4, 6))
        self.datos = {}
        for i, (clave, texto) in enumerate((('punta', 'punta'), ('consigna', 'consigna'),
                                            ('error', 'error'), ('sat', 'saturación'),
                                            ('k', 'K en marcha'))):
            ttk.Label(marco, text=texto, style='Suave.TLabel').grid(
                row=2 + i, column=0, sticky='w', padx=(0, 10))
            self.datos[clave] = ttk.Label(marco, text='--', style='Dato.TLabel')
            self.datos[clave].grid(row=2 + i, column=1, sticky='w')
        return marco

    def _construir_mensajes(self, padre):
        marco = ttk.Labelframe(padre, text=' Mensajes de control, y órdenes a mano ',
                               padding=8)
        marco.columnconfigure(0, weight=1)
        marco.rowconfigure(0, weight=1)
        self.registro = tk.Text(marco, height=5, width=46, font=('TkFixedFont', 9), bg=PAPEL,
                                fg=TINTA, relief='flat', highlightthickness=1,
                                highlightbackground=BORDE, wrap='word', state='disabled')
        self.registro.grid(row=0, column=0, columnspan=2, sticky='nsew')
        for tag, color in (('error', ROJO), ('aviso', NARANJA), ('tu', AZUL),
                           ('gz', VERDE), ('hora', SUAVE)):
            self.registro.tag_configure(tag, foreground=color)
        self.registro.tag_configure('dato', foreground=VERDE,
                                    font=('TkFixedFont', 9, 'bold'))
        e = ttk.Entry(marco, textvariable=self.var_mano)
        e.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        e.bind('<Return>', self._mandar_mano)
        e.bind('<KP_Enter>', self._mandar_mano)
        self._boton(marco, 'Enviar', self._mandar_mano, row=1, column=1,
                    pady=(6, 0), padx=(6, 0))
        ttk.Label(marco, text='p. ej.  ir 300 -50 130 v=20   ·   k=5,5,2   ·   '
                              'circulo 330 0 30 vueltas=2',
                  style='Suave.TLabel').grid(row=2, column=0, columnspan=2, sticky='w')
        return marco

    # ----------------------------------------------------------- la planta
    def _px(self, x: float, y: float) -> tuple[float, float]:
        cx, cy = (VISTA_X[0] + VISTA_X[1]) / 2.0, (VISTA_Y[0] + VISTA_Y[1]) / 2.0
        return (self.lado / 2.0 + (x - cx) * self.esc,
                self.lado / 2.0 - (y - cy) * self.esc)

    def _mm(self, u: float, v: float) -> tuple[float, float]:
        cx, cy = (VISTA_X[0] + VISTA_X[1]) / 2.0, (VISTA_Y[0] + VISTA_Y[1]) / 2.0
        return (cx + (u - self.lado / 2.0) / self.esc,
                cy - (v - self.lado / 2.0) / self.esc)

    def _pz(self, z: float) -> float:
        a, b = 30.0, self.lado - 30.0
        lo, hi = self.z_lim
        return b - (z - lo) / max(hi - lo, 1e-9) * (b - a)

    def _fondo(self):
        """El mapa de alcance, la rejilla y la regla de z. Se pinta una vez."""
        c = self.lienzo
        n = self.lado // CELDA
        jj, ii = np.meshgrid(np.arange(n), np.arange(n))
        X, Y = self._mm((jj + 0.5) * CELDA, (ii + 0.5) * CELDA)
        a = C.alcance_plano(X, Y, self.robot)
        tonos = np.where(a['down'], TONO_DOWN, np.where(a['up'], TONO_OTRA, TONO_FUERA))
        img = tk.PhotoImage(width=n, height=n)
        img.put(' '.join('{' + ' '.join(f) + '}' for f in tonos))
        self.mapa = img.zoom(CELDA)
        c.create_image(0, 0, image=self.mapa, anchor='nw', tags='fondo')

        x = math.ceil(VISTA_X[0] / 50.0) * 50.0
        while x <= VISTA_X[1]:
            (u0, w0), (u1, w1) = self._px(x, VISTA_Y[0]), self._px(x, VISTA_Y[1])
            c.create_line(u0, w0, u1, w1, fill=REJILLA, tags='fondo')
            if x % 100 == 0:
                c.create_text(u0, w0 - 8, text=f'{x:.0f}', fill=SUAVE,
                              font=('TkDefaultFont', 7), tags='fondo')
            x += 50.0
        y = math.ceil(VISTA_Y[0] / 50.0) * 50.0
        while y <= VISTA_Y[1]:
            (u0, w0), (u1, w1) = self._px(VISTA_X[0], y), self._px(VISTA_X[1], y)
            c.create_line(u0, w0, u1, w1, fill=BORDE if y == 0 else REJILLA, tags='fondo')
            if y % 100 == 0:
                c.create_text(u0 + 14, w0 - 7, text=f'{y:+.0f}', fill=SUAVE,
                              font=('TkDefaultFont', 7), tags='fondo')
            y += 50.0
        u, w = self._px(VISTA_X[1], 0.0)
        c.create_text(u - 8, w - 9, text='x', fill=SUAVE, tags='fondo')
        u, w = self._px(VISTA_X[0], VISTA_Y[1])
        c.create_text(u + 6, w + 8, text='y ↑   rejilla cada 50 mm', anchor='w', fill=SUAVE,
                      font=('TkDefaultFont', 8), tags='fondo')
        u, w = self._px(*R.HOME[:2])
        c.create_line(u - 5, w, u + 5, w, fill=SUAVE, tags='fondo')
        c.create_line(u, w - 5, u, w + 5, fill=SUAVE, tags='fondo')
        c.create_text(u + 6, w + 9, text='home', fill=SUAVE, anchor='w',
                      font=('TkDefaultFont', 7), tags='fondo')
        self.l_deseado = c.create_line(0, 0, 0, 0, fill=VERDE_CLARO, width=6,
                                       capstyle='round', joinstyle='round', state='hidden')
        self.l_traza = c.create_line(0, 0, 0, 0, fill=NARANJA, width=2, state='hidden')

        r = self.regla
        lo, hi = self.z_lim
        r.create_rectangle(28, self._pz(hi), 36, self._pz(lo), fill=TONO_DOWN,
                           outline=BORDE, tags='fondo')
        for z in (lo, R.Z_TRABAJO, R.Z_SEGURO, hi):
            v = self._pz(z)
            r.create_line(24, v, 40, v, fill=BORDE, tags='fondo')
            r.create_text(ANCHO_Z - 2, v, anchor='e', text=f'{z:.0f}', fill=SUAVE,
                          font=('TkDefaultFont', 7), tags='fondo')
        r.create_text(ANCHO_Z / 2, 10, text='z (mm)', fill=SUAVE,
                      font=('TkDefaultFont', 8), tags='fondo')

    def _encima(self, ev):
        x, y = self._mm(ev.x, ev.y)
        a = C.alcance_plano(np.array([x]), np.array([y]), self.robot)
        llega = ('llega (codo down)' if a['down'][0] else
                 'solo con el codo up' if a['up'][0] else 'no llega')
        self.lbl_encima.configure(text=f'x {x:6.1f}  y {y:+6.1f} mm   {llega}',
                                  style='Dato.TLabel')

    def _clic(self, ev, kx, ky):
        x, y = self._mm(ev.x, ev.y)
        self.v[kx].set(f'{round(x):.0f}')
        self.v[ky].set(f'{round(y):.0f}')

    def _pintar(self):
        n, c = self.n, self.lienzo
        c.delete('dyn')
        self.regla.delete('dyn')

        if n.deseado_n != self.deseado_visto:
            self.deseado_visto = n.deseado_n
            pts = n.deseado[::max(1, len(n.deseado) // 600)] + n.deseado[-1:]
            if len(pts) >= 2:
                c.coords(self.l_deseado, *[v for p in pts for v in self._px(p[0], p[1])])
                c.itemconfigure(self.l_deseado, state='normal')
            else:
                c.itemconfigure(self.l_deseado, state='hidden')
        if n.deseado:
            self._marca(n.deseado[-1], VERDE, 7)
            z = [p[2] for p in n.deseado]
            v0, v1 = self._pz(max(z)), self._pz(min(z))
            self.regla.create_rectangle(40, v0 - 1, 46, v1 + 1, fill=VERDE_CLARO,
                                        outline='', tags='dyn')

        tr = n.traza[-3000:]
        if len(tr) >= 2:
            c.coords(self.l_traza, *[v for p in tr for v in self._px(p[0], p[1])])
            c.itemconfigure(self.l_traza, state='normal')
        else:
            c.itemconfigure(self.l_traza, state='hidden')
        c.tag_raise(self.l_deseado, 'fondo')
        c.tag_raise(self.l_traza, self.l_deseado)

        # Lo que se mandaria con los campos de ahora, en azul discontinuo.
        try:
            x, y = float(self.v['x'].get()), float(self.v['y'].get())
            u, w = self._px(x, y)
            c.create_line(u - 6, w - 6, u + 6, w + 6, fill=AZUL, width=2, tags='dyn')
            c.create_line(u - 6, w + 6, u + 6, w - 6, fill=AZUL, width=2, tags='dyn')
            z = float(self.v['z'].get())
            self.regla.create_line(16, self._pz(z), 28, self._pz(z), fill=AZUL, width=2,
                                   tags='dyn')
        except ValueError:
            pass
        try:
            cx, cy, r = (float(self.v[k].get()) for k in ('cx', 'cy', 'r'))
            (u0, w0), (u1, w1) = self._px(cx - r, cy + r), self._px(cx + r, cy - r)
            c.create_oval(u0, w0, u1, w1, outline=AZUL, dash=(4, 3), tags='dyn')
            u, w = self._px(cx, cy)
            c.create_line(u - 3, w, u + 3, w, fill=AZUL, tags='dyn')
            c.create_line(u, w - 3, u, w + 3, fill=AZUL, tags='dyn')
        except ValueError:
            pass

        if n.punta is not None:
            p = n.punta
            u, w = self._px(p[0], p[1])
            c.create_oval(u - 5, w - 5, u + 5, w + 5, fill=NARANJA, outline='white',
                          tags='dyn')
            v = self._pz(p[2])
            self.regla.create_polygon(28, v, 18, v - 5, 18, v + 5, fill=NARANJA, tags='dyn')
            if n.error is not None:
                pd = [a + b for a, b in zip(p, n.error)]
                u, w = self._px(pd[0], pd[1])
                c.create_oval(u - 8, w - 8, u + 8, w + 8, outline=VERDE, width=2, tags='dyn')
                v = self._pz(pd[2])
                self.regla.create_line(36, v, 50, v, fill=VERDE, width=3, tags='dyn')

    def _marca(self, p, color, r):
        u, w = self._px(p[0], p[1])
        self.lienzo.create_oval(u - r, w - r, u + r, w + r, outline=color, width=2, tags='dyn')

    def _grafica(self):
        """|e| de los ultimos VENTANA_S s, en escala logaritmica: e0*exp(-K t) sale
        recta, y se ven a la vez un salto de 30 mm y un seguimiento de 0.01 mm.
        Las franjas rojas, donde la ley saturo."""
        g = self.graf
        g.delete('all')
        W, H = int(g['width']), ALTO_ERROR
        iz, de, ar, ab = 46, W - 8, 8, H - 16
        ahora = time.monotonic()
        datos = [d for d in list(self.n.serie) if ahora - d[0] <= VENTANA_S]
        d0 = round(math.log10(E_MIN))
        d1 = max(d0 + 2, math.ceil(math.log10(max([d[1] for d in datos] + [0.01]))))

        def px(t, e):
            f = (math.log10(max(e, E_MIN)) - d0) / (d1 - d0)
            return iz + (de - iz) * (1.0 - (ahora - t) / VENTANA_S), ab - (ab - ar) * f

        g.create_rectangle(iz, ar, de, ab, outline=BORDE)
        for d in range(d0, d1 + 1):
            y = ab - (ab - ar) * (d - d0) / (d1 - d0)
            if d0 < d < d1:
                g.create_line(iz, y, de, y, fill=REJILLA)
            g.create_text(iz - 4, y, text=f'{10.0 ** d:g}', anchor='e', fill=SUAVE,
                          font=('TkDefaultFont', 7))
        g.create_text(4, ar - 2, text='mm', anchor='nw', fill=SUAVE, font=('TkDefaultFont', 7))
        for s in range(0, int(VENTANA_S) + 1, 5):
            x = de - (de - iz) * s / VENTANA_S
            g.create_text(x, ab + 8, text='ahora' if s == 0 else f'-{s} s', fill=SUAVE,
                          anchor='e' if s == 0 else 'center', font=('TkDefaultFont', 7))
        # Franjas de saturacion
        i = 0
        while i < len(datos):
            if datos[i][2] > 1.0:
                j = i
                while j + 1 < len(datos) and datos[j + 1][2] > 1.0:
                    j += 1
                x0, x1 = px(datos[i][0], 1.0)[0], px(datos[j][0], 1.0)[0]
                g.create_rectangle(x0, ar + 1, max(x1, x0 + 1), ab - 1, fill='#f6d5d2',
                                   outline='')
                i = j + 1
            else:
                i += 1
        paso = max(1, len(datos) // 500)
        xy = [v for d in datos[::paso] + datos[-1:] for v in px(d[0], d[1])]
        if len(xy) >= 4:
            g.create_line(*xy, fill=AZUL, width=2)
        if datos:
            g.create_text(de - 4, ar + 2, anchor='ne', fill=AZUL,
                          font=('TkFixedFont', 9, 'bold'),
                          text=f'{datos[-1][1]:.3f} mm')
        else:
            g.create_text((iz + de) / 2, (ar + ab) / 2, fill=SUAVE,
                          text='sin datos de control_nodo')

    # --------------------------------------------------------------- ordenes
    def _num(self, clave: str, positivo: bool = False) -> float:
        e = self.entradas[clave]
        try:
            v = float(self.v[clave].get().replace(',', '.'))
            if not math.isfinite(v) or (positivo and v <= 0.0):
                raise ValueError
        except ValueError:
            e.configure(style='Mal.TEntry')
            raise
        e.configure(style='TEntry')
        return v

    def _mandar(self, *claves, positivas=(), hacer):
        """Lee los campos; si alguno no es un numero lo marca y no manda nada."""
        try:
            vals = [self._num(k, k in positivas) for k in claves]
        except ValueError:
            return
        self.n.mandar(hacer(*vals))

    def _tarea_ruta(self, orden: str):
        self.n.mandar(orden if orden == 'home' else f'{orden} perfil={self.var_perfil.get()}')

    def _ir(self):
        self._mandar('x', 'y', 'z', 'v', positivas=('v',), hacer=lambda x, y, z, v:
                     f'ir {x:g} {y:g} {z:g} v={v:g} perfil={self.var_perfil.get()}')

    def _salto(self):
        self._mandar('x', 'y', 'z', hacer=lambda x, y, z: f'salto {x:g} {y:g} {z:g}')

    def _circulo(self):
        self._mandar('cx', 'cy', 'r', 'cz', 'cv', 'vueltas', positivas=('r', 'cv', 'vueltas'),
                     hacer=lambda cx, cy, r, z, v, n:
                     f'circulo {cx:g} {cy:g} {r:g} z={z:g} v={v:g} vueltas={n:g} '
                     f'perfil={self.var_perfil.get()}')

    def _empujon(self):
        self._mandar('dx', 'dy', 'dz', hacer=lambda dx, dy, dz: f'empujon {dx:g} {dy:g} {dz:g}')

    def _k(self):
        self._mandar('k', positivas=('k',), hacer=lambda k: f'k={k:g}')

    def _tau(self):
        try:
            k = float(self.v['k'].get().replace(',', '.'))
            if k <= 0.0:
                raise ValueError
            aviso = ('' if 5.0 <= k <= 20.0 else
                     '   (lento)' if k < 5.0 else '   (sobrepasa: mejor ≤ 20)')
            self.lbl_tau.configure(text=f'τ = 1/K = {1.0 / k:.3f} s · recomendada 5–20{aviso}')
        except ValueError:
            self.lbl_tau.configure(text='K tiene que ser un número > 0')

    def _mandar_mano(self, ev=None):
        texto = self.var_mano.get().strip()
        if texto and self.n.mandar(texto):
            self.var_mano.set('')
        return 'break'

    def _alternar_gz(self):
        self.n.pausar(not self.n.pausa)

    def cerrar(self):
        self.root.after_cancel(self._tarea)
        self.root.destroy()

    # --------------------------------------------------------------- refresco
    def _refrescar(self):
        # Ctrl+C en el launch apaga rclpy, pero eso no saca al mainloop de Tk.
        if not rclpy.ok():
            self.root.destroy()
            return
        self._tarea = self.root.after(100, self._refrescar)
        n = self.n
        while True:
            try:
                nivel, texto = n.log.get_nowait()
            except queue.Empty:
                break
            if nivel in ('tu', 'gz'):
                self._al_registro(('tú: ' if nivel == 'tu' else '') + texto, nivel)
            elif nivel >= 40:
                self._al_registro(texto, 'error')
            elif nivel >= 30:
                self._al_registro(texto, 'aviso')
            else:
                dato = ('terminada' in texto or 'tau medido' in texto
                        or texto.startswith('   '))
                self._al_registro(texto, 'dato' if dato else None)

        vivo, escucha = n.vivo(), n.escucha()
        estado = n.estado if vivo else ''
        if n.pausa:
            texto, color = 'Gazebo en pausa', NARANJA
        elif not escucha:
            texto, color = 'control: no está', ROJO
        elif not vivo or estado.startswith('sin /joint_states'):
            texto, color = 'control: sin medidas de Gazebo', NARANJA
        elif estado.startswith('regulando'):
            texto, color = 'control: regulando', VERDE
        else:
            texto, color = 'control: moviendo', AZUL
        self.lbl_ctrl.configure(text=texto, foreground=color)
        self.b_gz.configure(text='Seguir Gazebo' if n.pausa else 'Pausar Gazebo')
        for b in self.botones:
            b.state(['!disabled'] if escucha else ['disabled'])

        self.lbl_estado.configure(text=_estado_corto(estado) if estado else
                                  'esperando a control_nodo y a Gazebo…')
        m = K_T.search(estado)
        if m:
            k = [float(v) for v in m.group(1).split(',')]
            self.datos['k'].configure(text=f'{m.group(1)} 1/s   (τ = {1.0 / k[0]:.3f} s)')
        m = ORDEN_T.search(estado)
        self.barra['value'] = int(m.group(1)) if m else 0
        if n.punta is not None:
            p = n.punta
            self.datos['punta'].configure(text=f'x {p[0]:7.2f}  y {p[1]:+7.2f}  z {p[2]:6.2f} mm')
            if n.error is not None:
                e = n.error
                pd = [a + b for a, b in zip(p, e)]
                self.datos['consigna'].configure(
                    text=f'x {pd[0]:7.2f}  y {pd[1]:+7.2f}  z {pd[2]:6.2f} mm')
                self.datos['error'].configure(
                    text=f'{math.sqrt(sum(v * v for v in e)):.3f} mm   '
                         f'(máx. desde la última orden {n.err_max:.3f})')
            self.datos['sat'].configure(
                text=(f'recorta: pidió {n.sat:.1f} veces el techo' if n.sat > 1.0 else
                      f'no ({n.sat * 100:.0f} % del techo)'),
                foreground=ROJO if n.sat > 1.0 else TINTA)
        self._pintar()
        self._grafica()

    def _al_registro(self, texto: str, tag: str | None = None):
        r = self.registro
        r.configure(state='normal')
        r.insert('end', time.strftime('%H:%M:%S  '), 'hora')
        r.insert('end', texto.rstrip() + '\n', tag or ())
        if int(r.index('end-1c').split('.')[0]) > 400:
            r.delete('1.0', '100.0')
        r.see('end')
        r.configure(state='disabled')


def _estado_corto(estado: str) -> str:
    """'circulo aproximacion  40% (2.1/17.3 s) | error ...' -> 'circulo · aproximacion
    40% (2.1/17.3 s)'. El error y K ya salen en sus filas."""
    cabeza = estado.split(' | ')[0]
    p = cabeza.split(None, 2)
    if len(p) == 3 and ORDEN_T.search(p[2]):
        return f'{p[0]} · {p[1]}   {p[2]}'
    return cabeza


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
    node = ControlGuiNode()
    hilo = threading.Thread(target=_girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaControl(node)
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
