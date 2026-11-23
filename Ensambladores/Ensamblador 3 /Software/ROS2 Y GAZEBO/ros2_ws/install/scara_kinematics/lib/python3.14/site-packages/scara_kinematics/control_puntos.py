"""Nodo `control_puntos`: el ejercicio del control cinematico, en tres fases, con
el SCARA de verdad o con Gazebo.

    ros2 launch scara_kinematics control_placa.launch.py              # el brazo
    ros2 launch scara_kinematics control.launch.py tarea:=nada        # o Gazebo
    # y, en OTRA terminal (la fase 2 pide los puntos por el teclado):
    ros2 run scara_kinematics control_puntos --ros-args \\
        --params-file install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml

Fase 1  una trayectoria por varios puntos (`puntos`, en cm): una recta a cada uno.
Fase 2  te pide puntos por la terminal, [X, Y, Z] en cm, hasta que escribas 'salir'.
Fase 3  las cuatro graficas de las dos fases: la punta pedida y la medida, el
        error, la posicion de las juntas y su velocidad con sus techos.

El control no lo calcula este nodo: lo hace `control_nodo`, que ya esta en
marcha con el launch (qd = J(q)^-1 (xd_dot + K e), con la punta MEDIDA). Este
nodo le manda `ir X Y Z` por scara/control/orden, sabe por /rosout cuando
empieza, acaba o se rechaza cada orden, y apunta lo que publica:

    scara/control/datos/punta_ref    Vector3   la punta pedida (mm)
    scara/control/datos/punta_real   Vector3   la medida (mm)
    scara/control/datos/error        Vector3   pedida - medida (mm)
    scara/control/datos/qd           Vector3   lo que manda a las juntas (°/s, °/s, mm/s)
    /joint_states                              donde estan las juntas (rad, rad, m)

Los techos de velocidad y los topes los lee de los parametros del propio
`control_nodo` (se llama `control`): son los del launch que este en marcha.

Parametros:
    puntos     [x1, y1, z1, x2, ...] en cm   la trayectoria de la fase 1
    v          mm/s; 0 = la de control_nodo (v_ir: 40 en gz, 20 con el brazo)
    graficas   el PNG de la fase 3 (y un CSV al lado con lo apuntado)
    ventana    false = no abre la ventana de las graficas, solo las guarda

Ctrl+C en la fase 1 o en la 2 manda `parar` y pasa a las graficas.
"""
from __future__ import annotations

import csv
import math
import os
import re
import signal
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from scara_kinematics import ruta as R
from scara_kinematics.model import DEFAULT_PARAMS, ScaraModel, declare_model_params

# Dentro de la zona que alcanza el brazo con sus recorridos (config/
# micropython.yaml, los de Sergio del 21/10), en la rama down, la del brazo en
# 0, y las rectas entre ellos tambien caben. El primero, cerca del home de
# Sergio (0, -25 cm); de uno a otro ninguna junta gira mas de 20 grados (lo
# pidio Sergio). Vuelve al primero para cerrar la vuelta.
PUNTOS_CM = [2.0, -28.0, 20.0, 8.0, -32.0, 19.0, 14.0, -30.0, 18.0,
             13.0, -36.0, 19.0, 5.0, -38.0, 18.0, 1.0, -33.0, 19.0,
             2.0, -28.0, 20.0]
NODO_CONTROL = 'control'               # el nombre con el que lanzan a control_nodo
NODO_PLACA = 'puente_placa'            # con el brazo de verdad (control_placa.launch.py)
YA_ESTA_MM = 0.5                       # mas cerca que esto: no se manda el ir
SALIR = ('salir', 'q', 'exit', 'fin')

AQUA = '#1baf7a'                       # el tercer color de la paleta de ruta.py
SERIES = (R.AZUL, R.NARANJA, AQUA)
# Las leyendas, en la linea del titulo, a la derecha: dentro tapan las curvas
ARRIBA = dict(loc='lower right', bbox_to_anchor=(1.0, 1.0), borderaxespad=0.15,
              handlelength=2.2, columnspacing=1.2)


class PuntoMalo(ValueError):
    pass


def leer_punto(texto: str) -> np.ndarray | None:
    """'[40, -10, 12]', '40 -10 12' o '40,-10,12' (cm) -> mm. None = salir."""
    t = texto.strip().lower()
    if t in SALIR:
        return None
    partes = [p for p in re.split(r'[\s,;\[\]()]+', t) if p]
    try:
        v = [float(p) for p in partes]
    except ValueError:
        raise PuntoMalo(f'«{texto.strip()}» no son numeros: escribe X, Y, Z en cm') from None
    if len(v) != 3:
        raise PuntoMalo(f'hacen falta tres numeros (X, Y, Z en cm) y hay {len(v)}')
    return np.array(v) * 10.0


def orden_ir(p_mm, v: float = 0.0) -> str:
    """La orden de control_nodo: una recta hasta p (mm)."""
    texto = 'ir ' + ' '.join(f'{c:.1f}' for c in p_mm)
    return texto + (f' v={v:g}' if v > 0.0 else '')


def motivo_no_llega(p_mm, robot: R.Robot) -> str | None:
    """Por que no llega la punta a p (las dos ramas del codo), o None si llega."""
    motivos = []
    for codo in ('down', 'up'):
        try:
            R.ik_scara_rrp(p_mm, robot, codo)
            return None
        except R.RutaError as e:
            motivos.append(str(e))
    return motivos[0] if motivos[0] == motivos[1] else '; '.join(motivos)


class Registro:
    """Lo que publica control_nodo, con la hora (s) a la que llego aqui."""

    CAMPOS = ('punta_ref', 'punta_real', 'error', 'qd')

    def __init__(self):
        self.cerrojo = threading.Lock()
        self.t0 = time.monotonic()
        self.datos = {k: ([], []) for k in self.CAMPOS + ('q',)}
        self.marcas: list[tuple[float, str]] = []         # (t, 'fase 2'), (t, 'P3')...
        self.apuntando = True

    def ahora(self) -> float:
        return time.monotonic() - self.t0

    def poner(self, campo: str, valores, t: float | None = None):
        if not self.apuntando:
            return
        with self.cerrojo:
            ts, vs = self.datos[campo]
            ts.append(self.ahora() if t is None else t)
            vs.append([float(v) for v in valores])

    def marcar(self, texto: str):
        with self.cerrojo:
            self.marcas.append((self.ahora(), texto))

    def serie(self, campo: str):
        """(t, valores N x 3) del campo, en arrays."""
        with self.cerrojo:
            ts, vs = self.datos[campo]
            return np.array(ts), np.array(vs).reshape(-1, 3)

    def guardar_csv(self, ruta: str):
        """Una fila por dato que llego: t, campo y sus tres valores."""
        with self.cerrojo, open(ruta, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'campo', 'a', 'b', 'c'])
            for campo, (ts, vs) in self.datos.items():
                for t, v in zip(ts, vs):
                    w.writerow([f'{t:.3f}', campo] + [f'{x:.4f}' for x in v])
            for t, texto in self.marcas:
                w.writerow([f'{t:.3f}', 'marca', texto, '', ''])


def graficas(reg: Registro, techos=None, archivo: str | None = None, ventana: bool = True):
    """Las cuatro graficas de la fase 3. techos: [rad/s, rad/s, mm/s] o None."""
    import matplotlib
    if not ventana:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    plt.rcParams.update({
        'font.size': 8.5, 'axes.edgecolor': R.GRIS, 'axes.labelcolor': R.TINTA2,
        'xtick.color': R.TINTA2, 'ytick.color': R.TINTA2, 'text.color': R.TINTA,
        'axes.grid': True, 'grid.color': R.REJILLA, 'grid.linewidth': 0.7,
        'axes.titlesize': 9.5, 'axes.titlelocation': 'left', 'legend.fontsize': 7.5,
        'legend.frameon': False, 'lines.solid_capstyle': 'round',
        'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb'})
    # Cabe en 1366x768. A la derecha, juntas en dos franjas: grados y mm no
    # comparten eje.
    fig = plt.figure(figsize=(13.0, 6.6))
    g = fig.add_gridspec(4, 2, hspace=0.55, wspace=0.16, left=0.055, right=0.985,
                         top=0.93, bottom=0.08)
    ax_p = fig.add_subplot(g[0:2, 0])
    ax_e = fig.add_subplot(g[2:4, 0], sharex=ax_p)
    ax_qa = fig.add_subplot(g[0, 1], sharex=ax_p)
    ax_qd = fig.add_subplot(g[1, 1], sharex=ax_p)
    ax_va = fig.add_subplot(g[2, 1], sharex=ax_p)
    ax_vd = fig.add_subplot(g[3, 1], sharex=ax_p)
    fig.suptitle('Control cinemático: fase 1 (trayectoria por puntos) y fase 2 (puntos '
                 'tecleados)', x=0.055, ha='left', fontsize=10.5)

    t_ref, ref = reg.serie('punta_ref')
    t_real, real = reg.serie('punta_real')
    for i, eje in enumerate('xyz'):
        ax_p.plot(t_ref, ref[:, i] / 10.0, color=SERIES[i], lw=1.2, ls=(0, (4, 2)),
                  label=f'{eje} pedida')
        ax_p.plot(t_real, real[:, i] / 10.0, color=SERIES[i], lw=2.0, alpha=0.85,
                  label=f'{eje} medida')
    ax_p.set_title('1. Seguimiento de la punta')
    ax_p.set_ylabel('cm')
    # El color dice el eje; el trazo, si es la pedida o la medida
    ax_p.legend(handles=[Line2D([], [], color=c, lw=2.0, label=e) for c, e in zip(SERIES, 'xyz')]
                + [Line2D([], [], color=R.TINTA2, lw=1.2, ls=(0, (4, 2)), label='pedida'),
                   Line2D([], [], color=R.TINTA2, lw=2.0, label='medida')], **ARRIBA, ncol=5)

    t_e, err = reg.serie('error')
    if len(err):                         # la norma detras, para que no tape a las otras
        ax_e.plot(t_e, np.linalg.norm(err, axis=1) / 10.0, color=R.GRIS, lw=0.8,
                  label='|e|')
    for i, eje in enumerate('xyz'):
        ax_e.plot(t_e, err[:, i] / 10.0, color=SERIES[i], lw=1.2, label=f'e{eje}')
    ax_e.set_title('2. Error de la punta (pedida − medida)')
    ax_e.set_ylabel('cm')
    ax_e.set_xlabel('tiempo (s)')
    ax_e.legend(**ARRIBA, ncol=4)

    t_q, q = reg.serie('q')
    t_v, qd = reg.serie('qd')
    if len(q):
        ax_qa.plot(t_q, np.degrees(q[:, 0]), color=SERIES[0], lw=1.6, label='θ1')
        ax_qa.plot(t_q, np.degrees(q[:, 1]), color=SERIES[1], lw=1.6, label='θ2')
        ax_qd.plot(t_q, q[:, 2] * 1e3, color=SERIES[2], lw=1.6, label='d3')
    ax_qa.set_title('3. Posición de las juntas')
    ax_qa.set_ylabel('°')
    ax_qd.set_ylabel('mm')
    if len(qd):
        ax_va.plot(t_v, qd[:, 0], color=SERIES[0], lw=1.2, label='θ1')
        ax_va.plot(t_v, qd[:, 1], color=SERIES[1], lw=1.2, label='θ2')
        ax_vd.plot(t_v, qd[:, 2], color=SERIES[2], lw=1.2, label='d3')
    if techos is not None:
        for ax, tope, txt in ((ax_va, math.degrees(techos[0]), 'techo θ1'),
                              (ax_vd, techos[2], 'techo d3')):
            for s in (1, -1):
                ax.axhline(s * tope, color=R.GRIS, lw=1.0, ls=(0, (2, 2)),
                           label=(txt if not math.isclose(techos[0], techos[1]) or ax is ax_vd
                                  else 'techo') if s == 1 else None)
        if not math.isclose(techos[0], techos[1]):
            for s in (1, -1):
                ax_va.axhline(s * math.degrees(techos[1]), color=R.TINTA2, lw=1.0,
                              ls=(0, (1, 2)), label='techo θ2' if s == 1 else None)
    ax_va.set_title('4. Velocidad que manda el control a las juntas')
    ax_va.set_ylabel('°/s')
    ax_vd.set_ylabel('mm/s')
    ax_vd.set_xlabel('tiempo (s)')
    for ax in (ax_qa, ax_qd, ax_va, ax_vd):
        ax.legend(**ARRIBA, ncol=3)

    # Donde empieza la fase 2, sombreada, y cada punto con su nombre arriba
    t_fin = max([t[-1] for t in (t_ref, t_real, t_q, t_v) if len(t)] or [1.0])
    inicio_2 = [t for t, m in reg.marcas if m == 'fase 2']
    for ax in (ax_p, ax_e, ax_qa, ax_qd, ax_va, ax_vd):
        if inicio_2:
            ax.axvspan(inicio_2[0], t_fin, color=R.REJILLA, alpha=0.45, lw=0)
        for t, m in reg.marcas:
            if m.startswith('P') or m.startswith('T'):
                ax.axvline(t, color=R.GRIS, lw=0.6)
    for t, m in reg.marcas:
        if m.startswith('P') or m.startswith('T'):
            ax_p.annotate(m, (t, 1.0), xycoords=('data', 'axes fraction'),
                          xytext=(2, -10), textcoords='offset points',
                          fontsize=7, color=R.TINTA2)
    if inicio_2:
        for ax in (ax_p, ax_e):
            ax.annotate('fase 2', (inicio_2[0], 0.0), xycoords=('data', 'axes fraction'),
                        xytext=(3, 4), textcoords='offset points', fontsize=7.5,
                        color=R.TINTA2)
    for ax in (ax_p, ax_qa, ax_qd, ax_va):          # la hora, solo en las de abajo
        ax.tick_params(labelbottom=False)

    if archivo:
        fig.savefig(archivo, dpi=110)
    if ventana:
        plt.show()
    return fig


class ControlPuntos(Node):

    def __init__(self):
        super().__init__('control_puntos')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)[:3]
        self.robot = R.Robot.desde_modelo(self.model)      # hasta que conteste control
        self.puntos_cm = [float(v) for v in self.declare_parameter('puntos', PUNTOS_CM).value]
        self.v = float(self.declare_parameter('v', 0.0).value)
        self.archivo = os.path.expanduser(str(self.declare_parameter(
            'graficas', '~/control_puntos.png').value))
        self.ventana = bool(self.declare_parameter('ventana', True).value)
        if len(self.puntos_cm) % 3 or not self.puntos_cm:
            raise ValueError(f'puntos va de tres en tres (cm): {self.puntos_cm}')

        self.reg = Registro()
        self.techos = None                       # [rad/s, rad/s, mm/s], los de control
        self.cerrojo = threading.Lock()
        self.estado = ''
        self.suelta = None                       # por que solto la placa los motores
        self.ultimo_fallo = ''
        self.avisos: list[str] = []              # lo que dice control por /rosout
        self.orden_pub = self.create_publisher(String, 'scara/control/orden', 10)
        for campo in Registro.CAMPOS:
            self.create_subscription(
                Vector3, f'scara/control/datos/{campo}',
                lambda m, c=campo: self.reg.poner(c, (m.x, m.y, m.z)), 50)
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.create_subscription(String, 'scara/control/estado', self.on_estado, 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)

    # ------------------------------------------------------------------ entradas
    def on_js(self, msg: JointState):
        donde = {n: i for i, n in enumerate(msg.name)}
        if all(j in donde for j in self.juntas):
            self.reg.poner('q', [msg.position[donde[j]] for j in self.juntas])

    def on_estado(self, msg: String):
        with self.cerrojo:
            self.estado = msg.data

    def on_log(self, msg: Log):
        nodo = msg.name.split('.')[-1]
        if nodo == NODO_CONTROL:
            with self.cerrojo:
                self.avisos.append(msg.msg)
        elif nodo == NODO_PLACA:
            # Con el brazo de verdad: si un vigilante de la placa suelta los
            # motores, control_nodo se para y aceptaria la orden siguiente con
            # el brazo suelto. Se apunta para no mandar nada mas.
            if msg.level >= Log.ERROR or msg.msg.startswith('placa: VIGILANTE'):
                self.ultimo_fallo = msg.msg.split(': ', 1)[-1]
            if 'motores sueltos' in msg.msg:
                self.suelta = self.ultimo_fallo or 'los han soltado'
            elif 'ACTIVO' in msg.msg:
                self.suelta, self.ultimo_fallo = None, ''

    def _avisos(self) -> list[str]:
        with self.cerrojo:
            a, self.avisos = self.avisos, []
        return a

    # ------------------------------------------------------------- control_nodo
    def conectar(self, plazo: float = 30.0) -> bool:
        """Espera a que control_nodo diga algo y lee sus techos y su robot."""
        fin = time.monotonic() + plazo
        while time.monotonic() < fin and not self.estado:
            time.sleep(0.1)
        if not self.estado:
            return False
        cliente = AsyncParameterClient(self, NODO_CONTROL)
        if cliente.wait_for_services(timeout_sec=5.0):
            nombres = ['max_vel_hw'] + list(DEFAULT_PARAMS)
            fut = cliente.get_parameters(nombres)
            fin = time.monotonic() + 5.0
            while not fut.done() and time.monotonic() < fin:
                time.sleep(0.05)
            if fut.done() and fut.result() is not None:
                vals = {n: parameter_value_to_python(v)
                        for n, v in zip(nombres, fut.result().values)}
                techo = list(vals.pop('max_vel_hw'))[:3]
                self.techos = [techo[0], techo[1], techo[2] * 1e3]
                modelo = ScaraModel.from_params(vals)
                self.robot = R.Robot.desde_modelo(modelo)
                # los nombres con los que publica /joint_states el launch en marcha
                self.juntas = list(modelo.joint_names)[:3]
        return True

    def esperar_quieto(self, plazo: float = 60.0) -> bool:
        """Hasta que control_nodo no tenga tarea (regulando)."""
        fin = time.monotonic() + plazo
        while time.monotonic() < fin:
            if self.estado.startswith('regulando'):
                return True
            time.sleep(0.1)
        return False

    def ir(self, p_mm, nombre: str) -> bool:
        """Manda `ir`, espera a que acabe y dice como ha ido. False si no salio."""
        motivo = motivo_no_llega(p_mm, self.robot)
        if motivo:
            print(f'   {nombre}: NO LLEGA, no se manda. {motivo}')
            return False
        if self.suelta:
            print(f'   {nombre}: no se manda: la placa tiene los motores sueltos '
                  f'({self.suelta}). Activalos («Activar motores») y vuelve a pedirlo')
            return False
        t, real = self.reg.serie('punta_real')
        if len(real) and np.linalg.norm(real[-1] - p_mm) < YA_ESTA_MM:
            print(f'   {nombre}: ya esta ahi')
            return True
        self._avisos()
        orden = orden_ir(p_mm, self.v)
        self.reg.marcar(nombre)
        self.orden_pub.publish(String(data=orden))
        print(f'   {nombre}: {orden}', flush=True)
        # Empieza: «ir: 12.3 s (recta 12.3 s) | ...»; o no sale: «ir ...» no sale: ...
        # Lo que venga detras en el mismo lote se guarda: un ir de 0 s acaba a la vez
        dura, fin, resto = None, time.monotonic() + 5.0, []
        while dura is None and time.monotonic() < fin:
            avisos = self._avisos()
            for i, a in enumerate(avisos):
                if a.startswith('«ir') and 'no sale' in a:
                    print(f'   {nombre}: control_nodo NO lo hace: {a.split(": ", 1)[-1]}')
                    return False
                m = re.match(r'ir: ([\d.]+) s', a)
                if m:
                    dura, resto = float(m.group(1)), avisos[i + 1:]
                    break
            time.sleep(0.05)
        if dura is None:
            print(f'   {nombre}: control_nodo no contesta. ¿Esta en marcha el launch?')
            return False
        # Acaba: «ir terminada | ... medio X mm, maximo Y mm, ...»
        fin = time.monotonic() + 2.0 * dura + 15.0
        while time.monotonic() < fin:
            avisos, resto = resto + self._avisos(), []
            for a in avisos:
                if a.startswith('ir terminada'):
                    t, err = self.reg.serie('error')
                    final = np.linalg.norm(err[-1]) if len(err) else float('nan')
                    m = re.search(r'medio ([\d.]+) mm, maximo ([\d.]+) mm', a)
                    resumen = (f'error en el camino: medio {m.group(1)} mm, maximo '
                               f'{m.group(2)} mm' if m else a)
                    print(f'   {nombre}: llega en {dura:.1f} s | {resumen} | al final '
                          f'{final:.2f} mm')
                    return True
                if 'no sale' in a or 'parado' in a:
                    time.sleep(0.2)                  # el porque llega por puente_placa
                    print(f'   {nombre}: cortado: {a}'
                          + (f' | la placa ha soltado los motores: {self.suelta}'
                             if self.suelta else ''))
                    return False
            time.sleep(0.05)
        print(f'   {nombre}: no acaba en {2.0 * dura + 15.0:.0f} s; sigo')
        return False

    def parar(self):
        self.orden_pub.publish(String(data='parar'))

    # ------------------------------------------------------------------ fases
    def fase1(self):
        pts = np.array(self.puntos_cm).reshape(-1, 3)
        print(f'\n=== FASE 1: trayectoria por {len(pts)} puntos ===')
        for i, p in enumerate(pts, 1):
            print(f'   P{i} = [{p[0]:g}, {p[1]:g}, {p[2]:g}] cm')
        for i, p in enumerate(pts, 1):
            if not self.ir(p * 10.0, f'P{i}'):
                if self.suelta:
                    print('   se acaba la fase 1: con los motores sueltos no se mueve nada')
                    return
                print('   la trayectoria sigue con el punto siguiente')

    def fase2(self):
        print('\n=== FASE 2: puntos a mano ===')
        self.reg.marcar('fase 2')
        n = 0
        while rclpy.ok():
            try:
                texto = input("Ingrese nuevo punto [X, Y, Z] en cm (o 'salir'): ")
            except EOFError:
                break
            try:
                p = leer_punto(texto)
            except PuntoMalo as e:
                print(f'   {e}')
                continue
            if p is None:
                break
            n += 1
            self.ir(p, f'T{n}')


def main(args=None):
    # Sin los manejadores de senales de rclpy, como control_nodo: con ellos,
    # Ctrl+C apaga ROS por detras, el `parar` ya no sale y este hilo se queda
    # esperando. Asi llega como KeyboardInterrupt, con ROS aun vivo.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = ControlPuntos()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    def girar():
        try:
            ex.spin()
        except Exception:                    # el rclpy.shutdown del final
            if rclpy.ok():
                raise

    hilo = threading.Thread(target=girar, daemon=True)
    hilo.start()
    try:
        print('esperando a control_nodo (lanza control_placa.launch.py o control.launch.py)...',
              flush=True)
        if not node.conectar():
            print('control_nodo no da señales: no hay nada que mandar. Lanza antes el launch.')
            return
        techo = ('techos sin leer' if node.techos is None else
                 f'techos {node.techos[0]:g} rad/s, {node.techos[1]:g} rad/s, '
                 f'{node.techos[2]:g} mm/s')
        print(f'control_nodo en marcha | {techo} | v '
              f'{node.v:g} mm/s' if node.v > 0 else f'control_nodo en marcha | {techo}')
        if not node.esperar_quieto():
            print('control_nodo sigue con otra tarea; mando parar')
            node.parar()
        try:
            node.fase1()
            node.fase2()
        except KeyboardInterrupt:
            node.parar()
            print('\nCtrl+C: parar mandado. Paso a las graficas (otro Ctrl+C, para salir).')
        node.reg.apuntando = False
        print('\n=== FASE 3: graficas ===')
        base = os.path.splitext(node.archivo)[0]
        node.reg.guardar_csv(base + '.csv')
        print(f'   datos: {base}.csv')
        print(f'   graficas: {node.archivo}' + ('  (cierra la ventana para acabar)'
                                                if node.ventana else ''))
        graficas(node.reg, node.techos, node.archivo,
                 node.ventana and bool(os.environ.get('DISPLAY') or
                                       os.environ.get('WAYLAND_DISPLAY')))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # En este orden: con el hilo del executor aun girando, destruir el nodo
        # da un Segmentation fault (visto al cerrar, con Gazebo).
        if rclpy.ok():
            rclpy.shutdown()
        hilo.join(timeout=2.0)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
