"""Ventana `efector`: escribes (x, y, z) de la punta y te dice si llega y con que angulos.

    ros2 launch scara_kinematics efector.launch.py                 # con la placa
    ros2 launch scara_kinematics efector.launch.py simular:=true   # sin placa

Va con el nodo `vector` (el que habla con el main.py de la ESP32): esta ventana
calcula y dibuja, y el boton Ir le manda 'ir x y z' a `vector`, que lo vuelve a
comprobar antes de mandar nada a la placa.

    la planta     la zona sombreada es donde llega la punta con los limites de
                  los motores; gris, el brazo donde esta; azul, donde iria.
                  Un clic pone x e y (la z se queda).
    el veredicto  ALCANZABLE / NO ALCANZABLE, y por que motor no llega
    los angulos   en la junta (lo de la cinematica) y en el motor (lo de main.py)

Los limites, la reduccion, los signos, el cero y el paso del husillo salen de
config/micropython.yaml, el mismo archivo que lee `vector`.
"""
from __future__ import annotations

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from scara_kinematics import alcance as A
from scara_kinematics import ruta as R
from scara_kinematics.estilo import (AZUL, BORDE, FONDO, PAPEL, REJILLA, ROJO,
                                     SUAVE, TINTA, VERDE)
from scara_kinematics.model import declare_model_params

LADO = 440            # pixeles del lienzo de la planta
ZONA = '#d6ebdf'      # relleno de la zona alcanzable (verde muy claro)


class EfectorNode(Node):

    def __init__(self):
        super().__init__('efector')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        d = A.Calibracion()
        defectos = {'limites_motor': [x for par in d.limites for x in par],
                    'reduccion': d.reduccion, 'signo': d.signo, 'cero': d.cero,
                    'paso_husillo_mm': d.paso_husillo_mm, 'velocidades': d.velocidades}
        self.cal = A.Calibracion.desde_parametros(
            lambda nombre, _: self.declare_parameter(nombre, defectos[nombre]).value)
        self.orden_pub = self.create_publisher(String, 'scara/vector/orden', 10)
        self.pos_motor: list[float] | None = None
        self.estado = ''
        self.placa = ''
        self.create_subscription(Vector3, 'scara/vector/posicion', self.on_pos, 10)
        # El ultimo vector que `vector` ha mandado a la placa: contra el se mide el error
        self.enviado: list[float] | None = None
        self.create_subscription(Float64MultiArray, 'scara/vector/enviado',
                                 lambda msg: setattr(self, 'enviado', list(msg.data)), 10)
        self.create_subscription(String, 'scara/vector/estado',
                                 lambda m: setattr(self, 'estado', m.data), 10)
        self.create_subscription(String, 'scara/vector/placa',
                                 lambda m: setattr(self, 'placa', m.data), 10)

    def on_pos(self, msg: Vector3):
        self.pos_motor = [msg.x, msg.y, msg.z]

    def mandar(self, texto: str):
        self.orden_pub.publish(String(data=texto))
        self.get_logger().info(f'-> vector: {texto}')


class VentanaEfector:

    def __init__(self, node: EfectorNode):
        self.n = node
        self.root = tk.Tk()
        self.root.title('SCARA - punto del efector final')
        self.root.configure(bg=FONDO)
        self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        self.veredicto: A.Veredicto | None = None
        self._estilos()
        self._construir()
        self.zona = A.contorno_planta(self.n.cal, self.n.robot)
        self._escala()
        self.root.bind('<Return>', lambda ev: self.ir())
        self.root.bind('<Escape>', lambda ev: self.n.mandar('parar'))
        self.comprobar()
        self.root.after(100, self._refrescar)

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
        s.configure('TLabelframe', background=FONDO, bordercolor=BORDE)
        s.configure('TLabelframe.Label', background=FONDO, foreground=SUAVE)
        s.configure('Parar.TButton', font=('TkDefaultFont', 10, 'bold'),
                    foreground='white', background=ROJO)
        s.map('Parar.TButton', background=[('active', '#a8322a')])

    def _construir(self):
        raiz = ttk.Frame(self.root, padding=10)
        raiz.pack(fill='both', expand=True)

        izq = ttk.Frame(raiz)
        izq.grid(row=0, column=0, sticky='n')
        ttk.Label(izq, text='Planta, vista desde arriba', style='Titulo.TLabel').pack(anchor='w')
        self.lienzo = tk.Canvas(izq, width=LADO, height=LADO, bg=PAPEL,
                                highlightthickness=1, highlightbackground=BORDE)
        self.lienzo.pack(pady=(6, 0))
        self.lienzo.bind('<Button-1>', self._clic)
        self.lienzo.bind('<Motion>', self._raton)
        self.pie = ttk.Label(izq, text='', style='Suave.TLabel')
        self.pie.pack(anchor='w', pady=(4, 0))

        der = ttk.Frame(raiz, padding=(14, 0, 0, 0))
        der.grid(row=0, column=1, sticky='n')

        caja = ttk.Labelframe(der, text=' Punto del efector final (mm) ', padding=10)
        caja.pack(fill='x')
        self.vars = []
        for i, (eje, defecto) in enumerate((('x', '430'), ('y', '-160'), ('z', '125'))):
            ttk.Label(caja, text=eje).grid(row=0, column=2 * i, padx=(0 if i == 0 else 12, 4))
            v = tk.StringVar(value=defecto)
            v.trace_add('write', lambda *a: self.comprobar())
            ttk.Entry(caja, textvariable=v, width=8, font=('TkFixedFont', 11)).grid(
                row=0, column=2 * i + 1)
            self.vars.append(v)

        self.l_veredicto = tk.Label(der, text='', font=('TkDefaultFont', 15, 'bold'),
                                    bg=FONDO, anchor='w')
        self.l_veredicto.pack(fill='x', pady=(10, 2))
        self.l_motivo = tk.Label(der, text='', bg=FONDO, fg=TINTA, justify='left',
                                 anchor='w', wraplength=380)
        self.l_motivo.pack(fill='x')

        tabla = ttk.Labelframe(der, text=' Ángulos ', padding=10)
        tabla.pack(fill='x', pady=(10, 0))
        self.celdas = {}
        for fila, (clave, texto) in enumerate((('junta', 'junta (cinemática)'),
                                               ('motor', 'motor (main.py)'),
                                               ('limite', 'recorrido motor'),
                                               ('ahora', 'motor ahora'),
                                               ('error', 'error (ahora − meta)'))):
            ttk.Label(tabla, text=texto, style='Suave.TLabel').grid(
                row=fila + 1, column=0, sticky='w', padx=(0, 10))
            for col in range(3):
                c = ttk.Label(tabla, text='', style='Dato.TLabel', width=11, anchor='e')
                c.grid(row=fila + 1, column=col + 1)
                self.celdas[clave, col] = c
        for col, nombre in enumerate(A.MOTORES):
            ttk.Label(tabla, text=nombre, style='Suave.TLabel').grid(row=0, column=col + 1)
        for col, (lo, hi) in enumerate(self.n.cal.limites):
            self.celdas['limite', col].configure(text=f'[{lo:g}, {hi:g}]{A.UNIDADES[col]}')

        self.l_error = ttk.Label(der, text='', style='Dato.TLabel')
        self.l_error.pack(anchor='w', pady=(6, 0))
        self.l_vector = ttk.Label(der, text='', style='Dato.TLabel')
        self.l_vector.pack(anchor='w', pady=(4, 0))

        botones = ttk.Frame(der)
        botones.pack(fill='x', pady=(8, 0))
        self.b_ir = ttk.Button(botones, text='Ir  (Enter)', command=self.ir)
        self.b_ir.pack(side='left')
        ttk.Button(botones, text='PARAR  (Esc)', style='Parar.TButton',
                   command=lambda: self.n.mandar('parar')).pack(side='left', padx=8)

        sup = ttk.Labelframe(der, text=' Supuestos (config/micropython.yaml) ', padding=8)
        sup.pack(fill='x', pady=(12, 0))
        self.l_supuestos = ttk.Label(sup, text=self._supuestos(), justify='left',
                                     style='Suave.TLabel', wraplength=380)
        self.l_supuestos.pack(anchor='w')

        self.l_estado = ttk.Label(der, text='', style='Suave.TLabel', wraplength=380,
                                  justify='left')
        self.l_estado.pack(fill='x', pady=(10, 0))

    def _supuestos(self) -> str:
        c = self.n.cal
        casa = ('0 de Alvin y Simon = brazo ESTIRADO' if c.cero[:2] == [0.0, 0.0]
                else f'0 de Alvin y Simon = junta en {c.cero[0]:g}°, {c.cero[1]:g}°')
        casa += f' | 0 de Teodoro = d3 {c.cero[2]:g} mm'
        if c.paso_husillo_mm == 360.0:
            husillo = 'Teodoro cuenta en mm (main.py lleva sus 41.4 mm por vuelta)'
        elif c.husillo_medido:
            husillo = f'{c.paso_husillo_mm:g} mm/vuelta'
        else:
            husillo = 'SIN MEDIR: la z no se comprueba y Teodoro no se mueve'
        return (f'{casa} | signos {c.signo} | reducción {c.reduccion}\n'
                f'paso del husillo: {husillo}\n'
                f'límites: limites_motor (config/micropython.yaml) y MIN/MAX (main.py)')

    # ------------------------------------------------------------- la planta
    def _escala(self):
        """Encuadre: la base y la zona alcanzable, con margen."""
        xs = [0.0] + [p[0] for p in self.zona]
        ys = [0.0] + [p[1] for p in self.zona]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        ancho = max(max(xs) - min(xs), max(ys) - min(ys)) * 1.25 + 40
        self.k = LADO / ancho
        self.cx, self.cy = cx, cy

    def _px(self, x, y):
        return (LADO / 2 + (x - self.cx) * self.k, LADO / 2 - (y - self.cy) * self.k)

    def _mm(self, px, py):
        return (self.cx + (px - LADO / 2) / self.k, self.cy - (py - LADO / 2) / self.k)

    def _brazo(self, q, color, ancho):
        t1, t2 = q[0], q[1]
        r = self.n.robot
        codo = (r.a1 * math.cos(t1 + r.b1), r.a1 * math.sin(t1 + r.b1))
        punta = R.fk_scara_rrp([t1, t2, 0.0], r)[:2]
        pts = [self._px(0, 0), self._px(*codo), self._px(*punta)]
        self.lienzo.create_line(*[c for p in pts for c in p], fill=color, width=ancho,
                                capstyle='round', joinstyle='round')
        for p in pts[1:]:
            self.lienzo.create_oval(p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3,
                                    fill=color, outline=PAPEL)

    def _dibujar(self):
        L = self.lienzo
        L.delete('all')
        # rejilla cada 100 mm
        x0, y1 = self._mm(0, 0)
        x1, y0 = self._mm(LADO, LADO)
        for v in range(int(x0 // 100) * 100, int(x1) + 100, 100):
            L.create_line(*self._px(v, y0), *self._px(v, y1), fill=REJILLA)
        for v in range(int(y0 // 100) * 100, int(y1) + 100, 100):
            L.create_line(*self._px(x0, v), *self._px(x1, v), fill=REJILLA)
        # alcance maximo del brazo (estirado)
        rmax = self.n.robot.a1 + self.n.robot.a2
        a, b = self._px(-rmax, rmax), self._px(rmax, -rmax)
        L.create_oval(*a, *b, outline=BORDE, dash=(4, 3))
        # la zona alcanzable con los limites de los motores
        L.create_polygon(*[c for p in self.zona for c in self._px(*p)],
                         fill=ZONA, outline=VERDE)
        bx, by = self._px(0, 0)
        L.create_rectangle(bx - 6, by - 6, bx + 6, by + 6, fill=TINTA, outline='')
        L.create_text(bx + 10, by + 12, text='base', fill=SUAVE, anchor='w')
        # el brazo ahora
        if self.n.pos_motor is not None:
            q = self.n.cal.a_junta(self.n.pos_motor)
            self._brazo(q, SUAVE, 4)
        # el objetivo
        v = self.veredicto
        if v is not None:
            px, py = self._px(v.punto[0], v.punto[1])
            if v.alcanzable:
                self._brazo(v.junta, AZUL, 3)
                L.create_oval(px - 6, py - 6, px + 6, py + 6, outline=AZUL, width=2)
            else:
                L.create_line(px - 7, py - 7, px + 7, py + 7, fill=ROJO, width=3)
                L.create_line(px - 7, py + 7, px + 7, py - 7, fill=ROJO, width=3)

    def _clic(self, ev):
        x, y = self._mm(ev.x, ev.y)
        self.vars[0].set(f'{round(x):d}')
        self.vars[1].set(f'{round(y):d}')

    def _raton(self, ev):
        x, y = self._mm(ev.x, ev.y)
        self.pie.configure(text=f'ratón: x {x:.0f}  y {y:.0f}  r {math.hypot(x, y):.0f} mm'
                                f'   ·   zona verde = alcanzable   ·   clic = poner x, y')

    # ------------------------------------------------------------- el calculo
    def comprobar(self):
        try:
            P = tuple(float(v.get().replace(',', '.')) for v in self.vars)
        except ValueError:
            self.veredicto = None
            self.l_veredicto.configure(text='escribe tres números', fg=SUAVE)
            self.l_motivo.configure(text='')
            self.b_ir.state(['disabled'])
            self._dibujar()
            return
        v = A.alcanzar(P, self.n.cal, self.n.robot, self.n.pos_motor)
        self.veredicto = v
        if v.alcanzable:
            self.l_veredicto.configure(text='ALCANZABLE', fg=VERDE)
            aviso = ('' if v.z_comprobada
                     else '   ·   la z no se ha comprobado: falta el paso del husillo')
            self.l_motivo.configure(text=f'codo {v.codo}{aviso}')
            t1, t2, d3 = v.junta
            for col, txt in enumerate((f'{math.degrees(t1):+.1f}°', f'{math.degrees(t2):+.1f}°',
                                       f'{d3:.1f} mm')):
                self.celdas['junta', col].configure(text=txt)
            for col, m in enumerate(v.motor):
                self.celdas['motor', col].configure(
                    text='—' if m is None else f'{m:+.1f}{A.UNIDADES[col]}')
            self.b_ir.state(['!disabled'])
            # Lo que saldra hacia la placa. Sin el paso del husillo, Teodoro se
            # queda donde esta (si aun no se sabe donde, sale '?').
            actual = self.n.pos_motor or [None, None, None]
            vec = A.vector_placa(v.motor, [0.0 if a is None else a for a in actual],
                                 self.n.cal)
            txt = [f'{x:g}' for x in vec]
            if v.motor[2] is None and actual[2] is None:
                txt[4] = '?'
            self.l_vector.configure(text='vector: [' + ', '.join(txt) + ']')
        else:
            self.l_veredicto.configure(text='NO ALCANZABLE', fg=ROJO)
            self.l_motivo.configure(text='\n'.join(v.motivos))
            for clave in ('junta', 'motor'):
                for col in range(3):
                    self.celdas[clave, col].configure(text='—')
            self.b_ir.state(['disabled'])
            self.l_vector.configure(text='vector: no sale')
        self._dibujar()

    def ir(self):
        v = self.veredicto
        if v is None:
            return
        if not v.alcanzable:      # Enter con un punto que no llega: se dice en la consola
            self.n.get_logger().error(v.resumen())
            return
        self.n.mandar('ir ' + ' '.join(f'{c:g}' for c in v.punto))

    def _error_ahora(self, m):
        """Cuanto le falta (o se ha pasado) cada motor y la punta, frente al ultimo
        vector que salio hacia la placa (desde esta ventana, un topic o una ruta)."""
        e = self.n.enviado
        if e is None or m is None:
            for col in range(3):
                self.celdas['error', col].configure(text='—')
            self.l_error.configure(text='error de la punta: — (aún no se ha mandado nada)')
            return
        meta = [e[0], e[2], e[4]]
        for col in range(3):
            self.celdas['error', col].configure(
                text=f'{m[col] - meta[col]:+.2f}{A.UNIDADES[col]}')
        q_meta, q_real = self.n.cal.a_junta(meta), self.n.cal.a_junta(m)
        medido = q_meta[2] is not None
        if not medido:                        # sin paso: la xy no depende de d3
            q_meta[2] = q_real[2] = 0.0
        p_meta = R.fk_scara_rrp(q_meta, self.n.robot)
        p = R.fk_scara_rrp(q_real, self.n.robot)
        exy = math.hypot(p[0] - p_meta[0], p[1] - p_meta[1])
        if medido:
            ez = p[2] - p_meta[2]
            txt = f'{math.hypot(exy, ez):.2f} mm ({exy:.2f} en xy, {ez:+.2f} en z)'
        else:
            txt = f'{exy:.2f} mm en xy (z sin medir)'
        self.l_error.configure(text=f'error de la punta: {txt}')

    def _refrescar(self):
        if not rclpy.ok():
            # Ctrl+C o el launch parado: rclpy se queda la senal y Tk no se entera.
            # Sin esto la ventana se quedaba abierta, huerfana.
            self.root.destroy()
            return
        m = self.n.pos_motor
        for col in range(3):
            self.celdas['ahora', col].configure(
                text='—' if m is None else f'{m[col]:+.1f}{A.UNIDADES[col]}')
        self._error_ahora(m)
        self.l_estado.configure(text=f'vector: {self.n.estado or "sin noticias"}\n'
                                     f'placa: {self.n.placa or "sin noticias"}')
        self._dibujar()
        self.root.after(200, self._refrescar)


def _girar(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:             # noqa: B902
        if rclpy.ok():
            raise


def main(args=None):
    rclpy.init(args=args)
    node = EfectorNode()
    hilo = threading.Thread(target=_girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaEfector(node)
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
