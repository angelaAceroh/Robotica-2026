"""La ventana del traycho POR JUNTAS (juntas.py, juntas_nodo).

    ros2 launch scara_kinematics traycho_juntas.launch.py

Es la de traycho_gui HEREDADA, con el mismo boton «traycho». Lo que cambia:

  - los puntos: P1 es el HOME (juntas.puntos_mm: la figura de su codigo girada
    alrededor de P1 para que quepa), no los de traycho.py;
  - como reparte el camino: cada punto, y entre uno y otro trozos iguales en
    juntas, de no mas de 20 grados (juntas.tramos_juntas). En planta va con
    Teodoro arriba (la z de P1); en cada punto mas bajo, baja y sube en
    vertical. Se puede salir desde HOME aunque Simon este en su tope.

De lo demas de la ventana, juntas_nodo solo hace ir, home y parar.
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy

from scara_kinematics import control_gui as G
from scara_kinematics import control_placa_gui as PG
from scara_kinematics import juntas as J
from scara_kinematics import ruta as R
from scara_kinematics import traycho as T
from scara_kinematics import traycho_gui as TG


class VentanaJuntas(TG.VentanaTraycho):

    def __init__(self, node: TG.TraychoGuiNode):
        # Lo de VentanaTraycho.__init__, con los puntos de juntas.py; y el camino
        # de verdad, no rectas (la de P2 a P3 cruzaba la zona blanca)
        self.pts = J.puntos_mm()
        self.camino = J.camino_planta(self.pts, node.robot, z_arriba=float(self.pts[0][2]))
        PG.VentanaPlaca.__init__(self, node)
        self.root.title('SCARA - brazo real, por juntas · traycho')
        # La z de «Ir a un punto» y del circulo, arriba: la de control_placa_gui
        # (150) queda por debajo del tope de Teodoro (el soporte)
        for clave in ('z', 'cz'):
            self.v[clave].set(f'{self.pts[0][2]:.0f}')

    def _fondo(self):
        # La cruz «home» de control_gui esta en el HOME de Gazebo (300, 0); la
        # del brazo es P1 (24/10: «pon bien el home que es P1»), que es adonde
        # va el boton Home con juntas_nodo
        antes = G.R.HOME
        G.R.HOME = np.asarray(self.pts[0], dtype=float)
        try:
            PG.VentanaPlaca._fondo(self)
        finally:
            G.R.HOME = antes

    def _construir_ordenes(self, padre):
        # El de VentanaTraycho, con el texto de estos puntos
        col = PG.VentanaPlaca._construir_ordenes(self, padre)
        t = ttk.Labelframe(col, text=' Traycho por juntas (tu código) ', padding=6)
        t.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self._boton(t, 'traycho', self._traycho, 'Ir.TButton', row=0, column=0, sticky='w')
        ttk.Label(t, text=f'  {len(self.pts)} puntos, P1 en HOME, girados '
                          f'{J.GIRO_GRADOS:+g}°', style='Suave.TLabel').grid(row=0, column=1,
                                                                              sticky='w')
        self.lbl_traycho = ttk.Label(t, text='en la planta, en morado', style='Suave.TLabel')
        self.lbl_traycho.grid(row=1, column=0, columnspan=2, sticky='w', pady=(4, 0))
        return col

    def _traycho(self):
        n = self.n
        if n.recorrido is not None and n.recorrido.activo():
            n.log.put((30, 'traycho ya está en marcha (Parar o Esc lo corta)'))
            return
        if not n.placa_activa():
            n.log.put((40, 'traycho: los motores están sueltos. «Activar motores» primero'))
            return
        if n.punta is None:
            n.log.put((40, 'traycho: no sé dónde está la punta (¿llegan los encoders?)'))
            return
        try:
            tramos = J.tramos_juntas(self.pts, n.robot, desde=n.punta,
                                     z_arriba=float(self.pts[0][2]))
        except R.RutaError as e:
            n.log.put((40, f'traycho: no cabe, no se manda nada. {e}'))
            return
        n.log.put(('tu', f'traycho por juntas: {len(tramos)} idas por los {len(self.pts)} '
                         f'puntos (ninguna junta gira más de {T.GRADOS_MAX:g}° en cada una); '
                         f'en planta va arriba, y en cada punto baja y sube en vertical; '
                         f'corrige al llegar a cada una'))
        n.recorrido = T.Recorrido(tramos, n.mandar_traycho,
                                  lambda nivel, texto: n.log.put((nivel, texto)))


    def _pintar(self):
        # El de VentanaTraycho, con el camino por juntas en vez de las rectas
        PG.VentanaPlaca._pintar(self)
        c = self.lienzo
        xy = [v for p in self.camino for v in self._px(p[0], p[1])]
        c.create_line(*xy, fill=TG.MORADO, width=1, dash=(5, 3), tags='dyn')
        for i, p in enumerate(self.pts[:-1], 1):
            u, w = self._px(p[0], p[1])
            c.create_oval(u - 3, w - 3, u + 3, w + 3, fill=TG.MORADO, outline='', tags='dyn')
            c.create_text(u + 5, w - 5, text=f'P{i}', anchor='sw', fill=TG.MORADO,
                          font=('TkDefaultFont', 8), tags='dyn')
        r = self.n.recorrido
        if r is not None and r.activo():
            u, w = self._px(*r.tramos[r.i][1][:2])
            c.create_oval(u - 6, w - 6, u + 6, w + 6, outline=TG.MORADO, width=2, tags='dyn')


def main(args=None):
    rclpy.init(args=args)
    node = TG.TraychoGuiNode()
    hilo = threading.Thread(target=G._girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaJuntas(node)
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
    main()
