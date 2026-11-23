"""La ventana del control con el brazo de verdad, con el boton «traycho».

    ros2 launch scara_kinematics traycho.launch.py k:=2 vel_max:=0.1

Es la ventana de control_placa_gui HEREDADA: todo lo de alli sigue igual (ir a
un punto, salto, circulo, soltar y activar motores...). Lo que anade:

    Traycho   el boton: recorre los ocho puntos de tu codigo (traycho.py), uno
              detras de otro, desde donde este la punta, y vuelve a P1
    planta    esos ocho puntos y sus rectas, en morado, para ver por donde va a ir

Cualquier otra orden mientras va (Parar, Esc, Ir en recta, Soltar motores...)
corta el traycho: control_nodo hace solo una cosa a la vez.
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from scara_kinematics import control_gui as G
from scara_kinematics import control_placa_gui as PG
from scara_kinematics import ruta as R
from scara_kinematics import traycho as T
from scara_kinematics.estilo import NARANJA, ROJO, VERDE

MORADO = '#7b4fc4'


class TraychoGuiNode(PG.PlacaGuiNode):

    recorrido: T.Recorrido | None = None

    def on_rosout(self, msg: Log):
        super().on_rosout(msg)
        r = self.recorrido
        if r is not None and r.activo():
            r.aviso(msg.name.split('.')[-1], int(msg.level), msg.msg)

    def mandar(self, texto: str) -> bool:
        """Las ordenes de la ventana: si va el traycho, lo cortan."""
        r = self.recorrido
        if r is not None and r.activo():
            r.cortar(f'has mandado «{texto}»')
        return super().mandar(texto)

    def mandar_placa(self, texto: str):
        r = self.recorrido
        if r is not None and r.activo():
            r.cortar(f'has mandado «{texto}» a la placa')
        super().mandar_placa(texto)

    def mandar_traycho(self, texto: str) -> bool:
        if not self.escucha():
            return False
        self.orden_pub.publish(String(data=texto))
        self.err_max = 0.0
        return True


class VentanaTraycho(PG.VentanaPlaca):

    def __init__(self, node: TraychoGuiNode):
        self.pts = T.puntos_mm()
        super().__init__(node)
        self.root.title('SCARA - control cinemático (brazo real) · traycho')

    def _construir_ordenes(self, padre):
        col = super()._construir_ordenes(padre)
        t = ttk.Labelframe(col, text=' Traycho (tu código) ', padding=6)
        t.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self._boton(t, 'traycho', self._traycho, 'Ir.TButton', row=0, column=0, sticky='w')
        dx, dy = T.DESPLAZAMIENTO_CM
        ttk.Label(t, text=f'  {len(self.pts)} puntos, {dx:+g} cm en X y {dy:+g} en Y',
                  style='Suave.TLabel').grid(row=0, column=1, sticky='w')
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
            tramos = T.tramos(self.pts, n.robot, desde=n.punta)
        except R.RutaError as e:
            n.log.put((40, f'traycho: no cabe, no se manda nada. {e}. Si el brazo está en '
                           'HOME, Simon está en su tope: mételo unos grados '
                           '(CONTROL_PLACA.md §3)'))
            return
        n.log.put(('tu', f'traycho: {len(tramos)} rectas por los {len(self.pts)} puntos '
                         f'(ninguna junta gira más de {T.GRADOS_MAX:g}° en cada una)'))
        n.recorrido = T.Recorrido(tramos, n.mandar_traycho,
                                  lambda nivel, texto: n.log.put((nivel, texto)))

    def _refrescar(self):
        super()._refrescar()
        if not rclpy.ok():
            return
        r = self.n.recorrido
        if r is None:
            return
        r.paso()
        try:
            if r.fase == 'hecho':
                self.lbl_traycho.configure(text='terminado: ha vuelto a P1', foreground=VERDE)
            elif r.fase == 'cortado':
                self.lbl_traycho.configure(text=f'cortado en {r.nombre()}', foreground=ROJO)
            else:
                self.lbl_traycho.configure(
                    text=f'va a {r.nombre()}   ·   recta {r.i + 1} de {len(r.tramos)}',
                    foreground=NARANJA)
        except tk.TclError:
            pass

    def _pintar(self):
        super()._pintar()
        c = self.lienzo
        xy = [v for p in self.pts for v in self._px(p[0], p[1])]
        c.create_line(*xy, fill=MORADO, width=1, dash=(5, 3), tags='dyn')
        for i, p in enumerate(self.pts[:-1], 1):
            u, w = self._px(p[0], p[1])
            c.create_oval(u - 3, w - 3, u + 3, w + 3, fill=MORADO, outline='', tags='dyn')
            c.create_text(u + 5, w - 5, text=f'P{i}', anchor='sw', fill=MORADO,
                          font=('TkDefaultFont', 8), tags='dyn')
        r = self.n.recorrido
        if r is not None and r.activo():
            u, w = self._px(*r.tramos[r.i][1][:2])
            c.create_oval(u - 6, w - 6, u + 6, w + 6, outline=MORADO, width=2, tags='dyn')


def main(args=None):
    rclpy.init(args=args)
    node = TraychoGuiNode()
    hilo = threading.Thread(target=G._girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaTraycho(node)
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
