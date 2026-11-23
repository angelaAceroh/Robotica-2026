"""La ventana del traycho por juntas CON LA GARRA (GARRA.md).

    ros2 launch scara_kinematics traycho_garra.launch.py

Es la de juntas_gui HEREDADA (que no se ha tocado). Lo que cambia:

  - el boton «traycho» hace el mismo recorrido, y en cada punto abre o cierra
    la garra (garra_traycho.ACCIONES): abre al salir de HOME, abre en P1,
    cierra en P2, abre en P3, cierra en P4, abre en P5 y cierra en P6;
  - debajo del boton, Abrir (1) y Cerrar (0), para moverla a mano, y como esta
    (lo ultimo que ha dicho la placa). Como cualquier otra orden, cortan el
    traycho si va.
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from scara_kinematics import control_gui as G
from scara_kinematics import garra_traycho as GT
from scara_kinematics import juntas as J
from scara_kinematics import juntas_gui as JG
from scara_kinematics import traycho as T
from scara_kinematics import traycho_gui as TG
from scara_kinematics.estilo import NARANJA, ROJO, SUAVE, VERDE


class GarraGuiNode(TG.TraychoGuiNode):

    garra = ''                    # lo ultimo que ha dicho: 'abierta 10' | 'cerrada 50'
    corriente = None              # tras el ultimo empujon: (le llego, mV que bajo el 3V3)

    def on_rosout(self, msg: Log):
        super().on_rosout(msg)
        if msg.name.split('.')[-1] == T.NODO_PLACA and msg.msg.startswith('placa: GARRA '):
            texto = msg.msg[len('placa: GARRA '):].strip()
            if texto.startswith('corriente '):          # «corriente si 430» | «corriente NO 5 (...)»
                partes = texto.split()
                self.corriente = (partes[1] == 'si', int(partes[2]))
            else:
                self.garra, self.corriente = texto, None

    def mandar_garra(self, texto: str) -> bool:
        """La orden de la garra del traycho: a la placa, sin cortarlo."""
        if self.placa_pub.get_subscription_count() == 0:
            return False
        self.placa_pub.publish(String(data=texto))
        return True


class VentanaGarra(JG.VentanaJuntas):

    def __init__(self, node: GarraGuiNode):
        # Los puntos de abajo, no por debajo del tope de Teodoro (teodoro_min, que
        # traycho_garra.launch.py pone a GT.TEODORO_SUELO_MM): la mano llega al suelo
        # antes. VentanaJuntas los coge de J.puntos_mm() al crearse
        puntos = J.puntos_mm
        J.puntos_mm = lambda *a, **k: GT.con_suelo(puntos(*a, **k), GT.z_suelo(node.robot))
        try:
            super().__init__(node)
        finally:
            J.puntos_mm = puntos
        self.root.title('SCARA - brazo real, por juntas · traycho con garra')
        node.log.put(('tu', f'con la garra, abajo hasta z {GT.z_suelo(node.robot):.1f} mm '
                            '(teodoro_min: la mano llega al suelo antes)'))

    def _construir_ordenes(self, padre):
        # La garra, una fila mas en el recuadro del traycho: un recuadro aparte
        # no cabe (la ventana pedia 733 px de alto y en la pantalla caben 699)
        col = super()._construir_ordenes(padre)
        t = self.lbl_traycho.master
        t.configure(text=' Traycho por juntas con garra (tu código) ')
        g = ttk.Frame(t)
        g.grid(row=2, column=0, columnspan=2, sticky='w', pady=(6, 0))
        self._boton(g, 'Abrir (1)', lambda: self.n.mandar_placa(GT.ABRIR), row=0, column=0)
        self._boton(g, 'Cerrar (0)', lambda: self.n.mandar_placa(GT.CERRAR),
                    row=0, column=1, padx=(4, 0))
        self.lbl_garra = ttk.Label(g, text='  garra: --', style='Suave.TLabel')
        self.lbl_garra.grid(row=0, column=2, sticky='w')
        # Si al servo le llego corriente en el ultimo empujon (garra.py lo mide en su 3V3)
        self.lbl_corriente = ttk.Label(g, text='', style='Suave.TLabel')
        self.lbl_corriente.grid(row=1, column=0, columnspan=3, sticky='w')
        return col

    def _traycho(self):
        # El de VentanaJuntas (mismas comprobaciones y mismos tramos); si sale,
        # su Recorrido se cambia por el de la garra antes de que mande nada
        n = self.n
        antes = n.recorrido
        super()._traycho()
        r = n.recorrido
        if r is not None and r is not antes:
            n.recorrido = GT.RecorridoGarra(r.tramos, r.mandar, r.decir, n.mandar_garra)
            n.log.put(('tu', f'con la garra: {GT.resumen()}'))

    def _refrescar(self):
        super()._refrescar()
        if not rclpy.ok():
            return
        try:
            garra = self.n.garra
            self.lbl_garra.configure(
                text=f'  garra {garra}°' if garra else '  garra: --',
                foreground=VERDE if garra.startswith('abierta') else (
                    NARANJA if garra else SUAVE))
            c = self.n.corriente
            self.lbl_corriente.configure(
                text='' if c is None else (
                    f'servo con corriente (3V3 −{c[1]} mV)' if c[0] else
                    'servo SIN corriente: mira el contacto del rojo y el marrón'),
                foreground=SUAVE if c is None or c[0] else ROJO)
            r = self.n.recorrido
            if r is not None and r.fase == 'garra' and r.garra is not None:
                orden, donde, _ = r.garra
                self.lbl_traycho.configure(text=f'garra {donde}: {GT.QUE[orden]}',
                                           foreground=NARANJA)
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GarraGuiNode()
    hilo = threading.Thread(target=G._girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaGarra(node)
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
