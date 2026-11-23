"""La ventana del control cinematico con el brazo de verdad.

    ros2 launch scara_kinematics control_placa.launch.py      # la abre sola

Es la ventana de Gazebo (control_gui) HEREDADA, no copiada: las ordenes, la
grafica del error y los mensajes funcionan igual, porque todo sigue yendo a
control_nodo por scara/control/orden. Lo que cambia:

    la planta    ensena la franja que alcanza el brazo con los recorridos medidos
                 de los motores (los limites que le pasa el launch), no el anillo
                 entero del CAD; y los campos empiezan dentro de ella
    cabecera     el boton de pausar Gazebo es ahora «Soltar motores» / «Activar
                 motores» (scara/placa/orden, lo atiende puente_placa)
    Ahora        dos filas mas: la placa (activa o suelta, enlace) y los motores
                 (grados o mm, y lo que va cada uno por detras de su consigna)
    mensajes     tambien los de puente_placa

    Esc     parar: control_nodo se queda regulando donde esta (los motores siguen
            activos, sujetando). Para soltarlos, el boton de la cabecera.
"""
from __future__ import annotations

import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from scara_kinematics import control_gui as G
from scara_kinematics import placa_vel as PV
from scara_kinematics.estilo import NARANJA, ROJO, SUAVE, TINTA, VERDE

MARGEN_VISTA = 35.0      # mm alrededor de la franja que alcanza la punta
VIVA_S = 1.0             # sin estado de puente_placa en este tiempo: no esta

TEXTOS = {
    '→  Gazebo, juntas por velocidad': '→  la placa (main_control.py), motores por velocidad',
    'la punta en Gazebo': 'la punta (encoders)',
}


class PlacaGuiNode(G.ControlGuiNode):

    def __init__(self):
        super().__init__()
        self.placa = ''
        self.t_placa = 0.0
        self.motores: tuple | None = None
        self.seguimiento: tuple | None = None
        self.placa_pub = self.create_publisher(String, 'scara/placa/orden', 10)
        self.create_subscription(String, 'scara/placa/estado', self.on_placa, 10)
        self.create_subscription(Vector3, 'scara/placa/motores', self.on_motores, 10)
        self.create_subscription(Vector3, 'scara/placa/seguimiento', self.on_seguimiento, 10)

    def on_placa(self, msg: String):
        self.placa = msg.data
        self.t_placa = time.monotonic()

    def on_motores(self, msg: Vector3):
        self.motores = (msg.x, msg.y, msg.z)

    def on_seguimiento(self, msg: Vector3):
        self.seguimiento = (msg.x, msg.y, msg.z)

    def on_rosout(self, msg: Log):
        if msg.name == 'control':
            self.log.put((int(msg.level), msg.msg))
        elif msg.name == 'puente_placa' and not msg.msg.startswith('-> placa: V'):
            self.log.put((int(msg.level), msg.msg))

    def placa_viva(self) -> bool:
        return time.monotonic() - self.t_placa < VIVA_S and self.placa.startswith(
            ('ACTIVA', 'SUELTA'))

    def placa_activa(self) -> bool:
        return self.placa_viva() and self.placa.startswith('ACTIVA')

    def mandar_placa(self, texto: str):
        if self.placa_pub.get_subscription_count() == 0:
            self.log.put((40, f'«{texto}» no sale: puente_placa no escucha'))
            return
        self.placa_pub.publish(String(data=texto))
        self.log.put(('tu', f'placa: {texto}'))


def _vista(caja):
    """Un cuadrado alrededor de la caja (x0, x1, y0, y1), con margen."""
    x0, x1, y0, y1 = caja
    lado = max(x1 - x0, y1 - y0) + 2.0 * MARGEN_VISTA
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return (cx - lado / 2.0, cx + lado / 2.0), (cy - lado / 2.0, cy + lado / 2.0)


class VentanaPlaca(G.VentanaControl):

    def __init__(self, node: PlacaGuiNode):
        zona = PV.zona_trabajo(node.robot)
        # control_gui lee la vista de estas dos variables de su modulo al pintar
        G.VISTA_X, G.VISTA_Y = _vista(zona['caja'])
        super().__init__(node)
        self.root.title('SCARA - control cinemático (brazo real)')

        # Los campos, dentro de lo que alcanza el brazo
        cx, cy, _ = zona['centro']
        r = max(5, min(15, int(zona['holgura'] * 0.5)))
        for clave, valor in (('x', cx), ('y', cy), ('z', 150), ('v', 20),
                             ('cx', cx), ('cy', cy), ('r', r), ('cz', 150), ('cv', 10),
                             ('dx', 0), ('dy', 5), ('dz', 0)):
            self.v[clave].set(f'{valor:.0f}')

        self._retocar(self.root)
        self.b_gz.configure(width=16)
        marco = self.lbl_estado.master
        for i, clave in enumerate(('placa', 'motores')):
            ttk.Label(marco, text=clave, style='Suave.TLabel').grid(
                row=7 + i, column=0, sticky='w', padx=(0, 10))
            self.datos[clave] = ttk.Label(marco, text='--', style='Dato.TLabel')
            self.datos[clave].grid(row=7 + i, column=1, sticky='w')

    def _retocar(self, w):
        """Los textos de Gazebo que pone control_gui al construir la ventana."""
        for hijo in w.winfo_children():
            try:
                texto = hijo.cget('text')
            except tk.TclError:
                texto = ''
            if isinstance(texto, str) and 'Gazebo' in texto:
                for viejo, nuevo in TEXTOS.items():
                    texto = texto.replace(viejo, nuevo)
                hijo.configure(text=texto.replace('Gazebo', 'el brazo'))
            self._retocar(hijo)

    def _alternar_gz(self):
        self.n.mandar_placa('soltar' if self.n.placa_activa() else 'activar')

    def _refrescar(self):
        super()._refrescar()
        if not rclpy.ok():
            return                      # control_gui ya ha cerrado la ventana
        try:
            self._refrescar_placa()
        except tk.TclError:
            pass

    def _refrescar_placa(self):
        n = self.n
        viva, activa = n.placa_viva(), n.placa_activa()
        self.b_gz.configure(text='Soltar motores' if activa else 'Activar motores')
        self.b_gz.state(['!disabled'] if viva else ['disabled'])
        if not viva:
            self.lbl_ctrl.configure(text='placa: no contesta', foreground=ROJO)
        elif not activa:
            self.lbl_ctrl.configure(text='motores sueltos', foreground=NARANJA)
        else:
            t = self.lbl_ctrl.cget('text')
            self.lbl_ctrl.configure(text=t.replace('de Gazebo', 'de la placa'))
        t = self.lbl_estado.cget('text')
        if 'Gazebo' in t:
            self.lbl_estado.configure(text=t.replace('Gazebo', 'la placa'))

        partes = n.placa.split(' | ')
        if viva:
            texto = ' · '.join(p for p in partes[:3])
            color = VERDE if activa else NARANJA
        else:
            texto, color = (partes[0] if n.placa else 'puente_placa no está'), ROJO
        self.datos['placa'].configure(text=texto, foreground=color)
        if n.motores is not None and viva:
            seg = n.seguimiento if (n.seguimiento is not None and activa) else None
            self.datos['motores'].configure(text='  '.join(
                f'{nombre[0]} {m:+7.2f}{u}' + (f' ({s:+.2f})' if seg is not None else '')
                for nombre, m, s, u in zip(PV.MOTORES, n.motores, seg or (0, 0, 0),
                                           PV.UNIDADES)),
                foreground=TINTA)
        else:
            self.datos['motores'].configure(text='--', foreground=SUAVE)


def main(args=None):
    rclpy.init(args=args)
    node = PlacaGuiNode()
    hilo = threading.Thread(target=G._girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaPlaca(node)
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
