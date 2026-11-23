"""La interfaz del brazo: cero, potencia y las tres juntas del SCARA real.

    ros2 launch scara_kinematics brazo.launch.py

Es la unica ventana del paquete. Da el cero y la potencia al brazo, ensena junta
a junta la consigna que la aduana manda a la placa, lo que mide Gazebo si esta
levantado y lo que arma cc con los encoders, y deja probar cada motor en lazo
abierto (orden A del firmware) o teclearle ordenes a mano.

No calcula nada: todo lo que ve llega de los otros nodos y todo lo que manda sale
por la aduana, que es la unica que tiene el puerto serie abierto.

    consigna  scara/hw/cmd        lo que la aduana esta escribiendo en la placa
    Gazebo    gz/joint_states     la simulacion, si corre
    real      joint_states        lo que arma cc con Alvin, Simon y Teodoro

Al dar potencia la consigna salta a donde esta el brazo real: si no, en cuanto el
firmware cerrase el lazo el brazo iria de golpe hacia donde estuviera la consigna.

    Esc   corta la potencia del brazo real
"""
from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8
from std_srvs.srv import SetBool, Trigger

from scara_kinematics.estilo import (AZUL, BORDE, FONDO, NARANJA, PAPEL, ROJO,
                                     SUAVE, TINTA, VERDE)
from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import (F_CERO, F_FALLO, F_HABILITADO, F_LIMITE,
                                        F_SATURACION, POR_JUNTA)

VIVO_S = 1.0             # sin noticias en este tiempo, esa parte se da por caida


class BrazoHwNode(Node):

    def __init__(self):
        super().__init__('scara_brazo_hw')
        self.model = declare_model_params(self)
        self.nombres = list(self.model.joint_names)
        n = len(self.nombres)
        self.consigna: list[float | None] = [None] * n
        self.gz: list[float | None] = [None] * n
        self.real: list[float | None] = [None] * n
        self.u = [0.0] * n
        self.t_cmd = self.t_gz = self.t_hw = 0.0
        self.flags = 0
        self.log: queue.Queue = queue.Queue()

        # La consigna es la que la aduana esta mandando a la placa; lo "real" es
        # el brazo que arma cc con las tres juntas; Gazebo, si esta, llega remapeado
        # para no pelearse con cc por /joint_states.
        self.create_subscription(JointState, 'scara/hw/cmd', self.on_cmd, 10)
        self.create_subscription(JointState, 'gz/joint_states', self.on_gz, 10)
        self.create_subscription(JointState, 'joint_states', self.on_hw, 10)
        self.create_subscription(UInt8, 'scara/hw/flags', self.on_flags, 10)
        self.create_subscription(String, 'scara/hw/log', lambda msg: self.log.put(msg.data), 50)
        self.raw_pub = self.create_publisher(String, 'scara/hw/raw', 10)
        self.set_pub = self.create_publisher(JointState, 'scara/hw/cmd', 10)
        self.cli_cero = self.create_client(Trigger, 'scara/hw/zero')
        self.cli_potencia = self.create_client(SetBool, 'scara/hw/enable')

    # -------------------------------------------------------------- entradas
    def _extraer(self, msg: JointState, destino: list, esfuerzo: list | None = None):
        for i, nombre in enumerate(self.nombres):
            try:
                k = msg.name.index(nombre)
            except ValueError:
                continue
            if k < len(msg.position):
                destino[i] = msg.position[k]
            if esfuerzo is not None and k < len(msg.effort):
                esfuerzo[i] = msg.effort[k]

    def on_cmd(self, msg: JointState):
        self._extraer(msg, self.consigna)
        self.t_cmd = time.monotonic()

    def on_gz(self, msg: JointState):
        self._extraer(msg, self.gz)
        self.t_gz = time.monotonic()

    def on_hw(self, msg: JointState):
        self._extraer(msg, self.real, self.u)
        self.t_hw = time.monotonic()

    def on_flags(self, msg: UInt8):
        self.flags = msg.data

    @staticmethod
    def vivo(t: float) -> bool:
        return time.monotonic() - t < VIVO_S

    def flags_vivos(self) -> int:
        return self.flags if self.vivo(self.t_hw) else 0

    # --------------------------------------------------------------- ordenes
    def _llamar(self, cliente, peticion, que: str):
        if not cliente.service_is_ready():
            self.log.put(f'{que}: el servicio no esta (la aduana no ha arrancado)')
            return

        def hecho(fut):
            try:
                self.log.put(f'{que}: {fut.result().message}')
            except Exception as exc:      # el nodo se cayo a medias, por ejemplo
                self.log.put(f'{que}: sin respuesta ({exc})')
        cliente.call_async(peticion).add_done_callback(hecho)

    def fijar_cero(self):
        self._llamar(self.cli_cero, Trigger.Request(), 'cero')

    def potencia(self, si: bool):
        self._llamar(self.cli_potencia, SetBool.Request(data=si), 'potencia')

    def sincronizar(self) -> bool:
        """Pone la consigna donde esta el brazo real, sin rampa."""
        if any(v is None for v in self.real):
            return False
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.nombres)
        js.position = [float(v) for v in self.real]
        self.set_pub.publish(js)
        return True

    def mandar(self, texto: str):
        self.raw_pub.publish(String(data=texto))


class VentanaBrazo:

    def __init__(self, node: BrazoHwNode):
        self.n = node
        self.root = tk.Tk()
        self.root.title('SCARA - brazo real (ESP32)')
        self.root.configure(bg=FONDO)
        self.root.protocol('WM_DELETE_WINDOW', self.cerrar)
        self._estilos()
        self._construir()
        self.root.bind('<Escape>', lambda ev: self.parar())
        self._tarea = self.root.after(50, self._refrescar)

    # ---------------------------------------------------------------- aspecto
    def _estilos(self):
        # Los colores viven en estilo.py, que era la paleta del panel.
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
        s.configure('Parar.TButton', font=('TkDefaultFont', 11, 'bold'),
                    foreground='white', background=ROJO, padding=6)
        s.map('Parar.TButton', background=[('active', '#a8322a'), ('disabled', '#a8b0bd')])

    def _construir(self):
        raiz = ttk.Frame(self.root, padding=12)
        raiz.pack(fill='both', expand=True)
        raiz.columnconfigure(0, weight=1)

        cab = ttk.Frame(raiz)
        cab.grid(row=0, column=0, sticky='ew', pady=(0, 10))
        ttk.Label(cab, text='Brazo real', style='Titulo.TLabel').pack(side='left')
        ttk.Label(cab, text='   aduana → Alvin · Simón · Teodoro → cc → esp_rec',
                  style='Suave.TLabel').pack(side='left')
        self.estado_hw = ttk.Label(cab, text='', style='Estado.TLabel')
        self.estado_hw.pack(side='right')
        self.estado_gz = ttk.Label(cab, text='', style='Estado.TLabel')
        self.estado_gz.pack(side='right', padx=14)

        tabla = ttk.Labelframe(raiz, text=' Juntas ', padding=10)
        tabla.grid(row=1, column=0, sticky='ew')
        for col, (texto, color) in enumerate((('Motor', SUAVE), ('Consigna', NARANJA),
                                              ('Gazebo', VERDE), ('Real', AZUL),
                                              ('PWM', SUAVE), ('Probar motor', SUAVE))):
            tk.Label(tabla, text=texto, fg=color, bg=FONDO,
                     font=('TkDefaultFont', 9, 'bold')).grid(
                row=0, column=col, sticky='w', padx=(0, 18), pady=(0, 4))
        self.celdas = []
        self.b_prueba = []
        for i, nombre in enumerate(self.n.nombres):
            m = POR_JUNTA.get(nombre)
            motor, que = (m.nombre, m.que) if m else (nombre, '')
            ttk.Label(tabla, text=f'{motor} ({que})').grid(
                row=i + 1, column=0, sticky='w', padx=(0, 18), pady=2)
            fila = []
            for col in range(1, 5):
                lbl = ttk.Label(tabla, text='--', style='Dato.TLabel', width=10)
                lbl.grid(row=i + 1, column=col, sticky='w', padx=(0, 18))
                fila.append(lbl)
            self.celdas.append(fila)
            caja = ttk.Frame(tabla)
            caja.grid(row=i + 1, column=5, sticky='w')
            for texto, signo in (('◀', -1), ('▶', 1)):
                b = ttk.Button(caja, text=texto, width=3,
                               command=lambda i=i, s=signo: self.probar(i, s))
                b.pack(side='left', padx=(0, 4))
                self.b_prueba.append(b)

        mandos = ttk.Frame(raiz)
        mandos.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        self.b_cero = ttk.Button(mandos, text='Fijar cero (brazo en reposo)',
                                 command=self.n.fijar_cero)
        self.b_cero.pack(side='left')
        self.b_potencia = ttk.Button(mandos, text='Dar potencia', width=15,
                                     command=self.alternar_potencia)
        self.b_potencia.pack(side='left', padx=6)
        self.b_parar = ttk.Button(mandos, text='PARAR  (Esc)', style='Parar.TButton',
                                  command=self.parar)
        self.b_parar.pack(side='right')

        self.avisos = tk.Label(raiz, text='', bg=FONDO, fg='white',
                               font=('TkDefaultFont', 9, 'bold'), padx=8)
        self.avisos.grid(row=3, column=0, sticky='w', pady=(8, 0))
        self.pista = ttk.Label(raiz, text='', style='Suave.TLabel', wraplength=660,
                               justify='left')
        self.pista.grid(row=4, column=0, sticky='ew', pady=(4, 0))

        reg = ttk.Labelframe(raiz, text=' Placa, y ordenes a mano (K, P, D, I, ?) ', padding=8)
        reg.grid(row=5, column=0, sticky='ew', pady=(10, 0))
        reg.columnconfigure(0, weight=1)
        self.registro = tk.Text(reg, height=7, font=('TkFixedFont', 9), bg=PAPEL, fg=TINTA,
                                relief='flat', highlightthickness=1, highlightbackground=BORDE,
                                wrap='word', state='disabled')
        self.registro.grid(row=0, column=0, columnspan=2, sticky='ew')
        self.orden_var = tk.StringVar()
        self.e_orden = ttk.Entry(reg, textvariable=self.orden_var)
        self.e_orden.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        self.e_orden.bind('<Return>', lambda ev: self.enviar_orden())
        self.b_orden = ttk.Button(reg, text='Enviar', command=self.enviar_orden)
        self.b_orden.grid(row=1, column=1, pady=(6, 0), padx=(6, 0))

        pines = '   ·   '.join(
            f'{m.nombre}: IN {m.pines[0]}/{m.pines[1]} ENA {m.pines[2]} '
            f'enc {m.encoder()}'
            for m in (POR_JUNTA[nm] for nm in self.n.nombres if nm in POR_JUNTA))
        ttk.Label(raiz, text=pines, style='Suave.TLabel').grid(
            row=6, column=0, sticky='w', pady=(8, 0))

    # ---------------------------------------------------------------- ordenes
    def alternar_potencia(self):
        if self.n.flags_vivos() & F_HABILITADO:
            self.parar()
            return
        # Antes de dar potencia, la consigna salta a donde esta el brazo real: el
        # firmware arranca el lazo en su posicion, y una consigna vieja lo
        # mandaria alli de golpe.
        if not self.n.sincronizar():
            self._al_registro('sin medida del brazo real: no puedo igualar la consigna')
            return
        self.root.after(150, lambda: self.n.potencia(True))

    def parar(self):
        self.n.potencia(False)

    def probar(self, i: int, signo: int):
        m = POR_JUNTA.get(self.n.nombres[i])
        u, ms = (m.empujon, m.ms) if m else (0.5, 150)
        orden = f'A {i} {signo * u:+.2f} {ms}'
        self._al_registro(f'tu: {orden}')
        self.n.mandar(orden)

    def enviar_orden(self):
        texto = self.orden_var.get().strip()
        if texto:
            self._al_registro(f'tu: {texto}')
            self.n.mandar(texto)
            self.orden_var.set('')

    def cerrar(self):
        self.root.after_cancel(self._tarea)
        # Como la aduana al morir: sin la ventana nadie esta mirando el brazo.
        self.n.mandar('E 0')
        time.sleep(0.2)                # a que el mensaje salga antes de apagar ROS
        self.root.destroy()

    # --------------------------------------------------------------- refresco
    def _fmt(self, i: int, q: float | None) -> str:
        if q is None:
            return '--'
        return f'{q * 1000:6.1f} mm' if i == 2 else f'{math.degrees(q):+7.1f}°'

    def _refrescar(self):
        # Ctrl+C en el launch apaga rclpy, pero eso no saca al mainloop de Tk.
        if not rclpy.ok():
            self.root.destroy()
            return
        self._tarea = self.root.after(50, self._refrescar)
        n = self.n
        while True:
            try:
                self._al_registro(n.log.get_nowait())
            except queue.Empty:
                break

        gz, hw, ik = n.vivo(n.t_gz), n.vivo(n.t_hw), n.vivo(n.t_cmd)
        f = n.flags_vivos()
        on, cero, fallo = bool(f & F_HABILITADO), bool(f & F_CERO), bool(f & F_FALLO)

        self.estado_gz.configure(text='Gazebo: recibiendo' if gz else 'Gazebo: sin datos',
                                 foreground=VERDE if gz else ROJO)
        if not hw:
            texto, color = 'ESP32: sin enlace', ROJO
        elif fallo and not on:
            texto, color = 'ESP32: fallo', ROJO
        elif not cero:
            texto, color = 'ESP32: sin cero', NARANJA
        elif not on:
            texto, color = 'ESP32: sin potencia', NARANJA
        else:
            texto, color = 'ESP32: siguiendo', VERDE
        self.estado_hw.configure(text=texto, foreground=color)

        for i, fila in enumerate(self.celdas):
            fila[0].configure(text=self._fmt(i, n.consigna[i]) if ik else '--')
            fila[1].configure(text=self._fmt(i, n.gz[i]) if gz else '--')
            if hw and cero:
                fila[2].configure(text=self._fmt(i, n.real[i]))
            else:
                fila[2].configure(text='sin cero' if hw else '--')
            fila[3].configure(text=f'{n.u[i] * 100:+.0f} %' if hw else '--')

        avisos = [t for bit, t in ((F_LIMITE, 'TOPE'), (F_SATURACION, 'SATURA'),
                                   (F_FALLO, 'FALLO: potencia cortada')) if f & bit]
        self.avisos.configure(text='  ·  '.join(avisos),
                              bg=(ROJO if fallo else NARANJA) if avisos else FONDO)

        self._activar(self.b_cero, hw and not on)
        self._activar(self.b_potencia, hw and cero)
        self.b_potencia.configure(text='Cortar potencia' if on else 'Dar potencia')
        self._activar(self.b_parar, hw)
        for b in self.b_prueba:
            self._activar(b, hw and not on)
        for w in (self.e_orden, self.b_orden):
            self._activar(w, hw)
        self.pista.configure(text=self._pista(gz, hw, on, cero, fallo))

    def _pista(self, gz: bool, hw: bool, on: bool, cero: bool, fallo: bool) -> str:
        if not hw:
            return ('Sin ESP32: no llega telemetria. Conecta la placa (la aduana la busca '
                    'sola) o lanza con hw:=false para no esperarla.')
        if fallo and not on:
            return ('La placa corto la potencia por un fallo; el motivo esta en el registro. '
                    'Corrigelo antes de volver a dar potencia.')
        if not cero:
            return ('Pon el brazo a mano en su reposo (hombro y codo rectos, husillo abajo) y '
                    'pulsa "Fijar cero". Sin cero, cc no sabe donde esta la punta.')
        if not on:
            return ('Pulsa "Dar potencia": la consigna se pondra donde esta el brazo real y la '
                    'placa cerrara el lazo ahi mismo. Los botones de "Probar motor" son un '
                    'empujon corto sin control, solo con la potencia cortada.')
        return ('La placa esta siguiendo la consigna. Esc corta la potencia (el brazo queda '
                'frenado).')

    @staticmethod
    def _activar(widget, si: bool):
        widget.state(['!disabled'] if si else ['disabled'])

    def _al_registro(self, texto: str):
        r = self.registro
        r.configure(state='normal')
        r.insert('end', texto.strip() + '\n')
        if int(r.index('end-1c').split('.')[0]) > 400:
            r.delete('1.0', '100.0')
        r.see('end')
        r.configure(state='disabled')


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
    node = BrazoHwNode()
    hilo = threading.Thread(target=_girar, args=(node,), daemon=True)
    hilo.start()
    try:
        ventana = VentanaBrazo(node)
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
