"""Nodo `puente_placa`: el control cinematico con el brazo de verdad.

    ros2 launch scara_kinematics control_placa.launch.py                 # lo normal
    ros2 launch scara_kinematics control_placa.launch.py simular:=true   # sin el brazo

Hace con la ESP32 lo que hacia Gazebo en control.launch.py: recibe las
velocidades de junta que manda control_nodo y devuelve lo que miden los
encoders. control_nodo no se ha tocado; solo se le cambia a donde manda.

    control_nodo --Float64--> /scara_urdf/placa/cmd_vel/<junta>   (rad/s, m/s)
       [puente_placa]  junta -> motor (grados/s, mm/s) -> 'V va vs vt *HH' --USB-->
       main_control.py  (en la placa: integra la velocidad y la sigue con un PI)
       main_control.py --> 'P ms pa ps pt EEE *HH' (50 por segundo) --USB-->
       [puente_placa]  motor -> junta, con la hora a la que MIDIO la placa
    /joint_states --> control_nodo (y robot_state_publisher, y la ventana)

De junta a motor, con config/micropython.yaml (alcance.Calibracion, la misma
cuenta que `vector` y la ventana `efector`, con los numeros de main_control.py:
Alvin y Simon en grados de su junta, Teodoro en mm). Las velocidades, con
placa_vel.vel_a_motor.

EL HUSILLO. Con husillo:=virtual (o sin `paso_husillo_mm`), Teodoro no se activa
y la d3 que se publica es la que se integra con la velocidad que pide
control_nodo: la z del control es de mentira (sale perfecta) y el plano, de
verdad. Se avisa al arrancar. Sirve para probar el plano antes de fiarse de la z.

AL ARRANCAR (activar:=true): en cuanto la placa contesta, compara sus limites con
los del YAML, le dice 'parar' a control_nodo (que regule donde esta el brazo) y
medio segundo despues manda 'a': cada motor se engancha donde esta, sin moverse.
Si la placa suelta los motores (un vigilante, 'q'), tambien le dice 'parar' a
control_nodo, y no se vuelven a activar solos: hace falta la orden 'activar'.

Ordenes (std_msgs/String en scara/placa/orden):
    activar                 engancha los motores donde estan
    soltar                  'q' a la placa: PWM 0 y las posiciones a la flash
    ? | l | g ... | h <motor> | fijar <motor> <pos>
                            pasan tal cual a la placa (h y fijar, con los motores sueltos)

Salidas:
    /joint_states             sensor_msgs/JointState   lo que mide la placa, en junta
    scara/placa/estado        std_msgs/String          5 Hz: ACTIVA/SUELTA, enlace...
    scara/placa/motores       geometry_msgs/Vector3    Alvin y Simon (grados), Teodoro (mm)
    scara/placa/seguimiento   geometry_msgs/Vector3    consigna - posicion, 5 Hz
    scara/placa/pwm           geometry_msgs/Vector3    el PWM de cada motor, 5 Hz
    scara/placa/linea         std_msgs/String          lo que dice la placa (menos P y D)
    scara/control/orden       std_msgs/String          'parar' a control_nodo (ver arriba)

Al cerrar (Ctrl+C) manda 'q': los motores quedan sueltos y la posicion, guardada.
No se lanza a la vez que `esp` ni `vector`: querrian el mismo puerto.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from scara_kinematics import alcance as A
from scara_kinematics import placa_vel as PV
from scara_kinematics import ruta as R
from scara_kinematics.model import declare_model_params

try:
    import serial
    from scara_kinematics.protocolo import buscar_puerto
except ImportError:          # sin pyserial no hay placa
    serial = None
    buscar_puerto = None

ESPERA_PARAR_S = 0.5     # entre decirle 'parar' a control_nodo y mandar 'a'


class PuentePlaca(Node):

    def __init__(self):
        super().__init__('puente_placa')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)
        self.juntas = list(self.model.joint_names)[:3]

        p = self.declare_parameter
        self.puerto = str(p('puerto', '').value)
        self.baudios = int(p('baudios', 115200).value)
        self.espera_arranque = float(p('espera_arranque', 4.0).value)
        self.cmd_prefix = str(p('cmd_vel_prefix', '/scara_urdf/placa/cmd_vel/').value)
        self.pedido_activar = bool(p('activar', True).value)
        # Sin velocidades de control_nodo en este tiempo, se mandan ceros: los
        # motores se quedan quietos, sujetando (la placa tiene su propio vigilante).
        self.vigilante = float(p('vigilante', 0.2).value)
        # Lo mismo que VEL_MAX en la tabla MOTORES de main_control.py (grados/s,
        # grados/s, mm/s): mas no lo integra la placa
        self.vel_max_motor = [float(v) for v in p('vel_max_motor', [15.0, 15.0, 10.0]).value]
        # De junta a motor: config/micropython.yaml (lo pasa el launch)
        d = A.Calibracion()
        defectos = {'limites_motor': [x for par in d.limites for x in par],
                    'reduccion': d.reduccion, 'signo': d.signo, 'cero': d.cero,
                    'paso_husillo_mm': d.paso_husillo_mm, 'velocidades': d.velocidades}
        self.cal = A.Calibracion.desde_parametros(
            lambda nombre, _: p(nombre, defectos[nombre]).value)
        husillo = str(p('husillo', 'auto').value).lower()
        self.husillo_real = self.cal.husillo_medido and husillo != 'virtual'
        if husillo == 'real' and not self.cal.husillo_medido:
            self.get_logger().error('husillo:=real sin paso_husillo_mm: va virtual')
        z0 = float(p('z_inicial', R.Z_SEGURO).value)
        lo, hi = self.robot.lim[2]
        self.d3_virtual = min(max((z0 - self.robot.z_ref) / self.robot.z_dir, lo), hi)

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.estado_pub = self.create_publisher(String, 'scara/placa/estado', 10)
        self.motores_pub = self.create_publisher(Vector3, 'scara/placa/motores', 10)
        self.seg_pub = self.create_publisher(Vector3, 'scara/placa/seguimiento', 10)
        self.pwm_pub = self.create_publisher(Vector3, 'scara/placa/pwm', 10)
        self.linea_pub = self.create_publisher(String, 'scara/placa/linea', 50)
        self.control_pub = self.create_publisher(String, 'scara/control/orden', 10)
        for i, j in enumerate(self.juntas):
            self.create_subscription(Float64, f'{self.cmd_prefix}{j}',
                                     lambda msg, i=i: self.on_vel(i, msg), 10)
        self.create_subscription(String, 'scara/placa/orden', self.on_orden, 10)

        self.ser = None
        self.t_abrir = 0.0
        self.buffer = ''
        self.abierto_en = None
        self.contesta = False            # ha llegado una P buena: es main_control.py
        self.avisado_main = False
        self.activo = [False, False, False]
        self.pos: list[float] | None = None          # lo que cuenta la placa
        self.reloj = PV.Reloj()
        self.vel = [0.0, 0.0, 0.0]                   # lo ultimo de control_nodo
        self.t_vel: list[float | None] = [None, None, None]
        self.t_enviado = time.monotonic()
        self.t_d3: float | None = None
        self.activar_en: float | None = None
        self.seguimiento: list[float] | None = None
        self.n_p, self.malas, self.hz = 0, 0, 0.0
        self.t_hz = time.monotonic()
        self.t_estado = 0.0

        self.get_logger().info(
            f'puente_placa | velocidades de {self.cmd_prefix}<junta> | limites '
            f'{self.cal.limites} | reduccion {self.cal.reduccion} | signo {self.cal.signo} '
            f'| cero {self.cal.cero} | ordenes por scara/placa/orden')
        if not self.husillo_real:
            por = 'husillo:=virtual' if self.cal.husillo_medido else 'falta paso_husillo_mm'
            self.get_logger().warning(
                f'HUSILLO VIRTUAL ({por}). '
                f'Teodoro no se activa y la z del control es de mentira, empezando en '
                f'{self.robot.z_ref + self.robot.z_dir * self.d3_virtual:.0f} mm. '
                f'El plano (Alvin y Simon) es de verdad')
        self.create_timer(0.005, self.tic)

    # ================================================================= entradas
    def on_vel(self, i: int, msg: Float64):
        self.vel[i] = msg.data
        self.t_vel[i] = time.monotonic()
        # control_nodo manda las tres juntas seguidas en cada tic: cuando han
        # llegado las tres, sale la V (asi la placa no recibe mezclas de dos tics)
        if all(t is not None and t > self.t_enviado for t in self.t_vel):
            self._enviar_v()

    def on_orden(self, msg: String):
        texto = msg.data.strip()
        if not texto:
            return
        cabeza = texto.split()[0].lower()
        if cabeza == 'activar':
            self.pedido_activar = True
            self.get_logger().info('activar: le digo parar a control_nodo y engancho '
                                   'los motores donde estan')
        elif cabeza in ('soltar', 'q'):
            self.pedido_activar = False
            self.activar_en = None
            self._mandar('q')
            self.get_logger().warning('soltar: q a la placa')
        elif cabeza in ('?', 'l', 'g', 'h', 'fijar'):
            self._mandar(texto)
        else:
            self.get_logger().error(f'no conozco «{texto}»: activar, soltar, ?, l, g, h, fijar')

    # =================================================================== placa
    def _abrir(self):
        if self.ser is not None or time.monotonic() - self.t_abrir < 0.5:
            return
        self.t_abrir = time.monotonic()
        if serial is None:
            self.get_logger().error('falta pyserial: sudo apt install python3-serial',
                                    throttle_duration_sec=30.0)
            return
        puerto = self.puerto or (buscar_puerto() if buscar_puerto else None)
        if not puerto:
            self.get_logger().warning('no encuentro la placa por USB',
                                      throttle_duration_sec=10.0)
            return
        try:
            s = serial.Serial()
            s.port = puerto
            s.baudrate = self.baudios
            s.timeout = 0
            try:
                s.dtr = False
                s.rts = False
            except (OSError, ValueError):
                pass
            s.open()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'no se pudo abrir {puerto}: {e}',
                                      throttle_duration_sec=10.0)
            return
        self.ser = s
        self.buffer = ''
        self.abierto_en = time.monotonic()
        self.contesta = False
        self.get_logger().info(f'{puerto} abierto. Con el CH340, abrir el puerto reinicia '
                               f'la ESP32: espero a que main_control.py hable')

    def _mandar(self, linea: str, eco: bool = True):
        if eco:
            self.get_logger().info(f'-> placa: {linea}')
        if self.ser is None:
            if eco:
                self.get_logger().error(f'«{linea}» no sale: la placa no esta conectada')
            return
        try:
            self.ser.write((linea + '\r\n').encode())
        except (serial.SerialException, OSError) as e:
            self._perdida(e)

    def _perdida(self, e):
        self.get_logger().error(f'se perdio la placa: {e}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.activo = [False, False, False]

    def _leer(self) -> list[str]:
        if self.ser is None:
            return []
        try:
            datos = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self._perdida(e)
            return []
        if not datos:
            return []
        self.buffer += datos.decode('utf-8', errors='replace')
        *lineas, self.buffer = self.buffer.split('\n')
        return [ln.strip() for ln in lineas if ln.strip()]

    def _enviar_v(self):
        ahora = time.monotonic()
        self.t_enviado = ahora
        if not any(self.activo) or self.ser is None:
            self.t_d3 = None
            return
        qd = [self.vel[0], self.vel[1], self.vel[2] * 1e3]          # rad/s, rad/s, mm/s
        if not self.husillo_real:
            # La z de mentira: se integra lo que pide control_nodo
            if self.t_d3 is not None:
                lo, hi = self.robot.lim[2]
                self.d3_virtual = min(max(
                    self.d3_virtual + qd[2] * min(ahora - self.t_d3, 0.1), lo), hi)
            self.t_d3 = ahora
        motor = PV.vel_a_motor(qd, self.cal)
        if not self.husillo_real:
            motor[2] = 0.0
        motor = [m if a else 0.0 for m, a in zip(motor, self.activo)]
        # La placa no integra mas de vel_max_motor: si alguno pide mas, los tres
        # por el mismo factor, para que la punta no se tuerza (como la Ley).
        f = max(abs(m) / v for m, v in zip(motor, self.vel_max_motor))
        if f > 1.0:
            motor = [m / f for m in motor]
            self.get_logger().warning(
                f'control_nodo pide {f:.1f} veces lo que da la placa (vel_max_motor '
                f'{self.vel_max_motor}): recorto los tres. Baja vel_max en el launch',
                throttle_duration_sec=5.0)
        self._mandar(PV.linea_v(motor), eco=False)

    # ================================================================= el reloj
    def tic(self):
        self._abrir()
        llegada = self.get_clock().now().nanoseconds * 1e-9
        for linea in self._leer():
            self._atender(linea, llegada)
        ahora = time.monotonic()

        if self.ser is not None and not self.contesta and not self.avisado_main \
                and ahora - self.abierto_en > self.espera_arranque + 2.0:
            self.avisado_main = True
            self.get_logger().error(
                'la placa no manda P: no parece main_control.py. Cargalo en Thonny como '
                'main.py (CONTROL_PLACA.md §2) y reinicia la placa')

        if any(self.activo) and ahora - self.t_enviado > self.vigilante:
            self._mandar(PV.linea_v([0.0, 0.0, 0.0]), eco=False)
            self.t_enviado = ahora
            self.t_d3 = None
            self.get_logger().warning(
                f'control_nodo no manda velocidades desde hace {self.vigilante:g} s: '
                f'quietos, sujetando', throttle_duration_sec=5.0)

        if (self.pedido_activar and self.contesta and self.pos is not None
                and not any(self.activo) and self.activar_en is None):
            # Primero que control_nodo regule donde esta el brazo AHORA (si no,
            # iria a por la ultima consigna que tenia), y luego enganchar.
            self.control_pub.publish(String(data='parar'))
            self.activar_en = ahora + ESPERA_PARAR_S
        if self.activar_en is not None and ahora >= self.activar_en:
            self.activar_en = None
            self.pedido_activar = False
            self._mandar('a' if self.husillo_real else 'a alvin simon')

        if ahora - self.t_hz >= 1.0:
            self.hz = self.n_p / (ahora - self.t_hz)
            self.n_p, self.t_hz = 0, ahora
        if ahora - self.t_estado >= 0.2:
            self.t_estado = ahora
            self.estado_pub.publish(String(data=self._estado()))

    def _estado(self) -> str:
        if self.ser is None:
            return 'SIN PLACA: no encuentro el puerto'
        if not self.contesta:
            return 'CONECTANDO: esperando a main_control.py'
        e = ''.join('A' if a else 'L' for a in self.activo)
        txt = (f'{"ACTIVA" if any(self.activo) else "SUELTA"} {e} | {self.hz:.0f} Hz | '
               f'husillo {"real" if self.husillo_real else "virtual"}')
        if self.seguimiento is not None and any(self.activo):
            txt += ' | seguimiento ' + ' '.join(
                f'{n} {v:+.2f}{u}' for n, v, u, a in zip(PV.MOTORES, self.seguimiento,
                                                         PV.UNIDADES, self.activo) if a)
        if self.malas:
            txt += f' | {self.malas} lineas estropeadas'
        return txt

    def _atender(self, linea: str, llegada: float):
        try:
            d = PV.leer(linea)
        except PV.ChecksumMalo:
            self.malas += 1
            return
        if isinstance(d, PV.Muestra):
            self._muestra(d, llegada)
            return
        if isinstance(d, PV.Diagnostico):
            self.seguimiento = d.seguimiento
            self.seg_pub.publish(Vector3(x=d.seguimiento[0], y=d.seguimiento[1],
                                         z=d.seguimiento[2]))
            self.pwm_pub.publish(Vector3(x=d.pwm[0], y=d.pwm[1], z=d.pwm[2]))
            return
        self.linea_pub.publish(String(data=linea))
        log = self.get_logger()
        if 'Sistema listo' in linea:
            log.error('la placa lleva main.py (el de `vector`), no main_control.py: '
                      'cargalo en Thonny como main.py (CONTROL_PLACA.md §2)')
        elif linea.startswith('CONTROL listo'):
            log.info(f'placa: {linea}')
        elif linea.startswith('ACTIVO'):
            log.info(f'placa: {linea}  (motores enganchados donde estan)')
        elif linea.startswith('RECHAZADO'):
            log.error(f'la placa no activa los motores: {linea[10:]}')
        elif linea.startswith('PARADO'):
            log.warning(f'placa: {linea}  (motores sueltos; para seguir, «activar»)')
            self.control_pub.publish(String(data='parar'))
        elif linea.startswith('ERROR V'):
            self.malas += 1                          # una V estropeada por el USB
        elif linea.startswith('ERROR'):
            log.error(f'placa: {linea[6:]}')
        elif linea.startswith('VIGILANTE'):
            log.warning(f'placa: {linea}')
        elif linea.startswith('LIMITES'):
            self._comprobar_limites(linea)
        else:
            log.info(f'placa: {linea}')

    def _muestra(self, d: PV.Muestra, llegada: float):
        t = self.reloj(d.ms, llegada)
        if not self.contesta:
            self.contesta = True
            self.get_logger().info('main_control.py contesta. Le pido sus limites (l)')
            self._mandar('l', eco=False)
        self.pos = d.pos
        self.activo = d.activo
        self.n_p += 1
        q = self.cal.a_junta(d.pos)
        d3 = q[2] if self.husillo_real and q[2] is not None else self.d3_virtual
        js = JointState()
        js.header.stamp = Time(nanoseconds=int(t * 1e9)).to_msg()
        js.name = list(self.juntas)
        js.position = [float(q[0]), float(q[1]), float(d3) * 1e-3]
        self.js_pub.publish(js)
        self.motores_pub.publish(Vector3(x=d.pos[0], y=d.pos[1], z=d.pos[2]))

    def _comprobar_limites(self, linea: str):
        placa = PV.leer_limites(linea)
        pc = [x for par in self.cal.limites for x in par]
        if placa is not None and all(abs(a - b) < 1e-6 for a, b in zip(placa, pc)):
            self.get_logger().info('limites: la placa y config/micropython.yaml coinciden')
        else:
            self.get_logger().warning(
                f'LIMITES DISTINTOS. PC (config/micropython.yaml): {pc}; placa '
                f'(main_control.py, tabla MOTORES): {placa}. La placa no deja salir de los '
                f'suyos y control_nodo de los del YAML: manda el mas estrecho')


def main(args=None):
    rclpy.init(args=args)
    node = PuentePlaca()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C rclpy cierra el contexto con el ejecutor a medias y lanza
        # RCLError. No es un fallo, es el final.
        if rclpy.ok():
            raise
    finally:
        if node.ser is not None:
            try:
                node.ser.write(b'q\r\n')     # motores sueltos y posicion guardada
                node.ser.flush()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
