"""Nodo 5: la aduana. Unico punto de paso entre la ESP32 y el resto del grafo.

    ros2 run scara_kinematics esp --ros-args \
        --params-file .../config/scara_urdf3.yaml -p port:=/dev/ttyUSB0

Todo lo que entra o sale de la placa pasa por aqui, de ahi el nombre. Abre el
puerto, lo reconecta solo, y lo que llega no lo publica de golpe: lo reparte
junta a junta, un topic por motor, para que Alvin, Simon y Teodoro solo tengan
que mirar lo suyo.

    ESP32 --USB--> ESTA ADUANA --+--> scara/alvin/aduana
                                 +--> scara/simon/aduana
                                 +--> scara/teodoro/aduana

Cada linea de telemetria es UNA muestra: el vector [q1 q2 d3] medido en el mismo
instante. Antes de repartirla, la aduana:

  - comprueba el checksum que la firma, y si no cuadra la tira entera;
  - le pone numero (muestra=1, 2, 3...) y apunta el reloj de la placa
    (placa_ms) y los ceros que lleva fijados (cero=N): las tres piezas salen
    con la misma etiqueta y el mismo sello, para que cc pueda volver a juntar
    las de la misma muestra;
  - saca la velocidad con el reloj de la placa, no con la hora de llegada: el
    USB entrega a rafagas, y dos lineas pegadas darian un dt de casi cero. Y
    no la saca a traves de un cero: ahi la lectura salta a 0 sin moverse nada;
  - y cuenta las muestras que no llegaron (huecos en el reloj de la placa).

Que sea la unica que abre el puerto no es un capricho de diseno: dos procesos no
pueden tener /dev/ttyUSB0 abierto a la vez. Por eso esp_rec no escribe en la
placa por su cuenta, sino que le pasa la consigna a esta aduana por scara/hw/cmd.

Entradas:
    scara/hw/cmd   sensor_msgs/JointState   consigna de posicion -> orden "T"
    scara/hw/raw   std_msgs/String          una orden del protocolo, tal cual

Salidas:
    scara/<motor>/aduana  sensor_msgs/JointState  su pieza de la muestra, etiquetada
    scara/hw/ok           std_msgs/Bool           enlace vivo + con cero + habilitado
    scara/hw/flags        std_msgs/UInt8          palabra de estado (0 sin enlace)
    scara/hw/log          std_msgs/String         lo que dice la placa ("!" y "#")

Servicios:
    scara/hw/enable  std_srvs/SetBool   da o quita potencia a los motores
    scara/hw/zero    std_srvs/Trigger   fija la pose actual como cero
"""
from __future__ import annotations

import math
import re
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import SetBool, Trigger

from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import (F_CERO, F_FALLO, F_HABILITADO, F_LIMITE,
                                        F_SATURACION, POR_JUNTA, Telemetria,
                                        buscar_puerto, etiqueta, leer_telemetria, linea)

# Lo que contesta la placa a una "F": asi la aduana sabe cada cuanto tiene que
# llegar una muestra aunque la frecuencia la haya cambiado alguien a mano.
_F_CONFIRMADA = re.compile(r'^!\s*telemetria\s+([0-9.]+)\s*Hz')
# Lo que contesta a una "Z". Llega por el mismo puerto y en orden con la
# telemetria: toda muestra que venga detras ya se midio con el cero nuevo.
_CERO_CONFIRMADO = re.compile(r'^!\s*cero establecido')


class Aduana(Node):

    def __init__(self):
        super().__init__('esp')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)
        n = self.model.ndof

        self.port = self.declare_parameter('port', '').value
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.rate = float(self.declare_parameter('rate', 50.0).value)
        self.telemetria_hz = float(self.declare_parameter('telemetria_hz', 25.0).value)

        # Techo de velocidad del hardware. El max_vel del YAML esta pensado para
        # la simulacion, donde el husillo puede ir a 0.10 m/s; con un JGA25-370
        # de 60 RPM sobre un husillo de 8 mm el maximo real son 0.008 m/s. Si no
        # se corrige, la consigna se adelanta al brazo y lo que se dibuja miente.
        self.max_vel_hw = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:n]

        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()
        self.parar = threading.Event()

        self.q_cmd: list[float] | None = None
        self.flags = 0
        self.t_ultimo_estado = 0.0

        # La muestra anterior, para la velocidad y para ver si falta alguna.
        self.muestra = 0                      # numero de la ultima que salio
        self.ant: Telemetria | None = None
        self.sello_ant = 0                    # ns; el sello siempre crece
        self.hz_placa = self.telemetria_hz    # lo que la placa dice que manda
        self.ceros = 0                        # ceros que ha confirmado la placa
        self.ceros_ant = 0                    # los que habia en la muestra anterior

        # Cuentas del enlace, para decir que se esta perdiendo y por que.
        self.tiradas: dict[str, int] = {}     # lineas rotas, por motivo
        self.huecos = 0                       # muestras que la placa mando y no llegaron

        # Un topic por motor: esto es el reparto que da nombre al nodo. Se indexa
        # por la posicion de la junta en joint_names, no por el orden de MOTORES,
        # para que el 'manita' del YAML de 4 GDL -que no tiene motor- no descoloque
        # a los otros tres.
        self.reparto = [(i, POR_JUNTA[j].clave,
                         self.create_publisher(JointState, f'scara/{POR_JUNTA[j].clave}/aduana', 10))
                        for i, j in enumerate(self.juntas) if j in POR_JUNTA]

        self.ok_pub = self.create_publisher(Bool, 'scara/hw/ok', 10)
        self.flags_pub = self.create_publisher(UInt8, 'scara/hw/flags', 10)
        self.log_pub = self.create_publisher(String, 'scara/hw/log', 50)
        self.create_subscription(JointState, 'scara/hw/cmd', self.on_cmd, 10)
        self.create_subscription(String, 'scara/hw/raw', self.on_raw, 10)
        self.create_service(SetBool, 'scara/hw/enable', self.on_enable)
        self.create_service(Trigger, 'scara/hw/zero', self.on_zero)

        self.create_timer(1.0 / self.rate, self.enviar_consigna)
        self.create_timer(0.25, self.publicar_ok)

        self.hilo = threading.Thread(target=self.bucle_serie, daemon=True)
        self.hilo.start()

        detalle = ', '.join(f'{c} <- {self.juntas[i]}' for i, c, _ in self.reparto)
        self.get_logger().info(f'aduana abierta | reparto: {detalle}')

    # ------------------------------------------------------------------ serie
    def conectar(self) -> bool:
        puerto = self.port or buscar_puerto()
        if not puerto:
            return False
        try:
            # dtr/rts desactivados ANTES de abrir, para intentar no disparar el
            # circuito de auto-reset de la placa. Es un intento, no una garantia:
            # MEDIDO con el CH340 de esta placa, el driver levanta las dos lineas
            # al abrir el puerto igualmente y la ESP32 se reinicia, perdiendo el
            # cero. O sea que cada reconexion obliga a referenciar otra vez.
            s = serial.Serial()
            s.port = puerto
            s.baudrate = self.baud
            s.timeout = 0.2
            try:
                s.dtr = False
                s.rts = False
            except (OSError, ValueError):
                pass          # hay conversores que no dejan tocar esas lineas
            s.open()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'no se pudo abrir {puerto}: {exc}',
                                      throttle_duration_sec=10.0)
            return False

        with self.lock:
            self.ser = s
        # Lo que no llego mientras el puerto estaba cerrado no es una muestra
        # perdida: es un corte, y ya se ha dicho. La velocidad empieza de nuevo.
        self.ant = None
        time.sleep(0.3)
        s.reset_input_buffer()

        self.get_logger().info(f'conectado a {puerto} a {self.baud} baudios')
        # La placa trae sus limites compilados; aqui se le imponen los del YAML,
        # que es donde vive la definicion del robot.
        for i, _ in enumerate(self.juntas):
            lo, hi = self.model.limits()[i]
            self.escribir(f'R {i} {lo:.5f} {hi:.5f} {self.max_vel_hw[i]:.5f}')
        self.escribir(f'F {self.telemetria_hz:.1f}')
        self.escribir('?')
        return True

    def desconectar(self):
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
                self.ser = None
        self.flags = 0

    def escribir(self, carga: str) -> bool:
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(linea(carga))
                return True
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'escritura fallida: {exc}')
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
                self.ser = None
                return False

    def bucle_serie(self):
        """Hilo de lectura: reconecta solo y reparte cada linea segun su prefijo."""
        while not self.parar.is_set():
            if self.ser is None:
                if not self.conectar():
                    self.parar.wait(2.0)
                continue
            try:
                cruda = self.ser.readline()
            except (serial.SerialException, OSError, TypeError):
                if self.parar.is_set():
                    return        # el puerto lo ha cerrado destroy_node, no la placa
                self.get_logger().warning('enlace perdido, reintentando')
                self.desconectar()
                continue
            if not cruda:
                continue
            texto = cruda.decode('utf-8', errors='replace').strip()
            if not texto:
                continue
            try:
                if texto[0] == '>':
                    self.on_telemetria(texto)
                elif texto[0] == '!':
                    self.get_logger().info(f'esp32: {texto[1:].strip()}')
                    self.log_pub.publish(String(data=texto))
                    m = _F_CONFIRMADA.match(texto)
                    if m:
                        self.hz_placa = float(m.group(1))
                    if _CERO_CONFIRMADO.match(texto):
                        self.ceros += 1
                elif texto[0] == '#':
                    self.get_logger().debug(f'esp32: {texto[1:].strip()}')
                    self.log_pub.publish(String(data=texto))
            except Exception:
                # Con Ctrl+C rclpy se apaga antes que este hilo, y publicar con el
                # contexto ya cerrado lanza RCLError: no es un fallo, es el final.
                if not rclpy.ok():
                    return
                raise

    def on_telemetria(self, texto: str):
        t = leer_telemetria(texto, self.model.ndof)
        if isinstance(t, str):
            self.tirar(t, texto)
            return

        ahora = time.monotonic()
        self.flags, self.t_ultimo_estado = t.flags, ahora
        ant = self.ant

        if ant is not None and t.ms == ant.ms:
            # La misma medida dos veces: repartirla otra vez seria mandarle a cc
            # dos muestras distintas con los mismos numeros.
            self.tirar('repetida', texto)
            return
        if ant is not None and t.ms < ant.ms:
            # El reloj de la placa solo va hacia atras si la placa ha vuelto a
            # arrancar, y al arrancar pierde el cero y la potencia.
            self.get_logger().warning(
                f'la placa se ha reiniciado (su reloj ha vuelto de {ant.ms} a {t.ms} '
                'ms): ha perdido el cero, hay que volver a fijarlo')
            ant = None

        # Velocidad por diferencias, con el reloj de la PLACA. El firmware no la
        # manda, y la hora de llegada no vale para esto: el USB entrega a
        # rafagas y dos lineas pegadas darian un dt de casi cero.
        #
        # Tras un cero la lectura salta a 0 sin que el brazo se mueva: esa
        # diferencia no es velocidad, asi que la primera muestra con el cero
        # nuevo sale con 0. Los huecos se siguen contando: el reloj no cambia.
        n = self.model.ndof
        v = [0.0] * n
        if ant is not None:
            dt_ms = t.ms - ant.ms
            if self.ceros == self.ceros_ant:
                v = [(a - b) * 1000.0 / dt_ms for a, b in zip(t.q, ant.q)]
            periodo_ms = 1000.0 / max(self.hz_placa, 0.1)
            if dt_ms > 1.5 * periodo_ms:
                self.huecos += max(1, round(dt_ms / periodo_ms) - 1)
                self.get_logger().debug(f'hueco en el reloj de la placa: {ant.ms} -> {t.ms} ms')
                self.avisar_enlace()
        self.ant = t
        self.ceros_ant = self.ceros

        # El sello siempre crece, aunque el reloj no lo haga (con use_sim_time
        # vale 0 hasta que Gazebo arranca). Si el reloj salta hacia atras mas de
        # un segundo es que Gazebo se ha reiniciado, y se le sigue.
        ahora_ns = self.get_clock().now().nanoseconds
        if ahora_ns <= self.sello_ant and ahora_ns > self.sello_ant - 1_000_000_000:
            ahora_ns = self.sello_ant + 1
        self.sello_ant = ahora_ns
        stamp = Time(nanoseconds=ahora_ns,
                     clock_type=self.get_clock().clock_type).to_msg()

        self.muestra += 1
        marca = etiqueta(self.muestra, t.ms, self.ceros)
        for i, _clave, pub in self.reparto:
            js = JointState()
            js.header.stamp = stamp
            js.header.frame_id = marca
            js.name = [self.juntas[i]]
            js.position = [t.q[i]]
            js.velocity = [v[i]]
            # "effort" es en realidad el ciclo de trabajo con signo (-1..1). No son
            # newtons, pero en un motor de continua es lo mas parecido al par que
            # hay sin medir corriente.
            js.effort = [t.u[i]]
            pub.publish(js)

    def tirar(self, motivo: str, texto: str):
        """Una linea que no entra en la cadena: se cuenta y se dice por que."""
        self.tiradas[motivo] = self.tiradas.get(motivo, 0) + 1
        self.get_logger().debug(f'telemetria tirada ({motivo}): {texto!r}')
        self.avisar_enlace()

    def avisar_enlace(self):
        tiradas = ', '.join(f'{v} por {k}' for k, v in self.tiradas.items())
        # Una linea rota tambien deja un hueco en el reloj de la placa; lo que
        # sobra de huecos son muestras que ni siquiera llegaron al puerto. Las
        # repetidas no dejan hueco: su original si entro.
        rotas = sum(v for k, v in self.tiradas.items() if k != 'repetida')
        self.get_logger().warning(
            f'enlace con la placa: {sum(self.tiradas.values())} linea(s) tirada(s)'
            + (f' ({tiradas})' if tiradas else '')
            + f' y {max(0, self.huecos - rotas)} muestra(s) que nunca llegaron. '
            'Si crece deprisa, mira el cable USB y no subas los baudios',
            throttle_duration_sec=10.0)

    # -------------------------------------------------------------- callbacks
    def on_cmd(self, msg: JointState):
        try:
            q_cmd = [msg.position[msg.name.index(nombre)] for nombre in self.juntas]
        except (ValueError, IndexError):
            self.get_logger().warning(
                f'scara/hw/cmd no trae las juntas {self.juntas}',
                throttle_duration_sec=5.0)
            return
        if not all(math.isfinite(v) for v in q_cmd):
            # Un nan llegaria a la placa como "nan", el firmware lo leeria tal
            # cual y el PID perseguiria un numero que no existe.
            self.get_logger().error(
                f'scara/hw/cmd trae un valor no finito {q_cmd}: no se lo mando a la '
                'placa', throttle_duration_sec=5.0)
            return
        self.q_cmd = q_cmd

    def enviar_consigna(self):
        if self.q_cmd is None:
            return
        self.escribir('T ' + ' '.join(f'{v:.5f}' for v in self.q_cmd))

    def on_raw(self, msg: String):
        # Lo mismo que se teclearia en el monitor serie (K, P, D, la prueba A...),
        # pero desde ROS. Es tambien la via por la que esp_rec llega a la placa.
        texto = msg.data.strip()
        if texto:
            self.escribir(texto)

    def publicar_ok(self):
        vivo = (time.monotonic() - self.t_ultimo_estado) < 1.0
        listo = vivo and bool(self.flags & F_HABILITADO) and bool(self.flags & F_CERO)
        self.ok_pub.publish(Bool(data=listo))
        self.flags_pub.publish(UInt8(data=self.flags if vivo else 0))
        if vivo and self.flags & F_LIMITE:
            self.get_logger().warning('alguna junta esta en su tope de recorrido',
                                      throttle_duration_sec=5.0)
        if vivo and self.flags & F_SATURACION:
            self.get_logger().warning('el PID esta saturando: baja la velocidad '
                                      'o revisa la sintonia',
                                      throttle_duration_sec=5.0)
        if vivo and self.flags & F_FALLO:
            # La placa ya ha cortado la potencia por su cuenta; esto es para que
            # se vea desde ROS y no solo en el log de la ESP32.
            self.get_logger().error(
                'la placa corto la potencia por bloqueo de una junta: revisa el '
                'encoder, el eje y el signo del lazo antes de rehabilitar',
                throttle_duration_sec=5.0)

    def on_enable(self, request, response):
        if self.ser is None:
            response.success, response.message = False, 'sin enlace con la ESP32'
            return response
        if request.data and not (self.flags & F_CERO):
            # El firmware tambien lo rechaza, pero conviene explicar aqui por que:
            # con encoders incrementales, habilitar sin cero significa mover el
            # brazo hacia una posicion que la placa cree conocer y no conoce.
            response.success = False
            response.message = ('sin cero: coloca el brazo en la pose de '
                                'referencia y llama a scara/hw/zero primero')
            return response
        ok = self.escribir(f'E {1 if request.data else 0}')
        response.success = ok
        response.message = ('potencia ' + ('habilitada' if request.data else 'cortada')
                            if ok else 'no se pudo escribir en el puerto')
        return response

    def on_zero(self, request, response):
        ok = self.escribir('Z')
        response.success = ok
        response.message = ('cero fijado en la pose actual' if ok
                            else 'sin enlace con la ESP32')
        return response

    # ------------------------------------------------------------------ cierre
    def destroy_node(self):
        self.parar.set()
        # Cortar la potencia al salir: si el nodo muere y los motores se quedan
        # dando corriente, nadie va a estar mirando el brazo.
        self.escribir('E 0')
        self.desconectar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Aduana()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C rclpy cierra el contexto aunque el ejecutor este a medias,
        # y entonces lanza RCLError al preparar la siguiente espera: no es un
        # fallo, es el final. El finally corta la potencia igual.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
