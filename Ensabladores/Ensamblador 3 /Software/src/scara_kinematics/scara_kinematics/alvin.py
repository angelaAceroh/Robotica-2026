"""Nodo 2: Alvin, el hombro.

    ros2 run scara_kinematics alvin --ros-args --params-file .../scara_urdf3.yaml

Alvin mueve 'juntura1', el giro del hombro: su posicion va en radianes y se
ensena en grados. Encoder de dos canales. Ojo con el: se sospecha que un cable de
su motor toca masa, y frenarlo por arriba llego a bloquear a los demas.

Escucha SOLO su pieza de cada muestra, scara/alvin/aduana, y con ella:

  1. comprueba que es suya y que viene entera: su junta, un valor, finito;
  2. comprueba que va en orden por el numero de muestra de la etiqueta
     (muestra=k en el frame_id): si falta alguna la cuenta, y si llega repetida
     la tira, porque cc la juntaria dos veces;
  3. se la pasa a cc por scara/alvin/junta sin tocar un solo numero, con el
     mismo sello y la misma etiqueta: cc junta por la etiqueta las tres piezas
     de la MISMA muestra;
  4. la saca en grados por scara/alvin/valor, para mirarla sin traducir;
  5. avisa si esta en su tope de recorrido (scara/alvin/tope);
  6. y vigila el encoder: si la placa le esta dando PWM y la posicion no se
     mueve, lo canta en scara/alvin/atascado.

Lo ultimo no es un adorno. En este brazo ha pasado varias veces que un motor
recibia PWM y no contaba -12 V desenchufados, un canal de encoder muerto, el
freno por arriba bloqueando el puente-, y desde fuera se veia como "el brazo no
va" sin saber cual de los tres fallaba. Con un nodo por motor, el que calla es
el que tiene el problema.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import POR_CLAVE, leer_etiqueta

# Junta, pines y prueba de motor: la tabla de protocolo.py, que es la misma que
# leen la aduana, cc y la ventana. Asi un cambio de cableado se hace en un sitio.
MOTOR = POR_CLAVE['alvin']


def fmt(q: float) -> str:
    """La posicion de Alvin para leerla: grados con signo."""
    return f'{math.degrees(q):+.1f}°'


class Alvin(Node):
    """El hombro: recibe su pieza de cada muestra, la comprueba y la cuida."""

    def __init__(self):
        super().__init__('alvin')
        self.model = declare_model_params(self)

        juntas = list(self.model.joint_names)
        if MOTOR.junta not in juntas:
            raise RuntimeError(
                f'alvin mueve "{MOTOR.junta}", que no esta en joint_names {juntas}: '
                'revisa el --params-file')
        self.i = juntas.index(MOTOR.junta)
        self.lo, self.hi = self.model.limits()[self.i]

        # Margen para dar la junta por "en el tope": 0.5 grados.
        self.margen = float(self.declare_parameter('margen_tope', 0.0087).value)
        # Por debajo de este PWM no se le puede pedir al motor que se mueva.
        self.u_minimo = float(self.declare_parameter('u_minimo', 0.15).value)
        # Cuanto aguantamos empujando sin ver movimiento antes de cantarlo.
        self.t_mudo = float(self.declare_parameter('t_mudo', 1.0).value)
        # Movimiento por debajo del cual consideramos que no se ha movido: 0.2 grados.
        self.eps = float(self.declare_parameter('eps', 0.0035).value)

        self.junta_pub = self.create_publisher(JointState, 'scara/alvin/junta', 10)
        self.valor_pub = self.create_publisher(Float64, 'scara/alvin/valor', 10)
        self.tope_pub = self.create_publisher(Bool, 'scara/alvin/tope', 10)
        # 'atascado' solo se publica cuando cambia, asi que va enganchado: quien
        # se suscriba despues -la ventana, un echo desde la terminal- tiene que
        # ver el estado actual y no quedarse esperando a la siguiente transicion.
        self.atasco_pub = self.create_publisher(
            Bool, 'scara/alvin/atascado',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState, 'scara/alvin/aduana', self.on_aduana, 10)

        # El orden de las muestras.
        self.muestra: int | None = None       # numero de la ultima que paso
        self.perdidas = 0                     # huecos entre la aduana y yo
        self.repetidas = 0

        # El vigilante del encoder.
        self.q_quieto: float | None = None    # posicion desde la que no se mueve
        self.t_empuje: float | None = None    # desde cuando se le esta dando PWM
        self.atascado = False
        self.atasco_pub.publish(Bool(data=False))

        self.get_logger().info(
            f'{MOTOR.nombre} ({MOTOR.que}) | junta {MOTOR.junta} [{self.i}] | '
            f'recorrido {fmt(self.lo)} .. {fmt(self.hi)} | '
            f'IN {MOTOR.pines[0]}/{MOTOR.pines[1]} ENA {MOTOR.pines[2]} '
            f'enc {MOTOR.encoder()}')

    # -------------------------------------------------------------- callback
    def on_aduana(self, msg: JointState):
        # 1. Que sea suya y venga entera.
        if list(msg.name) != [MOTOR.junta] or len(msg.position) != 1:
            self.get_logger().warning(
                f'me llega una pieza que no es mia o no viene entera: name '
                f'{list(msg.name)}, {len(msg.position)} posicion(es). La tiro',
                throttle_duration_sec=5.0)
            return
        q = msg.position[0]
        u = msg.effort[0] if msg.effort else 0.0
        if not (math.isfinite(q) and math.isfinite(u)):
            self.get_logger().warning(f'posicion o PWM no finitos (q={q}, u={u}): la tiro',
                                      throttle_duration_sec=5.0)
            return

        # 2. Que vaya en orden.
        if not self._en_orden(msg):
            return

        # 3. A cc, tal cual: el mismo mensaje, con su sello y su etiqueta.
        self.junta_pub.publish(msg)

        # 4. En grados.
        self.valor_pub.publish(Float64(data=math.degrees(q)))

        # 5. El tope.
        en_tope = q <= self.lo + self.margen or q >= self.hi - self.margen
        self.tope_pub.publish(Bool(data=en_tope))
        if en_tope:
            self.get_logger().warning(f'{MOTOR.nombre} en el tope ({fmt(q)})',
                                      throttle_duration_sec=5.0)

        # 6. El encoder.
        self._vigilar_encoder(q, u)

    def _en_orden(self, msg: JointState) -> bool:
        """Mira el numero de muestra. False = esta pieza no debe seguir."""
        marca = leer_etiqueta(msg.header.frame_id)
        if marca is None:
            # Sin etiqueta (la sonda del auditor la manda asi): no hay numero
            # que mirar. cc la juntara por el sello.
            return True
        k, ant = marca[0], self.muestra
        if ant is not None and k == ant:
            self.repetidas += 1
            self.get_logger().warning(
                f'la muestra {k} me ha llegado dos veces ({self.repetidas} en total): '
                'tiro la copia para que cc no la junte dos veces',
                throttle_duration_sec=5.0)
            return False
        if ant is not None and k < ant:
            # Solo pasa si la aduana ha vuelto a arrancar y cuenta desde 1.
            self.get_logger().info(f'la aduana ha vuelto a contar desde {k}')
        elif ant is not None and k > ant + 1:
            self.perdidas += k - ant - 1
            self.get_logger().warning(
                f'me faltan {k - ant - 1} muestra(s) entre la {ant} y la {k} '
                f'({self.perdidas} en total): se perdieron entre la aduana y yo',
                throttle_duration_sec=5.0)
        self.muestra = k
        return True

    def _vigilar_encoder(self, q: float, u: float):
        """Empujando y sin moverse el tiempo suficiente = algo va mal ahi."""
        ahora = self.get_clock().now().nanoseconds * 1e-9

        if abs(u) < self.u_minimo:
            # Sin empuje no se puede concluir nada: se reinicia la cuenta.
            self.q_quieto, self.t_empuje = None, None
            self._atasco(False)
            return

        if self.q_quieto is None or abs(q - self.q_quieto) > self.eps:
            self.q_quieto, self.t_empuje = q, ahora
            self._atasco(False)
            return

        if not self.atascado and ahora - self.t_empuje > self.t_mudo:
            self._atasco(True)
            self.get_logger().error(
                f'{MOTOR.nombre} no se mueve con {u * 100:+.0f} % de PWM durante '
                f'{self.t_mudo:.1f} s. Mira, por este orden: los 12 V del L298N (el '
                f'LED de la placa se enciende con los 5 V del USB, asi que no '
                f'demuestra nada), el encoder en {MOTOR.encoder()}, que el eje no '
                f'este topado, y el cable de su motor, que se sospecha que toca masa.')

    def _atasco(self, atascado: bool):
        if atascado != self.atascado:
            self.atascado = atascado
            self.atasco_pub.publish(Bool(data=atascado))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Alvin()
    except RuntimeError as exc:
        print(f'alvin: {exc}')
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C rclpy cierra el contexto aunque haya un callback a medias,
        # y ese callback lanza RCLError al publicar: no es un fallo, es el final.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
