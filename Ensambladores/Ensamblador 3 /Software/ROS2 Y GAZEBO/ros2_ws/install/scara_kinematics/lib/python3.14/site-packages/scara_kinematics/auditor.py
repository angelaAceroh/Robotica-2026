"""El auditor: certifica que los datos van y vuelven enteros por la cadena.

    ros2 run scara_kinematics auditor --ros-args \
        --params-file .../config/scara_urdf3.yaml

En el grafo de ROS 2 nadie acusa recibo de nada. Un nodo publica y se olvida, y
si el de al lado no lo recibe -o lo recibe cambiado, o dos veces, o medio
segundo tarde- no se entera ninguno de los dos. Este nodo es el que mira: no
toca los datos, los SIGUE, salto a salto, y dice si llegaron.

    [esp] --scara/<motor>/aduana--> [alvin|simon|teodoro] --scara/<motor>/junta-->
          --> [cc] --joint_states--> --scara/ee_pose-->
     ^                                                           ^
     |                       [auditor] escucha los cuatro         |
     +--- scara/hw/raw "?" ---> la placa ---> scara/hw/log -------+
                          la ida y la vuelta

Tres cosas distintas, porque "que los datos lleguen bien" son tres cosas:

  1. VIGILANCIA (siempre). Sigue cada muestra por los tres saltos de la cadena
     y cuenta lo que se pierde, lo que se altera, lo que se duplica y lo que
     tarda. Es pasivo: no publica nada en la cadena.

  2. IDA Y VUELTA (a peticion). Le manda un "?" a la placa y compara lo que
     contesta con lo que la aduana le habia mandado al conectar. Si los limites
     que devuelve son los del YAML, el camino de ida (PC -> puerto -> placa) y
     el de vuelta (placa -> puerto -> topic) estan probados con datos reales, y
     ademas NO MUEVE NADA.

  3. SONDA (a peticion, y solo sin placa). Mete por la cadena una rampa
     conocida y comprueba que sale identica por el otro extremo. Es la prueba
     de la cadena de nodos sola, sin hardware de por medio.

Salidas:
    scara/auditor/informe   std_msgs/String   la tabla, cada 1/hz_informe s
    scara/auditor/ok        std_msgs/Bool     enganchado: false si algo falla

Servicios:
    scara/auditor/comprobar  std_srvs/Trigger  ida y vuelta (o sonda si no hay
                                               placa); el veredicto va en la
                                               respuesta
"""
from __future__ import annotations

import math
import os
import re
import threading
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger

from scara_kinematics.model import declare_model_params
from scara_kinematics.protocolo import F_HABILITADO, POR_JUNTA, leer_cero

# Lineas del volcado de "?" que interesan:  "#    q=[-2.0944, +2.0944] vmax=1.5000"
_LIMITES = re.compile(r'q=\[\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\]\s*vmax=([-+0-9.eE]+)')


@dataclass
class Salto:
    """Lo que se sabe de un salto de la cadena despues de mirarlo un rato."""

    nombre: str
    entrados: int = 0        # mensajes que entraron por el lado de arriba
    salidos: int = 0         # los que salieron por el de abajo
    perdidos: int = 0        # entraron y no salieron nunca
    alterados: int = 0       # salieron con un valor distinto del que entro
    duplicados: int = 0      # salieron dos veces
    colados: int = 0         # salieron sin haber entrado
    desordenados: int = 0    # salieron antes que uno mas viejo
    descartados: int = 0     # del arranque: entraron antes de poder juzgar
    lat_suma: float = 0.0
    lat_max: float = 0.0
    nota: str = ''

    def latencia(self) -> tuple[float, float]:
        media = self.lat_suma / self.salidos if self.salidos else 0.0
        return media * 1000.0, self.lat_max * 1000.0

    def sano(self) -> bool:
        return not (self.perdidos or self.alterados or self.colados or self.desordenados)

    def fila(self) -> str:
        media, maximo = self.latencia()
        estado = 'OK ' if self.sano() else '!! '
        # Lo raro va en la nota y no en una columna mas: una tabla con una
        # marca de aviso y todas las columnas a cero no sirve de nada.
        raro = [f'{n} {t}' for n, t in ((self.duplicados, 'duplicados'),
                                        (self.colados, 'colados'),
                                        (self.desordenados, 'desordenados'),
                                        (self.descartados,
                                         'descartados en el arranque')) if n]
        nota = ', '.join(raro + ([self.nota] if self.nota else []))
        return (f'{estado}{self.nombre:<26} {self.entrados:>7} {self.salidos:>7} '
                f'{self.perdidos:>8} {self.alterados:>8} '
                f'{media:>7.1f} {maximo:>7.1f}  {nota}')


@dataclass
class Pendiente:
    """Un mensaje que entro en un salto y todavia no ha salido."""

    t: float
    valores: tuple


class Auditor(Node):

    def __init__(self):
        super().__init__('auditor')
        self.model = declare_model_params(self)
        self.juntas = list(self.model.joint_names)
        n = self.model.ndof

        self.hz_informe = float(self.declare_parameter('hz_informe', 0.5).value)
        # Cuanto se espera a que un mensaje salga antes de darlo por perdido. La
        # cadena entera tarda milisegundos; medio segundo es no tener dudas.
        self.t_perdido = float(self.declare_parameter('t_perdido', 0.5).value)
        # Tolerancia al comparar. Es CERO a proposito: entre nodos los valores
        # viajan tal cual, sin cuentas por medio, asi que un solo bit distinto
        # ya es un dato alterado y hay que verlo.
        self.tol = float(self.declare_parameter('tol', 0.0).value)
        # Para la cinematica: aqui si se rehace la cuenta, y en coma flotante
        # dos caminos distintos no dan el mismo bit.
        self.tol_fk = float(self.declare_parameter('tol_fk', 1e-9).value)
        self.max_vel_hw = [float(v) for v in self.declare_parameter(
            'max_vel_hw', [1.5, 1.5, 0.008, 2.0]).value][:n]
        # Margen sobre la velocidad maxima antes de cantar un salto imposible.
        # 3 veces es holgado: no busca medio milimetro de mas, busca la muestra
        # que se salto o la linea de telemetria que llego rota.
        self.margen_salto = float(self.declare_parameter('margen_salto', 3.0).value)
        # Cuanto se deja reposar un mensaje antes de juzgarlo. No es por
        # prudencia: es que el ejecutor no procesa los callbacks en el orden en
        # que llegaron los mensajes (ver evaluar_junta).
        self.t_gracia = float(self.declare_parameter('t_gracia', 0.1).value)
        self.registro = str(self.declare_parameter('registro', '').value)
        # La sonda inyecta datos en la cadena: apagada salvo que se pida.
        self.sonda_n = int(self.declare_parameter('sonda_n', 40).value)
        self.sonda_hz = float(self.declare_parameter('sonda_hz', 25.0).value)
        self.t_espera = float(self.declare_parameter('t_espera', 3.0).value)

        self.lock = threading.Lock()
        self.t_ini = time.monotonic()
        self.cola_junta: deque = deque()
        self.cola_js: deque = deque()

        # --- salto 1: la aduana reparte, cada motor republica. Se casa por
        # sello de tiempo, porque cada motor (alvin.py, simon.py, teodoro.py)
        # republica el mensaje TAL CUAL y el sello que puso la aduana sobrevive.
        self.motores = [(i, POR_JUNTA[j].clave) for i, j in enumerate(self.juntas)
                        if j in POR_JUNTA]
        self.saltos: dict[str, Salto] = {}
        self.pendientes: dict[str, dict[int, Pendiente]] = {}
        self.vistos: dict[str, deque] = {}
        # Un salto no se puede juzgar hasta haber visto pasar algo por el: al
        # arrancar, el auditor se suscribe a los dos extremos en instantes
        # distintos y lo que quede a medias no lo ha perdido nadie.
        self.arrancado: dict[str, bool] = {}
        for _i, clave in self.motores:
            nombre = f'aduana -> {clave}'
            self.saltos[clave] = Salto(nombre)
            self.pendientes[clave] = {}
            self.vistos[clave] = deque(maxlen=500)
            self.arrancado[clave] = False

        # --- salto 2: de los motores a cc. Tambien por sello: cc junta las tres
        # piezas de una muestra y saca UN joint_states con el sello de esa
        # muestra, asi que cada joint_states tiene que traer, exactos, los
        # valores que los tres motores publicaron con ese mismo sello.
        self.saltos['cc'] = Salto('motores -> cc')
        # sello -> {motor: (valor, hora de llegada)}. Se apunta al llegar, sin
        # esperar la gracia: cuando se juzgue el joint_states, sus piezas tienen
        # que estar ya aqui (el motor las publica antes de que cc pueda sacarlo).
        self.piezas: dict[int, dict[str, tuple[float, float]]] = {}
        self.js_vistos: deque = deque(maxlen=500)
        self.cc_arrancado = False
        self.cc_incompletas = 0      # piezas de muestras a las que les falto otra
        self.n_js = 0

        # --- salto 3: de las juntas a la punta. Se rehace la FK y se compara.
        self.saltos['fk'] = Salto('cc -> punta (ee_pose)')
        # Las dos mitades de un ciclo de cc: las juntas y la punta. Van por
        # topics distintos y el orden de entrega no esta garantizado, asi que
        # se guardan por sello y se comprueban cuando estan las dos.
        self.ciclos: dict[int, dict] = {}
        self.error_fk = 0.0

        # --- plausibilidad fisica: lo unico que delata desde fuera una linea de
        # telemetria rota, porque la aduana no comprueba el XOR que trae.
        self.saltos_imposibles = 0
        self.ultimo_q: dict[str, tuple] = {}

        self.flags = 0
        self.t_flags = 0.0
        self.log = deque(maxlen=200)
        self.log_ev = threading.Event()

        # Colas hondas (200) a proposito. Los nodos de la cadena van con 10,
        # que es lo suyo para datos que caducan -mas vale la medida nueva que la
        # vieja-, pero el auditor no puede permitirse tirar ninguna: si pierde
        # un mensaje se lo apunta al de al lado, y entonces el informe acusa a
        # quien no es.
        grupo = MutuallyExclusiveCallbackGroup()
        for i, clave in self.motores:
            self.create_subscription(
                JointState, f'scara/{clave}/aduana',
                lambda msg, c=clave: self.on_aduana(c, msg), 200, callback_group=grupo)
            self.create_subscription(
                JointState, f'scara/{clave}/junta',
                lambda msg, c=clave: self.on_junta(c, msg), 200, callback_group=grupo)
        self.create_subscription(JointState, 'joint_states', self.on_js, 200,
                                 callback_group=grupo)
        self.create_subscription(PoseStamped, 'scara/ee_pose', self.on_pose, 200,
                                 callback_group=grupo)
        self.create_subscription(String, 'scara/hw/log', self.on_log, 50,
                                 callback_group=grupo)
        self.create_subscription(UInt8, 'scara/hw/flags', self.on_flags, 10,
                                 callback_group=grupo)

        self.informe_pub = self.create_publisher(String, 'scara/auditor/informe', 10)
        self.ok_pub = self.create_publisher(
            Bool, 'scara/auditor/ok',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.raw_pub = self.create_publisher(String, 'scara/hw/raw', 10)
        self.aduana_pub = {clave: self.create_publisher(
            JointState, f'scara/{clave}/aduana', 50) for _i, clave in self.motores}

        # La comprobacion espera respuestas que llegan por otros callbacks: si
        # compartiera grupo con ellos, el ejecutor se quedaria esperandose a si
        # mismo. Por eso va en un grupo reentrante y el nodo gira con el
        # ejecutor multihilo.
        self.create_service(Trigger, 'scara/auditor/comprobar', self.on_comprobar,
                            callback_group=ReentrantCallbackGroup())

        self.create_timer(0.2, self.barrer, callback_group=grupo)
        self.create_timer(1.0 / max(self.hz_informe, 0.01), self.informar,
                          callback_group=grupo)

        self.t_informe = 0.0
        self.tarde = 0            # veces que el auditor ha ido con retraso

        self.csv = None
        if self.registro:
            self.abrir_csv()

        self.ok_pub.publish(Bool(data=True))
        self.get_logger().info(
            'auditor en marcha | vigilando ' + ', '.join(c for _i, c in self.motores)
            + ' + cc + la punta | "ros2 service call /scara/auditor/comprobar '
            'std_srvs/srv/Trigger" para la prueba de ida y vuelta')

    # --------------------------------------------------------------- utilidades
    @staticmethod
    def sello(cabecera) -> int:
        return cabecera.stamp.sec * 1_000_000_000 + cabecera.stamp.nanosec

    @staticmethod
    def valores(msg: JointState) -> tuple:
        """Lo que tiene que sobrevivir al salto, tal cual."""
        return (tuple(msg.name), tuple(msg.position), tuple(msg.velocity),
                tuple(msg.effort))

    def iguales(self, a: tuple, b: tuple) -> bool:
        if self.tol <= 0.0:
            return a == b
        if a[0] != b[0] or len(a[1]) != len(b[1]):
            return False
        return all(abs(x - y) <= self.tol
                   for ta, tb in zip(a[1:], b[1:]) for x, y in zip(ta, tb))

    # ------------------------------------------------- salto 1: aduana -> motor
    def on_aduana(self, clave: str, msg: JointState):
        ahora = time.monotonic()
        with self.lock:
            s = self.saltos[clave]
            s.entrados += 1
            self.pendientes[clave][self.sello(msg.header)] = Pendiente(
                ahora, self.valores(msg))
            self.comprobar_salto(clave, msg, ahora)

    def comprobar_salto(self, clave: str, msg: JointState, ahora: float):
        """Un brinco mas grande de lo que el motor puede dar en ese tiempo.

        Desde el 28/09/2026 la aduana comprueba el XOR de cada linea y tira las
        rotas, asi que un salto imposible ya no deberia venir de un byte
        cambiado: es la pista de muestras que no llegaron, o de una linea rota
        que por casualidad cuadro el checksum.
        """
        if not msg.position:
            return
        i = self.juntas.index(msg.name[0]) if msg.name else 0
        q = msg.position[0]
        # El intervalo se mide con el SELLO, no con la hora de llegada: dos
        # mensajes pueden llegar pegados porque el ejecutor los saco juntos de
        # su cola, y entonces el techo sale ridiculo y lo pasa cualquier cosa.
        # El sello lo pone la aduana al partir la linea de telemetria, que es
        # cuando de verdad se midio.
        t = self.sello(msg.header) * 1e-9
        cero = leer_cero(msg.header.frame_id)
        previo = self.ultimo_q.get(clave)
        self.ultimo_q[clave] = (q, t, ahora, cero)
        if previo is None:
            return
        if cero != previo[3]:
            # Se acaba de fijar el cero: la lectura ha saltado a 0 sin que el
            # brazo se mueva. Es la placa cambiando de origen, no un dato que
            # llego mal.
            return
        q_ant, t_ant = previo[0], previo[1]
        dt = t - t_ant
        if dt < 0.005 or dt > 1.0:
            return
        techo = self.max_vel_hw[i] * dt * self.margen_salto
        if abs(q - q_ant) > techo:
            self.saltos_imposibles += 1
            self.get_logger().warning(
                f'{clave}: salto de {abs(q - q_ant):.5f} en {dt * 1000:.0f} ms, mas '
                f'de lo que puede moverse ({techo:.5f}). O se perdio una muestra, '
                f'o esa linea de telemetria llego rota',
                throttle_duration_sec=5.0)

    def on_junta(self, clave: str, msg: JointState):
        # Se juzga en barrer(), pasado t_gracia: ver la nota de evaluar_junta.
        # Pero para el salto de abajo se apunta ya, que cc la va a necesitar.
        with self.lock:
            if msg.position:
                self.piezas.setdefault(self.sello(msg.header), {})[clave] = (
                    msg.position[0], time.monotonic())
                self.saltos['cc'].entrados += 1
            self.cola_junta.append((clave, time.monotonic(), self.sello(msg.header),
                                    self.valores(msg)))

    def evaluar_junta(self, clave: str, ahora: float, sello: int, valores: tuple):
        """Compara lo que salio del motor con lo que le entro de la aduana.

        Se hace con retraso a proposito. El ejecutor de ROS 2 no reparte los
        callbacks en el orden en que llegaron los mensajes, sino segun le toca
        a cada suscripcion, asi que el mensaje de salida se puede procesar
        ANTES que el de entrada. Juzgando al momento, eso saldria como "el
        motor publico algo que nadie le mando", que es acusar de una perdida a
        quien no la tuvo. Con una decima de margen, el orden dejo de importar.
        """
        s = self.saltos[clave]
        pend = self.pendientes[clave].pop(sello, None)
        if pend is None:
            # O ya salio antes (duplicado), o nunca entro (se lo invento el
            # de abajo, o el de arriba no estaba escuchandose).
            if sello in self.vistos[clave]:
                s.duplicados += 1
            elif self.arrancado[clave]:
                # Antes de la primera casada no se juzga: el auditor se suscribe
                # a los dos extremos del salto en instantes distintos, y lo que
                # pille a medias no lo ha perdido nadie.
                s.colados += 1
                self.get_logger().warning(
                    f'{clave} publico en scara/{clave}/junta algo que no habia '
                    f'llegado por scara/{clave}/aduana',
                    throttle_duration_sec=10.0)
            return
        if self.vistos[clave] and sello < self.vistos[clave][-1]:
            s.desordenados += 1
        self.vistos[clave].append(sello)
        s.salidos += 1
        lat = ahora - pend.t
        s.lat_suma += lat
        s.lat_max = max(s.lat_max, lat)
        if not self.iguales(pend.valores, valores):
            s.alterados += 1
            self.get_logger().error(
                f'{clave} cambio el dato: entro {pend.valores[1]} y salio '
                f'{valores[1]}')
        if not self.arrancado[clave]:
            self.arrancado[clave] = True
            # Este es el primer mensaje que se ve cruzar el salto. Lo que
            # quedaba pendiente de antes se tira sin contarlo: la aduana lo
            # publico cuando el motor a lo mejor ni se habia suscrito todavia
            # -el emparejado de cada suscripcion pasa a su ritmo-, y apuntarlo
            # como perdido seria acusar al motor de lo que hizo el arranque.
            for viejo in [k for k in self.pendientes[clave] if k < sello]:
                del self.pendientes[clave][viejo]
                s.descartados += 1

    # ------------------------------------------------------ salto 2: motor -> cc
    def on_js(self, msg: JointState):
        """joint_states: una muestra entera, con el sello de sus tres piezas."""
        ahora = time.monotonic()
        with self.lock:
            # La mitad de la FK se empareja por sello y no le afecta el orden.
            self.guardar_ciclo(self.sello(msg.header), 'juntas',
                               (tuple(msg.position), ahora))
            # Lo otro, a la cola: por lo mismo que evaluar_junta.
            self.cola_js.append((ahora, self.sello(msg.header), tuple(msg.position)))

    def evaluar_js(self, ahora: float, sello: int, posiciones: tuple):
        s = self.saltos['cc']
        self.n_js += 1
        if sello in self.js_vistos:
            s.duplicados += 1
            return
        if self.js_vistos and sello < self.js_vistos[-1]:
            s.desordenados += 1
        self.js_vistos.append(sello)

        fila = self.piezas.pop(sello, {})
        completa = True
        for i, clave in self.motores:
            pieza = fila.get(clave)
            if pieza is None or i >= len(posiciones):
                completa = False
                # Antes de la primera casada no se juzga: cc ya estaba
                # publicando cuando el auditor llego.
                if self.cc_arrancado:
                    s.colados += 1
                    self.get_logger().error(
                        f'joint_states trae una muestra con {clave} que {clave} no '
                        'publico: cc la ha armado con algo que no es de esa muestra')
                continue
            q, t_pub = pieza
            s.salidos += 1
            lat = ahora - t_pub
            s.lat_suma += lat
            s.lat_max = max(s.lat_max, lat)
            if not (q == posiciones[i] if self.tol <= 0.0
                    else abs(q - posiciones[i]) <= self.tol):
                s.alterados += 1
                self.get_logger().error(
                    f'cc cambio el dato de {clave}: el motor publico {q} y en '
                    f'joint_states sale {posiciones[i]}')
        if completa and not self.cc_arrancado:
            self.cc_arrancado = True
            # Lo de antes de la primera casada es del arranque: fuera, sin culpa.
            for viejo in [k for k in self.piezas if k < sello]:
                s.descartados += len(self.piezas.pop(viejo))

    # ------------------------------------------------- salto 3: juntas -> punta
    def on_pose(self, msg: PoseStamped):
        """La punta que publica cc tiene que ser la FK de las juntas de ese ciclo.

        Esto ya no es transporte: es comprobar que el dato no solo llego, sino
        que el nodo lo uso bien.
        """
        with self.lock:
            self.guardar_ciclo(self.sello(msg.header), 'punta',
                               (msg.pose, time.monotonic()))

    def guardar_ciclo(self, sello: int, mitad: str, dato):
        """Junta las dos mitades de un ciclo de cc y, cuando estan, las compara.

        cc sella joint_states y ee_pose del mismo ciclo con el MISMO sello, asi
        que se emparejan sin ambiguedad; lo que no se puede dar por hecho es el
        ORDEN en que llegan, porque son dos topics distintos. De ahi que se
        guarden las dos mitades y se compruebe cuando esten las dos.
        """
        ciclo = self.ciclos.setdefault(sello, {})
        ciclo[mitad] = dato
        if 'juntas' in ciclo and 'punta' in ciclo:
            del self.ciclos[sello]
            self.comparar_fk(ciclo)

    def comparar_fk(self, ciclo: dict):
        (q, t_js), (pose, t_pose) = ciclo['juntas'], ciclo['punta']
        s = self.saltos['fk']
        s.entrados += 1
        s.salidos += 1
        lat = abs(t_pose - t_js)
        s.lat_suma += lat
        s.lat_max = max(s.lat_max, lat)

        x, y, z, yaw = self.model.fk(list(q))
        d = math.dist((x, y, z), (pose.position.x, pose.position.y, pose.position.z))
        giro = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        d_yaw = abs(math.atan2(math.sin(giro - yaw), math.cos(giro - yaw)))
        self.error_fk = max(self.error_fk, d)
        if d > self.tol_fk or d_yaw > self.tol_fk:
            s.alterados += 1
            self.get_logger().error(
                f'la punta no es la FK de sus juntas: {d * 1000:.4f} mm y '
                f'{math.degrees(d_yaw):.4f} grados de diferencia')

    # ------------------------------------------------------------- lo de la placa
    def on_flags(self, msg: UInt8):
        self.flags, self.t_flags = msg.data, time.monotonic()

    def on_log(self, msg: String):
        with self.lock:
            self.log.append(msg.data)
        self.log_ev.set()

    def hay_placa(self) -> bool:
        return (time.monotonic() - self.t_flags) < 2.0

    # -------------------------------------------------------------- el barrido
    def barrer(self):
        """Juzga lo que ya ha reposado, y da por perdido lo que no llego.

        El orden importa: primero los mensajes de salida que estaban en cola
        -que son los que pueden cerrar un pendiente- y solo despues se declara
        perdido lo que siga sin cerrar. Al reves, se perderian cosas que
        estaban a punto de casar.
        """
        ahora = time.monotonic()
        maduro = ahora - self.t_gracia
        limite = ahora - self.t_perdido
        with self.lock:
            while self.cola_junta and self.cola_junta[0][1] < maduro:
                clave, t, sello, valores = self.cola_junta.popleft()
                self.evaluar_junta(clave, t, sello, valores)
            while self.cola_js and self.cola_js[0][0] < maduro:
                t, sello, posiciones = self.cola_js.popleft()
                self.evaluar_js(t, sello, posiciones)
            # Piezas que nunca salieron en un joint_states. Si a su muestra le
            # faltaba otra, cc hizo bien en no sacarla (mezclaria instantes);
            # si estaba entera y no salio, eso si es una perdida de cc.
            cc = self.saltos['cc']
            for sello in [k for k, f in self.piezas.items()
                          if max(t for _q, t in f.values()) < limite]:
                fila = self.piezas.pop(sello)
                if not self.cc_arrancado:
                    cc.descartados += len(fila)
                elif len(fila) < len(self.motores):
                    self.cc_incompletas += len(fila)
                else:
                    cc.perdidos += len(fila)
                    self.get_logger().error(
                        'cc no saco una muestra que tenia entera: se pierde entre '
                        'scara/<motor>/junta y joint_states',
                        throttle_duration_sec=10.0)
            for _i, clave in self.motores:
                viejos = [s for s, p in self.pendientes[clave].items() if p.t < limite]
                for sello in viejos:
                    del self.pendientes[clave][sello]
                    # Antes de ver cruzar el primero no se acusa a nadie, pero
                    # tampoco se hace como si no hubieran existido: van a la
                    # cuenta de descartados, para que entran = salen +
                    # perdidos + descartados y la tabla cuadre.
                    if self.arrancado[clave]:
                        self.saltos[clave].perdidos += 1
                    else:
                        self.saltos[clave].descartados += 1
                if viejos and self.arrancado[clave]:
                    self.get_logger().error(
                        f'{clave} no republico {len(viejos)} mensaje(s) que le mando '
                        f'la aduana: se pierden entre scara/{clave}/aduana y '
                        f'scara/{clave}/junta')

            # Un ciclo de cc con una sola mitad: publico las juntas y no la
            # punta, o al reves. No es un retraso, es una de las dos que falta.
            mancos = [k for k, c in self.ciclos.items()
                      if max(t for _d, t in c.values()) < limite]
            for k in mancos:
                falta = 'la punta' if 'juntas' in self.ciclos[k] else 'las juntas'
                del self.ciclos[k]
                self.saltos['fk'].perdidos += 1
                self.get_logger().error(
                    f'cc publico un ciclo al que le falta {falta}',
                    throttle_duration_sec=10.0)

    # ------------------------------------------------------------- el informe
    def tabla(self) -> str:
        t = time.monotonic() - self.t_ini
        lineas = [
            f'AUDITORIA DE LA CADENA   {t:6.1f} s vigilando',
            '   salto                      entran   salen perdidos alterados '
            ' lat_ms  max_ms   observaciones',
        ]
        with self.lock:
            for _i, clave in self.motores:
                lineas.append(self.saltos[clave].fila())
            cc = self.saltos['cc']
            cc.nota = f'{self.n_js} joint_states, uno por muestra'
            if self.cc_incompletas:
                cc.nota += (f'; {self.cc_incompletas} pieza(s) de muestras incompletas '
                            '(les falto otra pieza antes de cc: no se publican)')
            lineas.append(cc.fila())
            fk = self.saltos['fk']
            # Aqui la latencia no mide a cc: mide lo que tarda el auditor en
            # ver las dos mitades del mismo ciclo, porque cc las sella igual y
            # las publica seguidas. Lo que dice algo de cc es el error de FK.
            fk.nota = (f'error FK max {self.error_fk * 1000:.5f} mm; la latencia '
                       'de esta fila es separacion entre las dos mitades')
            lineas.append(fk.fila())
            sanos = all(s.sano() for s in self.saltos.values())
            imposibles = self.saltos_imposibles
        if imposibles:
            lineas.append(f'!! {imposibles} salto(s) fisicamente imposibles en la '
                          'telemetria: muestras perdidas o lineas rotas')
        if self.tarde:
            lineas.append(f'!! el auditor llego tarde {self.tarde} vez(ces): parte de '
                          'lo que cuenta como perdido puede haberlo perdido el')
        lineas.append('veredicto: ' + (
            'los datos llegan enteros por los tres saltos'
            if sanos and not imposibles else
            'HAY DATOS QUE NO LLEGAN BIEN, mira las filas con !!'))
        return '\n'.join(lineas)

    def informar(self):
        # Lo primero, mirarse a si mismo. Si el temporizador llega tarde es que
        # el nodo no da abasto, y entonces los mensajes que pierda EL van a
        # salir en el informe como perdidas de otros. Vale mas un auditor que
        # dice "no me fio de mi" que uno que acusa al vecino.
        ahora = time.monotonic()
        periodo = 1.0 / max(self.hz_informe, 0.01)
        if self.t_informe and (ahora - self.t_informe) > periodo * 1.25:
            self.tarde += 1
            self.get_logger().warning(
                f'el auditor va con retraso ({ahora - self.t_informe:.2f} s en vez '
                f'de {periodo:.2f}): las cuentas de este informe pueden acusar a '
                'otros de lo que se esta perdiendo el, baja la telemetria',
                throttle_duration_sec=10.0)
        self.t_informe = ahora

        texto = self.tabla()
        self.informe_pub.publish(String(data=texto))
        with self.lock:
            sanos = all(s.sano() for s in self.saltos.values()) and not self.saltos_imposibles
        self.ok_pub.publish(Bool(data=sanos))
        if self.csv:
            self.anotar()

    def abrir_csv(self):
        try:
            ruta = os.path.expanduser(self.registro)
            self.csv = open(ruta, 'w', buffering=1)
            self.csv.write('t,salto,entran,salen,perdidos,alterados,dobles,colados,'
                           'desordenados,descartados,lat_media_ms,lat_max_ms\n')
            self.get_logger().info(f'anotando la auditoria en {ruta}')
        except OSError as exc:
            self.csv = None
            self.get_logger().error(f'no puedo escribir en {self.registro}: {exc}')

    def anotar(self):
        t = time.monotonic() - self.t_ini
        with self.lock:
            filas = list(self.saltos.values())
        for s in filas:
            media, maximo = s.latencia()
            self.csv.write(f'{t:.3f},{s.nombre},{s.entrados},{s.salidos},{s.perdidos},'
                           f'{s.alterados},{s.duplicados},{s.colados},'
                           f'{s.desordenados},{s.descartados},{media:.3f},'
                           f'{maximo:.3f}\n')

    # ------------------------------------------------------- ida y vuelta real
    def ida_y_vuelta(self) -> tuple[bool, str]:
        """Le pregunta a la placa lo que la aduana le dijo al conectar.

        La aduana, nada mas abrir el puerto, le manda un "R j qmin qmax vmax"
        por junta con los limites del YAML. El "?" hace que la placa los lea de
        vuelta. Si coinciden, el dato hizo el viaje entero -PC, puerto, placa,
        puerto, PC- sin estropearse, y no se ha movido ni un motor.
        """
        with self.lock:
            self.log.clear()
        self.log_ev.clear()
        t0 = time.monotonic()
        self.raw_pub.publish(String(data='?'))

        limites = {}
        while time.monotonic() - t0 < self.t_espera:
            self.log_ev.wait(0.2)
            self.log_ev.clear()
            with self.lock:
                lineas = list(self.log)
            limites = {}
            for linea in lineas:
                m = _LIMITES.search(linea)
                if m:
                    limites[len(limites)] = tuple(float(v) for v in m.groups())
            if len(limites) >= len(self.juntas):
                break
        ms = (time.monotonic() - t0) * 1000.0

        if not limites:
            return False, (f'la placa no contesto al "?" en {self.t_espera:.1f} s: '
                           'no hay enlace, o la aduana no esta corriendo')

        fallos = []
        for i, j in enumerate(self.juntas):
            if i not in limites:
                fallos.append(f'{j}: la placa no la volco')
                continue
            lo, hi = self.model.limits()[i]
            v_lo, v_hi, v_max = limites[i]
            if abs(v_lo - lo) > 1e-4 or abs(v_hi - hi) > 1e-4:
                fallos.append(f'{j}: mande [{lo:.4f}, {hi:.4f}] y me devuelve '
                              f'[{v_lo:.4f}, {v_hi:.4f}]')
            elif abs(v_max - self.max_vel_hw[i]) > 1e-4:
                fallos.append(f'{j}: vmax mandado {self.max_vel_hw[i]:.4f}, devuelto '
                              f'{v_max:.4f} (puede ser que la aduana lleve otro '
                              'max_vel_hw)')
        if fallos:
            return False, f'ida y vuelta en {ms:.0f} ms, PERO: ' + '; '.join(fallos)
        return True, (f'ida y vuelta correcta en {ms:.0f} ms: las {len(self.juntas)} '
                      'juntas vuelven con los limites que se les mandaron')

    # -------------------------------------------------------------- la sonda
    def sonda(self) -> tuple[bool, str]:
        """Mete una rampa conocida por la cadena y mira si sale igual.

        Publica en los mismos scara/<motor>/aduana que la aduana, asi que solo
        se hace cuando NO hay nadie mas publicando ahi: con dos fuentes, lo que
        salga por el otro lado no es de nadie. Y nunca con potencia: si esp_rec
        va con reenviar:=true, un dato inventado acabaria en la placa.
        """
        with self.lock:
            ocupada = [c for _i, c in self.motores
                       if self.ultimo_q.get(c)
                       and time.monotonic() - self.ultimo_q[c][2] < 1.0]
        if ocupada:
            return False, ('la aduana esta publicando ahora mismo en '
                           f'{", ".join(ocupada)}: la sonda solo se puede meter con '
                           'la cadena parada (lanza con hw:=false)')
        if self.hay_placa() and (self.flags & F_HABILITADO):
            return False, ('la placa esta con potencia: no meto datos de prueba en '
                           'la cadena con los motores alimentados')

        base = [(lo + hi) / 2.0 for lo, hi in self.model.limits()]
        periodo = 1.0 / max(self.sonda_hz, 1.0)
        # Amplitud pequena y lenta: aunque alguien este escuchando el otro
        # extremo, lo que se dibuje sera un movimiento suave y corto. Y que no
        # pase de la mitad de la velocidad real de cada junta: si no, el propio
        # auditor cantaria como "salto imposible" su sonda (el husillo, a 8 mm/s,
        # no puede seguir la rampa que si siguen los giros).
        vuelta = 2.0 * math.pi * max(self.sonda_hz, 1.0) / max(self.sonda_n, 1)
        amplitud = [min(0.05, (hi - lo) / 8.0, 0.5 * self.max_vel_hw[i] / vuelta)
                    for i, (lo, hi) in enumerate(self.model.limits())]
        antes = {c: (self.saltos[c].salidos, self.saltos[c].alterados,
                     self.saltos[c].perdidos) for _i, c in self.motores}
        antes_cc = (self.saltos['cc'].salidos, self.saltos['cc'].colados)

        for k in range(self.sonda_n):
            fase = 2.0 * math.pi * k / self.sonda_n
            stamp = self.get_clock().now().to_msg()
            for idx, (i, clave) in enumerate(self.motores):
                js = JointState()
                js.header.stamp = stamp
                js.name = [self.juntas[i]]
                js.position = [base[i] + amplitud[i] * math.sin(fase)]
                js.velocity = [amplitud[i] * math.cos(fase)]
                js.effort = [0.0]
                self.aduana_pub[clave].publish(js)
            time.sleep(periodo)
        time.sleep(self.t_perdido + 0.3)     # que termine de recorrer la cadena

        fallos = []
        for _i, clave in self.motores:
            s = self.saltos[clave]
            salidos = s.salidos - antes[clave][0]
            alterados = s.alterados - antes[clave][1]
            perdidos = s.perdidos - antes[clave][2]
            if salidos < self.sonda_n or alterados or perdidos:
                fallos.append(f'{clave}: {salidos}/{self.sonda_n} llegaron, '
                              f'{alterados} alterados, {perdidos} perdidos')
        cc_salidos = self.saltos['cc'].salidos - antes_cc[0]
        cc_colados = self.saltos['cc'].colados - antes_cc[1]
        if cc_colados:
            fallos.append(f'cc publico {cc_colados} valor(es) que nadie le mando')
        if fallos:
            return False, 'la sonda NO sale limpia: ' + '; '.join(fallos)
        return True, (f'sonda correcta: {self.sonda_n} muestras por motor entraron '
                      f'por scara/<motor>/aduana y salieron identicas; cc las paso a '
                      f'joint_states {cc_salidos} veces, sin inventarse ninguna')

    # ------------------------------------------------------------- el servicio
    def on_comprobar(self, request, response):
        if self.hay_placa():
            ok, detalle = self.ida_y_vuelta()
        else:
            self.get_logger().info('no hay placa: compruebo la cadena con la sonda')
            ok, detalle = self.sonda()
        response.success = ok
        response.message = detalle
        # Dos llamadas separadas y no una condicional: rclpy se queda con la
        # severidad de cada linea de codigo la primera vez que se usa, y si
        # luego sale por la otra rama lanza "Logger severity cannot be changed
        # between calls" y tira el nodo abajo.
        if ok:
            self.get_logger().info(detalle)
        else:
            self.get_logger().error(detalle)
        return response

    # ------------------------------------------------------------------ cierre
    def destroy_node(self):
        if self.csv:
            try:
                self.csv.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Auditor()
    ejecutor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=ejecutor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C rclpy cierra el contexto aunque haya un callback a medias,
        # y ese callback lanza RCLError al publicar: no es un fallo, es el final.
        if rclpy.ok():
            raise
    finally:
        node.get_logger().info('\n' + node.tabla())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
