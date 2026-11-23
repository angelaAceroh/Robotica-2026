"""Nodo `vector`: de la trayectoria al main.py de la ESP32 (MicroPython).

    ros2 run scara_kinematics vector --ros-args -p simular:=true        # sin placa
    ros2 run scara_kinematics vector                                    # con la placa
    ros2 topic pub --once /scara/vector/orden std_msgs/String "data: 'perforado'"
    ros2 topic pub --once /scara/vector/orden std_msgs/String "data: 'ir 430 -160 125'"

Coge la ruta de ruta.py (la del Jacobiano inverso) y, tramo a tramo, le manda a
la placa su VALOR FINAL en la forma que espera main.py:

    [pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, vel_teodoro]

en las unidades de la placa (Alvin y Simon en grados de su junta y RPM, Teodoro
en mm y mm/min, como en los programas de Sergio), y espera a que la placa conteste LISTO antes de
mandar el siguiente. Las velocidades salen de la trayectoria: cada motor gira
lo suyo en el tiempo que dura el tramo, para que los tres lleguen a la vez.

Por que un nodo en el PC y no en la placa: MicroPython no puede ejecutar un nodo
de ROS 2. La placa corre main.py, y este nodo es su cara en ROS: lo que se le
manda y lo que contesta sale por topics.

Ordenes (std_msgs/String en scara/vector/orden):
    perforado | contorno [perfil=..] [codo=..]   la ruta, tramo a tramo
    ir x y z                                    la punta a ese punto (mm), si llega
    parar                                       manda 'q' a la placa y corta
    [a, b, c, d, e, f]                          un vector tal cual, para probar
    ?  |  l  |  h simon  |  fijar teodoro -5    pasan tal cual a la placa (a Alvin
                                                no se le da la h: «NO DARLE A LA H»)

Salidas:
    scara/vector/enviado   std_msgs/Float64MultiArray  el ultimo vector mandado
    scara/vector/posicion  geometry_msgs/Vector3       Alvin y Simon en grados, Teodoro en mm
    scara/vector/placa     std_msgs/String             cada linea que dice la placa
    scara/vector/estado    std_msgs/String             que hace y por que tramo va
    scara/vector/alcance   std_msgs/String             el veredicto del ultimo 'ir'

Nada sale hacia la placa si algun motor se sale de su recorrido
(limites_motor, en config/micropython.yaml).

No se lanza a la vez que `esp` (brazo.launch.py): los dos querrian el mismo
puerto, y hablan protocolos distintos (scara_esp32 frente a main.py).
"""
from __future__ import annotations

import math
import re
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String

from scara_kinematics import alcance as A
from scara_kinematics import ruta as R
from scara_kinematics.model import declare_model_params

try:
    import serial
    from scara_kinematics.protocolo import buscar_puerto
except ImportError:          # sin pyserial solo vale simular:=true
    serial = None
    buscar_puerto = None

NOMBRES = ('Alvin', 'Simon', 'Teodoro')
NUMERO = r'(-?\d+(?:\.\d+)?)'
TRES = re.compile(r'^(LISTO|POS|PARADO)\s+' + NUMERO + r'\s+' + NUMERO + r'\s+' + NUMERO)
TELEMETRIA = re.compile(r'(Alvin|Simon|Teodoro)\s+' + NUMERO + '(?:°|mm)')
RECIBE = re.compile(r'^RECIBE (\w+) pos ' + NUMERO + ' vel ' + NUMERO + ' esta ' + NUMERO)
LIMITES = re.compile(r'^LIMITES Alvin ' + NUMERO + ' ' + NUMERO + ' Simon ' + NUMERO + ' '
                     + NUMERO + ' Teodoro ' + NUMERO + ' ' + NUMERO)
CONFIRMAR_S = 1.5        # lo que tiene la placa para devolver el vector y los RECIBE


def checksum(texto: str) -> str:
    """Suma de los bytes, modulo 256, en hexadecimal: la misma cuenta que main.py."""
    return '%02X' % (sum(texto.encode()) & 0xFF)


def numeros(texto: str) -> list[float]:
    return [float(v) for v in re.findall(NUMERO, texto)]


class Vector(Node):

    def __init__(self):
        super().__init__('vector')
        self.model = declare_model_params(self)
        self.robot = R.Robot.desde_modelo(self.model)

        p = self.declare_parameter
        self.puerto = str(p('puerto', '').value)
        self.baudios = int(p('baudios', 115200).value)
        self.simular = bool(p('simular', False).value)
        self.ruta_inicial = str(p('ruta', '').value)
        self.perfil = str(p('perfil', 'trapecio').value)
        self.codo = str(p('codo', 'down').value)
        # De junta a motor y hasta donde llega cada motor: config/micropython.yaml.
        # RUTA.md §8 dice como medir lo que falta con la placa conectada.
        d = A.Calibracion()
        defectos = {'limites_motor': [x for par in d.limites for x in par],
                    'reduccion': d.reduccion, 'signo': d.signo, 'cero': d.cero,
                    'paso_husillo_mm': d.paso_husillo_mm, 'velocidades': d.velocidades}
        self.cal = A.Calibracion.desde_parametros(
            lambda nombre, _: p(nombre, defectos[nombre]).value)
        self.vel_min = float(p('vel_min', 10.0).value)
        self.vel_max = float(p('vel_max', 60.0).value)
        self.vel_aprox = float(p('vel_aprox', 20.0).value)
        self.espera_arranque = float(p('espera_arranque', 4.0).value)
        # Espejo en Gazebo: los grados de los motores, pasados a junta, a los
        # mandos de gz. Con la placa, gz copia al brazo real; con simular, a la
        # placa de mentira.
        self.gz = bool(p('gz', False).value)
        self.cmd_prefix = str(p('cmd_prefix', '/scara_urdf/cmd/').value)
        self.juntas = list(self.model.joint_names)[:3]

        self.enviado_pub = self.create_publisher(Float64MultiArray, 'scara/vector/enviado', 10)
        self.pos_pub = self.create_publisher(Vector3, 'scara/vector/posicion', 10)
        self.placa_pub = self.create_publisher(String, 'scara/vector/placa', 50)
        self.estado_pub = self.create_publisher(String, 'scara/vector/estado', 10)
        self.alcance_pub = self.create_publisher(String, 'scara/vector/alcance', 10)
        # Error de posicion al acabar cada vector: grados de motor y mm de la punta
        self.error_pub = self.create_publisher(Vector3, 'scara/vector/error', 10)
        self.error_punta_pub = self.create_publisher(Float64, 'scara/vector/error_punta', 10)
        self.create_subscription(String, 'scara/vector/orden', self.on_orden, 10)
        self.gz_pub = ({j: self.create_publisher(Float64, f'{self.cmd_prefix}{j}', 10)
                        for j in self.juntas} if self.gz else {})
        # Sin el paso del husillo, Teodoro no dice a que altura esta: en gz se
        # ensena la d3 que pide la trayectoria (mm), para ver a donde iria.
        self.d3_vista: float | None = None

        self.ser = None
        self.buffer = ''
        self.abierto_en = None
        self.lista = False                # la placa ha arrancado main.py
        self.pos_motor: list[float] | None = None    # grados, lo ultimo que dijo
        self.pos_vivo: list[float] | None = None     # y tambien la telemetria, 10 Hz
        # (etiqueta, destino en grados de motor, duracion o None, d3 de la junta en mm)
        self.cola: list[tuple[str, list, float | None, float]] = []
        self.esperando: tuple[str, float] | None = None       # (etiqueta, plazo)
        self.pendiente: str | None = None                      # orden a la espera de POS
        self.pedir_pos = False            # mandar '?' en cuanto la placa este lista
        self.pedir_limites = True         # y 'l', para comparar sus limites con los del YAML
        # El ultimo vector mandado y lo que la placa ha confirmado de el
        self.confirma: dict | None = None
        self.ultimo_vec: list[float] | None = None
        # La placa de mentira (simular): donde esta cada motor y que movimiento lleva
        self.sim_pos = [0.0, 0.0, 0.0]                 # la 'h' con el brazo estirado
        self.sim_mov: list = [None, None, None]        # (desde, hasta, rpm, t0)
        self.sim_listo = False
        self.sim_t0 = time.monotonic()
        self.sim_t_tel = 0.0
        self._sim_salida: list[str] = []

        if not self.cal.husillo_medido:
            self.get_logger().warning(
                'paso_husillo_mm sin poner: Teodoro va con velocidad 0 (no se mueve). '
                'Mide cuantos mm sube por vuelta y pasalo con -p paso_husillo_mm:=...')
        modo = ('SIMULANDO la placa' if self.simular else 'placa por USB') + \
            (' + espejo en Gazebo' if self.gz else '')
        self.get_logger().info(
            f'vector | {modo} | limites {self.cal.limites} | reduccion '
            f'{self.cal.reduccion} | cero {self.cal.cero} | signo {self.cal.signo} | '
            f'velocidades {self.cal.velocidades} (RPM, RPM, mm/min), entre '
            f'{self.vel_min:.0f} y {self.vel_max:.0f} | ordenes por scara/vector/orden')
        if self.ruta_inicial:
            self.on_orden(String(data=self.ruta_inicial))
        self.create_timer(0.02, self.tic)

    # ================================================================ junta <-> motor
    def a_motor(self, q) -> list[float | None]:
        """[theta1 rad, theta2 rad, d3 mm] -> Alvin y Simon en grados, Teodoro en mm."""
        return self.cal.a_motor(q)

    def vector_hacia(self, destino, desde, duracion: float | None):
        """El vector de main.py para ir de `desde` a `destino` (unidades de la placa).

        Con `velocidades` (config/micropython.yaml), cada motor va a la suya,
        fija: [pos_alvin, 14, pos_simon, 30, pos_teodoro, 60]. Si alguna es 0,
        se calculan para que lleguen a la vez: con duracion, cada motor gira lo
        suyo en ese tiempo; sin ella, el que mas gira va a vel_aprox. Las
        velocidades, como en la placa: RPM de Alvin y Simon, mm/min de Teodoro.
        """
        if all(v > 0.0 for v in self.cal.velocidades):
            vec = A.vector_placa(destino, desde, self.cal)
            # cuanto tardara el mas lento, para saber cuanto esperar el LISTO
            previsto = max(A.segundos(vec[2 * i] - desde[i], vec[2 * i + 1], i)
                           for i in range(3))
            return vec, previsto, []
        if destino[2] is None:                       # husillo sin calibrar
            destino = [destino[0], destino[1], desde[2]]
        delta = [abs(b - a) for a, b in zip(desde, destino)]
        if duracion is None:
            duracion = max(max(A.segundos(d, self.vel_aprox, i) for i, d in enumerate(delta)),
                           1e-6)
        vec, avisos = [], []
        for i, (nombre, d, pos) in enumerate(zip(NOMBRES, delta, destino)):
            if d < 0.5:                              # main.py ya lo da por llegado
                rpm = 0.0
            else:
                ideal = d / (A.POR_SEGUNDO[i] * duracion)
                rpm = min(max(ideal, self.vel_min), self.vel_max)
                if abs(rpm - ideal) > 0.05:
                    avisos.append(f'{nombre} {ideal:.1f}->{rpm:.0f} {A.VEL_UNIDADES[i]}')
            vec += [round(pos, 3), round(rpm, 2)]
        if not self.cal.husillo_medido:
            vec[5] = 0.0
        return vec, duracion, avisos

    # ======================================================================= ordenes
    def on_orden(self, msg: String):
        texto = msg.data.strip()
        if not texto:
            return
        cabeza = texto.split()[0].lower()
        if cabeza == 'parar':
            self.cola.clear()
            self.esperando = None
            self.pendiente = None
            self.pedir_pos = False
            self._mandar('q')
            self.get_logger().warning('parado: q a la placa')
            return
        if texto.startswith('['):
            try:
                v = [float(x) for x in texto.strip('[]').replace(' ', '').split(',')]
                assert len(v) == 6
            except (ValueError, AssertionError):
                self.get_logger().error(f'«{texto}» no es un vector de 6 numeros')
                return
            malos = self.cal.fuera([v[0], v[2], v[4]])
            if malos:
                self.get_logger().error(f'«{texto}» no sale: ' + '; '.join(malos))
                return
            if self.cola or self.esperando:
                self.get_logger().error('hay algo en marcha: manda parar antes')
                return
            if self.pos_motor is None:
                self.get_logger().error('todavia no se donde estan los motores (manda ?)')
                return
            previsto = max(A.segundos(v[2 * i] - self.pos_motor[i], v[2 * i + 1], i)
                           for i in range(3))
            self._enviar_vector('a mano', v, previsto)
            return
        if cabeza in ('?', 'h', 'q', 'l', 'fijar'):
            self._mandar(texto)
            return
        if cabeza == 'ir':
            try:
                x, y, z = (float(v) for v in texto.split()[1:4])
            except ValueError:
                self.get_logger().error('ir pide tres numeros en mm: ir x y z')
                return
            ver = A.alcanzar((x, y, z), self.cal, self.robot, self.pos_motor)
            self.alcance_pub.publish(String(data=ver.resumen()))
            # Dos llamadas distintas a proposito: rclpy no deja que la misma linea
            # registre unas veces como info y otras como error.
            if not ver.alcanzable:
                self.get_logger().error(self._informe(ver))
                return
            self.get_logger().info(self._informe(ver))
        elif cabeza not in R.RUTAS:
            self.get_logger().error(f'no conozco {cabeza!r}: hay {", ".join(R.RUTAS)}, '
                                    f'ir x y z, parar, un vector [..] o ? / l / q / '
                                    f'h <motor> / fijar <motor> <pos>')
            return
        if self.cola or self.esperando:
            self.get_logger().error('ya hay una ruta en marcha: manda parar antes')
            return
        # Hace falta saber donde esta cada motor para la aproximacion. Se pide
        # en cuanto la placa este lista (al arrancar todavia no lo esta).
        self.pendiente = texto
        self.pedir_pos = True

    def _preparar(self, texto: str):
        palabras = texto.split()
        nombre = palabras[0].lower()
        if nombre == 'ir':
            # Se vuelve a resolver con la posicion de ahora: decide la rama del codo
            ver = A.alcanzar(tuple(float(v) for v in palabras[1:4]), self.cal,
                             self.robot, self.pos_motor)
            if not ver.alcanzable:
                self.get_logger().error(ver.resumen())
                return
            self.cola = [(f'ir {" ".join(palabras[1:4])}', ver.motor, None, ver.junta[2])]
            return
        op = dict(w.split('=', 1) for w in palabras[1:] if '=' in w)
        try:
            plan = R.planificar(R.RUTAS[nombre], self.robot, op.get('perfil', self.perfil),
                                op.get('codo', self.codo))
        except (ValueError, R.RutaError) as e:
            self.get_logger().error(f'la ruta {nombre} no sale: {e}')
            return
        # El valor final de cada tramo: la ultima muestra de ese tramo
        finales = []
        for i, tr in enumerate(plan.tramos):
            k = max(j for j in range(len(plan.t)) if plan.tramo[j] == i)
            finales.append((f'tramo {tr.n} {tr.tipo}', plan.q[k], tr.duracion))

        # Se guardan los DESTINOS; el vector se calcula al mandarlo, desde donde
        # la placa dice que esta (si un motor se paso, el siguiente lo corrige).
        cola = [('aproximacion', self.a_motor(plan.q[0]), None, plan.q[0][2])]
        cola += [(etiqueta, self.a_motor(q), dur, q[2]) for etiqueta, q, dur in finales]
        # Solo para avisar ya de las velocidades que no caben en la placa
        desde, avisos = list(self.pos_motor), []
        for etiqueta, destino, dur, _ in cola:
            vec, _, av = self.vector_hacia(destino, desde, dur)
            avisos += [f'{etiqueta}: {a}' for a in av]
            desde = [vec[0], vec[2], vec[4]]
        # Todos los destinos dentro del recorrido de los motores, o no sale nada
        for etiqueta, destino, _, _ in cola:
            malos = self.cal.fuera(destino)
            if malos:
                self.get_logger().error(f'la ruta {nombre} no cabe en el recorrido de los '
                                        f'motores ({etiqueta}): ' + '; '.join(malos))
                return
        self.cola = cola
        self.get_logger().info(f'{nombre}: {len(cola)} vectores (aproximacion + '
                               f'{len(finales)} tramos)')
        if avisos:
            self.get_logger().warning(
                f'{len(avisos)} velocidades recortadas al rango de la placa '
                f'[{self.vel_min:.0f}, {self.vel_max:.0f}] (RPM, mm/min), esos motores llegan antes '
                f'(o despues) que los otros: ' + '; '.join(avisos[:6])
                + (' ...' if len(avisos) > 6 else ''))

    # ======================================================================= placa
    def _abrir(self):
        if self.simular or self.ser is not None:
            return
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
        self.abierto_en = time.monotonic()
        self.lista = False
        self.get_logger().info(f'{puerto} abierto. Con este CH340 abrir el puerto reinicia '
                               f'la ESP32: espero a que main.py diga «Sistema listo»')

    def _mandar(self, linea: str, eco: bool = True):
        if eco:
            self.get_logger().info(f'-> placa: {linea}')
        if self.simular:
            self._simular_orden(linea)
            return
        if self.ser is None:
            self.get_logger().error(f'«{linea}» no sale: la placa no esta conectada')
            return
        try:
            self.ser.write((linea + '\r\n').encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'se perdio la placa: {e}')
            self.ser = None

    def _leer(self) -> list[str]:
        if self.simular:
            return self._simular_lineas()
        if self.ser is None:
            return []
        try:
            datos = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'se perdio la placa: {e}')
            self.ser = None
            return []
        self.buffer += datos.decode('utf-8', errors='replace')
        *lineas, self.buffer = self.buffer.split('\n')
        return [ln.strip() for ln in lineas if ln.strip()]

    # ======================================================================= el reloj
    def tic(self):
        self._abrir()
        if self.gz:
            self._espejo()
        if not self.simular and self.ser is not None and not self.lista \
                and time.monotonic() - self.abierto_en > self.espera_arranque:
            self.lista = True       # no ha dicho nada, pero ya deberia estar
        for linea in self._leer():
            self._atender(linea)
        if self.pedir_limites and (self.simular or self.lista):
            self.pedir_limites = False
            self._mandar('l')
        if self.pedir_pos and (self.simular or self.lista):
            self.pedir_pos = False
            self._mandar('?')
        c = self.confirma
        if c is not None and not c['hecho'] and time.monotonic() - c['t'] > CONFIRMAR_S:
            falta = [n for n in NOMBRES if n not in c['nodos']]
            self._sin_confirmar(
                'la placa no ha devuelto ' + ('el vector' if not c['eco'] else '')
                + (' ni ' if not c['eco'] and falta else '')
                + (f'lo que toma {", ".join(falta)}' if falta else '')
                + f' en {CONFIRMAR_S:g} s')

        if self.esperando is not None:
            etiqueta, plazo = self.esperando
            if time.monotonic() > plazo:
                self.get_logger().error(f'{etiqueta}: la placa no ha dicho LISTO a tiempo. '
                                        f'Mando q y corto la ruta')
                self._mandar('q')
                self.cola.clear()
                self.esperando = None
            return
        if self.cola and (self.simular or self.lista):
            etiqueta, destino, dur, self.d3_vista = self.cola.pop(0)
            vec, dur, _ = self.vector_hacia(destino, self.pos_motor, dur)
            malos = self.cal.fuera([vec[0], vec[2], vec[4]])
            if malos:                      # ultima barrera antes del USB
                self.get_logger().error(f'{etiqueta} no sale: ' + '; '.join(malos))
                self.cola.clear()
                return
            self.estado_pub.publish(String(data=f'{etiqueta} ({len(self.cola)} detras)'))
            self._enviar_vector(etiqueta, vec, dur)
        elif not self.cola:
            self.estado_pub.publish(String(data='en espera'))

    def _atender(self, linea: str):
        self.placa_pub.publish(String(data=linea))
        if 'Sistema listo' in linea:
            self.lista = True
        m = TRES.match(linea)
        if m:
            self.pos_motor = [float(m.group(i)) for i in (2, 3, 4)]
            self.pos_vivo = list(self.pos_motor)
            self.pos_pub.publish(Vector3(x=self.pos_motor[0], y=self.pos_motor[1],
                                         z=self.pos_motor[2]))
            if m.group(1) == 'LISTO' and self.esperando is not None:
                self.get_logger().info(
                    f'LISTO ({self.esperando[0]}): ' + '   '.join(
                        f'{n} {float(m.group(i)):+.2f}{u}'
                        for n, i, u in zip(NOMBRES, (2, 3, 4), A.UNIDADES)))
                self._error_posicion(self.pos_motor)
                self.esperando = None
                if self.confirma is not None and not self.confirma['hecho']:
                    self._sin_confirmar('la placa ha dicho LISTO sin confirmar el vector')
                elif not self.cola:
                    self.get_logger().info('terminado: no queda nada por mandar')
            elif m.group(1) == 'PARADO':
                self.cola.clear()
                self.esperando = None
                self.confirma = None
            elif m.group(1) == 'POS' and self.pendiente:
                texto, self.pendiente = self.pendiente, None
                self._preparar(texto)
            return
        if linea.startswith('ERROR vector checksum') and self.confirma is not None:
            c = self.confirma
            if c['reintentos'] < 1:        # el CH340 mete ruido: una vez mas
                c['reintentos'] += 1
                c['t'], c['eco'], c['nodos'] = time.monotonic(), False, set()
                self.get_logger().warning('el USB ha estropeado el vector (checksum): '
                                          'lo mando otra vez')
                self._mandar(c['linea'], eco=False)
            else:
                self._sin_confirmar(linea[6:])
            return
        if linea.startswith('RECHAZADO'):
            self.get_logger().error('la placa ha RECHAZADO el vector: algun nodo se saldria '
                                    'de su recorrido (main.py, tabla MOTORES). No se ha '
                                    'movido nada')
            self.cola.clear()
            self.esperando = None
            self.confirma = None
            return
        if linea.startswith('ERROR'):
            self.get_logger().error(f'la placa dice: {linea[6:]}. Corto la ruta')
            self.cola.clear()
            return
        # Lo que le llega a la placa y lo que toma cada nodo, comparado con lo mandado
        if linea.startswith('VECTOR'):
            self._comprobar_eco(linea)
            return
        r = RECIBE.match(linea)
        if r:
            self._comprobar_nodo(r)
            return
        lim = LIMITES.match(linea)
        if lim:
            self._comprobar_limites([float(lim.group(i)) for i in range(1, 7)])
            return
        if linea.startswith(('[!]', '[✗]', '[🏠]')):
            self.get_logger().info(f'   placa: {linea}')
            return
        tel = dict((n, float(v)) for n, v in TELEMETRIA.findall(linea))
        if linea.startswith('T:') and len(tel) == 3:
            self.pos_vivo = [tel['Alvin'], tel['Simon'], tel['Teodoro']]
            self.pos_pub.publish(Vector3(x=tel['Alvin'], y=tel['Simon'], z=tel['Teodoro']))

    # ============================================= que llegue y que se entienda
    def _enviar_vector(self, etiqueta: str, vec, dur: float):
        """Manda el vector con su checksum y deja apuntado que hay que confirmar."""
        texto = '[' + ', '.join(f'{v:g}' for v in vec) + ']'
        linea = f'{texto} *{checksum(texto)}'
        self.ultimo_vec = [float(v) for v in vec]
        self.confirma = {'vec': self.ultimo_vec, 'linea': linea, 't': time.monotonic(),
                         'eco': False, 'nodos': set(), 'hecho': False, 'reintentos': 0}
        self.esperando = (etiqueta, time.monotonic() + 3.0 * dur + 10.0)
        self.enviado_pub.publish(Float64MultiArray(data=self.ultimo_vec))
        self.get_logger().info(f'VECTOR -> placa ({etiqueta}, ~{dur:.1f} s): {linea}')
        self._mandar(linea, eco=False)

    def _comprobar_eco(self, linea: str):
        """VECTOR [...] checksum ok: ¿la placa ha leido lo que se mando?"""
        c = self.confirma
        cuerpo = linea[linea.find('['):linea.find(']') + 1]
        leido = numeros(cuerpo)
        if c is None or c['hecho']:
            self.get_logger().info(f'   la placa recibe {linea[7:]}')
            return
        if len(leido) != 6 or any(abs(a - b) > 1e-3 for a, b in zip(leido, c['vec'])):
            self._sin_confirmar(f'la placa ha leido {cuerpo} y se mando '
                                f'[{", ".join(f"{v:g}" for v in c["vec"])}]')
            return
        c['eco'] = True
        self.get_logger().info(f'   la placa recibe {linea[7:]}   ✓ igual que lo mandado')
        self._quizas_confirmado()

    def _comprobar_nodo(self, r):
        """RECIBE <nodo> pos P vel V: ¿ese nodo ha tomado SU pareja?"""
        nombre, pos, vel = r.group(1), float(r.group(2)), float(r.group(3))
        u, uv = ((A.UNIDADES[NOMBRES.index(nombre)], A.VEL_UNIDADES[NOMBRES.index(nombre)])
                 if nombre in NOMBRES else ('', ''))
        texto = (f'   nodo {nombre:<8s} toma  pos {pos:+8.2f}{u}   vel {vel:g} {uv}   '
                 f'(estaba en {float(r.group(4)):+.2f}{u})')
        c = self.confirma
        if c is None or c['hecho'] or nombre not in NOMBRES:
            self.get_logger().info(texto)
            return
        i = NOMBRES.index(nombre)
        if abs(pos - c['vec'][2 * i]) > 1e-3 or abs(vel - c['vec'][2 * i + 1]) > 1e-3:
            self._sin_confirmar(f'{nombre} ha tomado pos {pos:g} vel {vel:g} y le tocaba '
                                f'pos {c["vec"][2 * i]:g} vel {c["vec"][2 * i + 1]:g} '
                                f'(vector[{2 * i}] y vector[{2 * i + 1}])')
            return
        c['nodos'].add(nombre)
        self.get_logger().info(texto + '   ✓')
        self._quizas_confirmado()

    def _quizas_confirmado(self):
        c = self.confirma
        if c['eco'] and len(c['nodos']) == 3 and not c['hecho']:
            c['hecho'] = True
            ms = (time.monotonic() - c['t']) * 1000
            self.get_logger().info(f'   CONFIRMADO en {ms:.0f} ms: la placa leyo el vector '
                                   f'entero y cada nodo tomo su pareja')

    def _sin_confirmar(self, motivo: str):
        """Algo no cuadra entre lo mandado y lo entendido: se para todo."""
        self.get_logger().error(f'VECTOR NO CONFIRMADO: {motivo}. Mando q y corto')
        self._mandar('q')
        self.cola.clear()
        self.esperando = None
        self.confirma = None

    def _comprobar_limites(self, placa: list[float]):
        pc = [x for par in self.cal.limites for x in par]
        if all(abs(a - b) < 1e-6 for a, b in zip(placa, pc)):
            self.get_logger().info(f'limites: la placa y config/micropython.yaml coinciden '
                                   f'(Alvin [{pc[0]:g}, {pc[1]:g}], Simon [{pc[2]:g}, '
                                   f'{pc[3]:g}], Teodoro [{pc[4]:g}, {pc[5]:g}])')
        else:
            self.get_logger().warning(
                f'LIMITES DISTINTOS. PC (config/micropython.yaml): {pc}; placa (main.py, '
                f'tabla MOTORES): {placa}. Manda el mas estrecho de los dos: el PC no manda '
                f'nada fuera de los suyos y la placa rechaza lo que salga de los suyos')

    def _error_posicion(self, pos):
        """Al acabar un vector: donde quedo cada motor frente a lo pedido, y la punta."""
        vec = self.ultimo_vec
        if vec is None or pos is None:
            return
        err = [pos[i] - vec[2 * i] for i in range(3)]
        pedido = self.cal.a_junta([vec[0], vec[2], vec[4]])
        real = self.cal.a_junta(pos)
        for q in (pedido, real):                  # sin paso, la d3 no se sabe medir
            if q[2] is None:
                q[2] = self.d3_vista if self.d3_vista is not None else 0.0
        p_ped = R.fk_scara_rrp(pedido, self.robot)
        p_real = R.fk_scara_rrp(real, self.robot)
        e_xy = math.hypot(p_real[0] - p_ped[0], p_real[1] - p_ped[1])
        if self.cal.husillo_medido:
            e_z = p_real[2] - p_ped[2]
            punta = f'{math.hypot(e_xy, e_z):.2f} mm ({e_xy:.2f} en xy, {e_z:+.2f} en z)'
        else:
            punta = f'{e_xy:.2f} mm en xy (la z no se mide: falta el paso del husillo)'
        self.get_logger().info('ERROR DE POSICION  ' + '   '.join(
            f'{n} {e:+.2f}{u}' for n, e, u in zip(NOMBRES, err, A.UNIDADES))
            + f'   |   punta: {punta}')
        self.error_pub.publish(Vector3(x=err[0], y=err[1], z=err[2]))
        self.error_punta_pub.publish(Float64(data=float(math.dist(p_real, p_ped))))

    def _informe(self, ver: A.Veredicto) -> str:
        """El resultado de la trayectoria, en varias lineas, para la consola."""
        x, y, z = ver.punto
        cab = f'TRAYECTORIA  ir ({x:.1f}, {y:.1f}, {z:.1f}) mm -> '
        if not ver.alcanzable:
            return cab + 'NO ALCANZABLE\n' + '\n'.join(f'   {m}' for m in ver.motivos)
        t1, t2, d3 = ver.junta
        m = ver.motor
        teo = f'{m[2]:+.2f} mm' if m[2] is not None else 'sin calcular (falta el paso del husillo)'
        return (cab + f'ALCANZABLE, codo {ver.codo}\n'
                f'   junta:  theta1 {math.degrees(t1):+.2f}°   theta2 {math.degrees(t2):+.2f}°   '
                f'd3 {d3:.2f} mm\n'
                f'   motor:  Alvin {m[0]:+.2f}°   Simon {m[1]:+.2f}°   Teodoro {teo}')

    # ================================================================= sin placa
    def _espejo(self):
        """Los motores, pasados a junta, a los mandos de Gazebo (rad y m)."""
        pos = self._sim_ahora() if self.simular else self.pos_vivo
        if pos is None:
            return
        t1, t2, d3 = self.cal.a_junta(pos)
        if d3 is None:
            d3 = self.d3_vista
        valores = [t1, t2] + ([d3 * 1e-3] if d3 is not None else [])
        for j, v in zip(self.juntas, valores):
            self.gz_pub[j].publish(Float64(data=float(v)))

    def _sim_ahora(self) -> list[float]:
        """Donde esta ahora cada motor de la placa de mentira (grados, grados, mm)."""
        ahora = time.monotonic()
        pos = list(self.sim_pos)
        for i, mov in enumerate(self.sim_mov):
            if mov is not None:
                desde, hasta, rpm, t0 = mov
                avance = A.POR_SEGUNDO[i] * rpm * (ahora - t0)   # RPM o mm/min
                pos[i] = hasta if avance >= abs(hasta - desde) else \
                    desde + math.copysign(avance, hasta - desde)
        return pos

    def _simular_orden(self, linea: str):
        """La placa de mentira: contesta lo mismo que main.py y mueve los motores
        a la velocidad que pide el vector."""
        # Como main.py: '?', 'l', 'h' y 'fijar' no paran lo que se mueve
        self.sim_pos = self._sim_ahora()
        if self.pos_motor is None:
            self.pos_motor = list(self.sim_pos)
        texto = ' '.join(f'{v:.2f}' for v in self.sim_pos)
        if linea == '?':
            self._sim_salida.append('POS ' + texto)
        elif linea == 'l':
            lim = self.cal.limites
            self._sim_salida.append('LIMITES ' + ' '.join(
                f'{n} {lo:g} {hi:g}' for n, (lo, hi) in zip(NOMBRES, lim)))
        elif linea == 'q':
            self.sim_mov = [None, None, None]
            self.sim_listo = False
            self._sim_salida.append('PARADO ' + texto)
        elif linea.split()[0].lower() in ('h', 'fijar'):
            # Como main.py: h <motor> (a Alvin no) o fijar <motor> <pos>
            p = linea.lower().split()
            i = [n.lower() for n in NOMBRES].index(p[1]) if len(p) > 1 and \
                p[1] in [n.lower() for n in NOMBRES] else None
            if any(m is not None for m in self.sim_mov):
                self._sim_salida.append('[✗] h y fijar, solo con los tres parados: manda q antes')
            elif i is None or len(p) != (2 if p[0] == 'h' else 3):
                self._sim_salida.append('[✗] uso: h <motor> | fijar <motor> <pos>')
            elif p[0] == 'h' and i == 0:
                self._sim_salida.append('[✗] a Alvin no se le da la h (NO DARLE A LA H)')
            else:
                try:
                    self.sim_pos[i] = 0.0 if p[0] == 'h' else float(p[2])
                except ValueError:
                    self._sim_salida.append('[✗] ' + p[2] + ' no es un numero')
                    return
                self._sim_salida.append(f'[🏠] {NOMBRES[i]}: la posición actual ahora es '
                                        f'{self.sim_pos[i]:.2f}{A.UNIDADES[i].strip()}.')
        elif linea.startswith('['):
            # Como main.py: checksum, eco, RECIBE de cada nodo y sus limites
            estado = 'sin'
            if '*' in linea:
                cuerpo, llegado = (t.strip() for t in linea.rsplit('*', 1))
                if checksum(cuerpo) != llegado.upper():
                    self._sim_salida.append('ERROR vector checksum: no cuadra')
                    return
                linea, estado = cuerpo, 'ok'
            v = numeros(linea)
            self._sim_salida.append('VECTOR [' + ', '.join(f'{x:g}' for x in v)
                                    + '] checksum ' + estado)
            for i, nombre in enumerate(NOMBRES):
                self._sim_salida.append(f'RECIBE {nombre} pos {v[2 * i]:.3f} '
                                        f'vel {v[2 * i + 1]:g} esta {self.sim_pos[i]:.2f}')
            fuera = [f'{n} pos {v[2 * i]:.2f} fuera de su recorrido [{lo:g}, {hi:g}]'
                     for i, (n, (lo, hi)) in enumerate(zip(NOMBRES, self.cal.limites))
                     if v[2 * i + 1] > 0 and not lo <= v[2 * i] <= hi]
            if fuera:
                self._sim_salida += [f'ERROR {f}' for f in fuera] + ['RECHAZADO ' + texto]
                return
            ahora = time.monotonic()
            self.sim_mov = [None, None, None]
            for i in range(3):
                if v[2 * i + 1] > 0 and abs(v[2 * i] - self.sim_pos[i]) >= 0.5:
                    self.sim_mov[i] = (self.sim_pos[i], v[2 * i], v[2 * i + 1], ahora)
            self.sim_listo = True

    def _simular_lineas(self) -> list[str]:
        salida, self._sim_salida = self._sim_salida, []
        pos = self._sim_ahora()
        ahora = time.monotonic()
        for i, mov in enumerate(self.sim_mov):
            if mov is not None and pos[i] == mov[1]:
                self.sim_pos[i], self.sim_mov[i] = mov[1], None
                u = A.UNIDADES[i].strip()
                salida.append(f'[!] {NOMBRES[i]}: OBJETIVO ALCANZADO en {mov[1]:.2f}{u} '
                              f'(meta {mov[1]:.2f}{u}, error +0.00{u})')
        if any(m is not None for m in self.sim_mov) and ahora - self.sim_t_tel >= 0.1:
            self.sim_t_tel = ahora
            rpm = [0.0 if m is None else math.copysign(m[2], m[1] - m[0])
                   for m in self.sim_mov]
            salida.append(f'T: {ahora - self.sim_t0:.2f}s | ' + ' | '.join(
                f'{n} {p:.1f}{u.strip()} {r:.1f}{uv.lower()}'
                for n, p, r, u, uv in zip(NOMBRES, pos, rpm, A.UNIDADES, A.VEL_UNIDADES)))
        if self.sim_listo and all(m is None for m in self.sim_mov):
            self.sim_listo = False
            salida.append('LISTO ' + ' '.join(f'{v:.2f}' for v in self.sim_pos))
        return salida


def main(args=None):
    rclpy.init(args=args)
    node = Vector()
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
                node.ser.write(b'q\r\n')     # que no se quede nada moviendose
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
