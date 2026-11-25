#!/usr/bin/env python3
"""
TRAYECTORIA RECTILINEA RESUELTA POR CINEMATICA INVERSA.

Se eligen DOS puntos alcanzables del espacio de trabajo, A y B.  El segmento
recto A-B se subdivide en N puntos (N >= 10) y para CADA uno se resuelve la
cinematica INVERSA, que es la que entrega la terna de angulos que el robot debe
adoptar.  Esas consignas se mandan a Gazebo y, con el estado REAL que devuelve
la fisica, se vuelve a aplicar la cinematica DIRECTA para saber donde quedo de
verdad la punta del gripper.  De ahi sale la tabla de comprobacion.

    punto pedido  --CI-->  q1 q2 q3  --Gazebo-->  q medido  --CD-->  punto real

Entre un punto de paso y el siguiente NO se interpola en angulos sino en el
espacio cartesiano (parametro interp:=cartesiano, por defecto): se recorre el
segmento con perfil quintico y se resuelve la CI a 50 Hz, de modo que el TCP
describe una RECTA de verdad.  Con interp:=articular se interpolan los angulos,
que es mas barato pero comba la trayectoria; sirve para verlo en la traza.

    ros2 launch planta_brazo trayectoria_lineal.launch.py
    ros2 launch planta_brazo trayectoria_lineal.launch.py n_puntos:=15
    ros2 launch planta_brazo trayectoria_lineal.launch.py p_ini:="300,-150,250" p_fin:="300,150,250"

Parametros:
    p_ini, p_fin  extremos del segmento, en MILIMETROS, "x,y,z"
    n_puntos      subdivisiones del segmento          (def. 12, minimo 10)
    interp        'cartesiano' (recta real) | 'articular'
    modo          'gazebo' (con fisica) | 'rviz' (cinematico puro)
    t_aprox       segundos para ir de home al punto A (def. 3.0)
    t_tramo       segundos de cada tramo entre puntos (def. 1.6)
    t_asentar     reposo antes de medir               (def. 2.5)
    t_medir       segundos promediando la medida      (def. 0.4)
    preferencia   postura que pide la CI: codo_arriba | codo_abajo | cercana
    extremo       'tcp' (punta de los dedos) | 'brida'
    salida        ruta base de los ficheros de resultados
"""
import csv
import math
import os
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cinematica as cin
from cinematica import Cinematica
from pares import Pares, kgfcm
from enlace_juntas import EnlaceJuntas
from trayectorias import quintica

HZ = 50.0
MM = 1000.0


def leer_punto(txt, defecto):
    """'300,-150,250' (mm) -> (0.300, -0.150, 0.250) (m)."""
    try:
        v = [float(s) for s in str(txt).replace(';', ',').split(',')]
        return tuple(x / MM for x in v) if len(v) == 3 else defecto
    except ValueError:
        return defecto


def dist_a_recta(p, a, b):
    """Distancia perpendicular de p al segmento infinito que pasa por a y b."""
    ab = [j - i for i, j in zip(a, b)]
    ap = [j - i for i, j in zip(a, p)]
    n2 = sum(c * c for c in ab)
    if n2 < 1e-12:
        return math.dist(p, a)
    t = sum(i * j for i, j in zip(ap, ab)) / n2
    pie = [i + t * j for i, j in zip(a, ab)]
    return math.dist(p, pie)


class TrayectoriaLineal(Node):
    def __init__(self):
        super().__init__('trayectoria_lineal')
        # los valores por defecto salen de cinematica.py, que es la UNICA
        # fuente de la geometria: antes estaban copiados a mano en cada nodo
        # y se quedaron con las medidas viejas del CAD.
        for n, v in (('d1', cin.D1), ('a1', cin.A1), ('L2', cin.L2),
                     ('L3', cin.L3), ('Lg', cin.LG)):
            self.declare_parameter(n, v)
        self.declare_parameter('p_ini', '300,-150,250')
        self.declare_parameter('p_fin', '300,150,250')
        self.declare_parameter('n_puntos', 12)
        self.declare_parameter('interp', 'cartesiano')
        self.declare_parameter('modo', 'gazebo')
        self.declare_parameter('t_aprox', 3.0)
        self.declare_parameter('t_tramo', 1.6)
        self.declare_parameter('t_asentar', 2.5)
        self.declare_parameter('t_medir', 0.4)
        self.declare_parameter('preferencia', 'codo_arriba')
        self.declare_parameter('extremo', 'tcp')
        self.declare_parameter('salida', '~/trayectoria_lineal')
        self.declare_parameter('marcadores_gz', True)
        self.declare_parameter('registro', True)
        self.declare_parameter('bucle', False)
        self.declare_parameter('t_barrido', 4.0)

        g = lambda n: self.get_parameter(n).value
        self.cin = Cinematica(g('d1'), g('a1'), g('L2'), g('L3'), g('Lg'))
        self.A = leer_punto(g('p_ini'), (0.300, -0.150, 0.250))
        self.B = leer_punto(g('p_fin'), (0.300, 0.150, 0.250))
        self.N = max(10, int(g('n_puntos')))
        self.interp = str(g('interp')).lower()
        self.modo = str(g('modo')).lower()
        self.t_aprox = float(g('t_aprox'))
        self.t_tramo = float(g('t_tramo'))
        self.t_asentar = float(g('t_asentar'))
        self.t_medir = float(g('t_medir'))
        self.preferencia = str(g('preferencia'))
        self.extremo = str(g('extremo')).lower()
        self.salida = os.path.expanduser(str(g('salida')))
        self.marcar_gz = bool(g('marcadores_gz')) and self.modo == 'gazebo'
        self._arrancar_marcadores()
        self.registrar = bool(g('registro'))
        self.bucle = bool(g('bucle'))
        self.t_barrido = float(g('t_barrido'))

        self.enlace = EnlaceJuntas(self, self.modo)
        self.pub_obj = self.create_publisher(Marker, 'trayectoria_objetivo', 2)
        self.pub_real = self.create_publisher(Marker, 'trayectoria_real', 2)

        self.par = Pares(self.cin)
        self.q = [0.0, 0.0, 0.0]
        self.qd = [0.0, 0.0, 0.0]
        self.tau_pico = [0.0, 0.0, 0.0]
        self.tau_pico_q = [None, None, None]
        self.qd_pico = [0.0, 0.0, 0.0]
        self.pot_pico = [0.0, 0.0, 0.0]
        self.carga_pico = [0.0, 0.0, 0.0]
        self.serie = []
        self.t_serie = 0.0
        self.p_ant = None
        self.muestras = []
        self.desvio_max = 0.0
        self.terminado = False

        if not self.planificar():
            self.terminado = True
            self.create_timer(0.5, lambda: rclpy.shutdown())
            return

        self.pasos = self._guion()
        self.paso = 0
        self.t = 0.0
        self.arrancado = False
        self.en_bucle = False
        self.vueltas = 0
        self.create_timer(1.0 / HZ, self.tic)
        self.create_timer(1.0, self.publicar_marcadores)

    # ------------------------------------------------------------ planificacion
    def planificar(self):
        """Subdivide el segmento y resuelve la CI en cada punto de paso."""
        self.filas = []
        rmin, rmax = self.cin.alcance(self.extremo)
        self.get_logger().info(
            'segmento A=[%.1f %.1f %.1f] -> B=[%.1f %.1f %.1f] mm, '
            'largo %.1f mm, %d puntos (alcance del hombro %.1f a %.1f mm)'
            % (*[v * MM for v in self.A], *[v * MM for v in self.B],
               math.dist(self.A, self.B) * MM, self.N, rmin * MM, rmax * MM))

        q_prev = [0.0, 0.0, 0.0]
        malos = []
        for k in range(self.N):
            s = k / (self.N - 1.0)
            p = tuple(a + (b - a) * s for a, b in zip(self.A, self.B))
            sol, todas = self.cin.resolver(p, self.extremo, self.preferencia,
                                           q_actual=q_prev)
            if sol is None:
                malos.append((k, p, todas))
            else:
                q_prev = list(sol['q'])
            self.filas.append({
                'k': k + 1, 's': s, 'p_obj': p, 'sol': sol,
                'n_sols': len(todas),
                'n_validas': sum(t['valida'] for t in todas),
                'residuo': sol['residuo'] if sol else None,
            })

        if malos:
            for k, p, todas in malos:
                razon = ('fuera del alcance' if not todas else
                         'todas las soluciones se salen de los limites: '
                         + ', '.join(sorted({j for t in todas for j in t['fuera']})))
                self.get_logger().error(
                    'punto %d [%.1f %.1f %.1f] mm SIN SOLUCION: %s'
                    % (k + 1, *[v * MM for v in p], razon))
            self.get_logger().error(
                'elige otro par de puntos: el segmento no es viable completo')
            return False

        self.tabla_plan()
        return True

    def tabla_plan(self):
        deg = math.degrees
        linea = '-' * 104
        print('\n' + linea)
        print('PLAN: CINEMATICA INVERSA DE LA RECTA  (%d puntos, postura pedida '
              '%s, extremo %s)' % (self.N, self.preferencia, self.extremo))
        print(linea)
        print('%3s %8s %26s | %26s | %-22s %s'
              % ('k', 's', 'punto pedido X Y Z [mm]', 'CI -> J1 J2 J3 [deg]',
                 'postura', 'sols (validas)'))
        print(linea)
        for f in self.filas:
            p, sol = f['p_obj'], f['sol']
            print('%3d %8.3f %26s | %26s | %-22s %d (%d)'
                  % (f['k'], f['s'],
                     '%8.1f%9.1f%9.1f' % tuple(v * MM for v in p),
                     '%8.2f%9.2f%9.2f' % tuple(deg(v) for v in sol['q']),
                     sol['config'], f['n_sols'], f['n_validas']))
        print(linea + '\n')

    # ------------------------------------------------------------------- guion
    def _guion(self):
        """('aprox'|'tramo'|'esperar'|'medir'|'volver'|'fin', dato, duracion)."""
        pasos = [('aprox', 0, self.t_aprox), ('esperar', None, self.t_asentar),
                 ('medir', 0, self.t_medir)]
        for k in range(1, self.N):
            pasos += [('tramo', k, self.t_tramo),
                      ('esperar', None, self.t_asentar),
                      ('medir', k, self.t_medir)]
        pasos += [('volver', None, self.t_aprox), ('fin', None, 0.0)]
        return pasos

    def _guion_bucle(self):
        """Barrido continuo A<->B, sin paradas de medida: para grabar.

        El paso 0 (volver a la recta) solo se ejecuta la primera vez; despues
        el indice da la vuelta al 1 y quedan solo los dos barridos."""
        return [('aprox', self.N - 1, self.t_aprox),
                ('barrido', (self.N - 1, 0), self.t_barrido),
                ('barrido', (0, self.N - 1), self.t_barrido)]

    # ------------------------------------------------------------------- bucle
    def tic(self):
        if self.terminado:
            return
        if not self.arrancado:
            if self.modo == 'gazebo' and not self.enlace.hay_medida:
                return
            self.arrancado = True
            self.q_ini = list(self.enlace.real)
            self.publicar_marcadores()
            self.dibujar_recta_gz()
            self.get_logger().info('robot listo, empieza el recorrido')

        clase, dato, dur = self.pasos[self.paso]
        self.fase = clase
        s = quintica(self.t / dur if dur > 0 else 1.0)

        if clase == 'fin':
            self.informe()
            if not self.bucle:
                self.terminado = True
                return
            self.get_logger().info(
                'modo bucle: barriendo A<->B en continuo, Ctrl+C para parar')
            self.en_bucle = True
            self.pasos = self._guion_bucle()
            self.paso = 0
            self.t = 0.0
            self.q_ini = list(self.q)
            return

        if clase == 'aprox':
            # de donde este hasta el primer punto de la recta, en angulos
            self.q = [a + (b - a) * s
                      for a, b in zip(self.q_ini, self.filas[0]['sol']['q'])]
        elif clase == 'volver':
            self.q = [a * (1.0 - s) for a in self.q_ini]
        elif clase == 'tramo':
            a = self.filas[dato - 1]
            b = self.filas[dato]
            if self.interp == 'articular':
                self.q = [i + (j - i) * s
                          for i, j in zip(a['sol']['q'], b['sol']['q'])]
            else:
                # RECTA de verdad: se interpola el PUNTO y se resuelve la CI
                p = tuple(i + (j - i) * s
                          for i, j in zip(a['p_obj'], b['p_obj']))
                sol, _ = self.cin.resolver(p, self.extremo, 'cercana',
                                           q_actual=self.q)
                if sol is not None:
                    self.q = list(sol['q'])
            self.vigilar_desvio()
        elif clase == 'barrido':
            ka, kb = dato
            p = tuple(i + (j - i) * s
                      for i, j in zip(self.filas[ka]['p_obj'],
                                      self.filas[kb]['p_obj']))
            sol, _ = self.cin.resolver(p, self.extremo, 'cercana',
                                       q_actual=self.q)
            if sol is not None:
                self.q = list(sol['q'])
            self.vigilar_desvio()
        elif clase == 'medir':
            self.muestras.append(list(self.enlace.real))

        self.enlace.enviar(self.q)
        self.vigilar_par()
        self.t += 1.0 / HZ

        if self.t >= dur:
            if clase == 'medir':
                self.guardar(dato)
            elif clase in ('aprox', 'tramo'):
                # clavar el punto de paso exacto: el ultimo tic del perfil cae
                # en s < 1 y dejaria unas centesimas de milimetro de sobra
                self.q = list(self.filas[dato]['sol']['q'])
                self.enlace.enviar(self.q)
            self.t = 0.0
            self.q_ini = list(self.q)
            self.paso += 1
            if self.en_bucle and self.paso >= len(self.pasos):
                self.paso = 1              # se salta el acercamiento inicial
                self.vueltas += 1
                self.get_logger().info('barrido %d completado' % self.vueltas)

    def vigilar_par(self):
        """Par instantaneo que pide la maniobra, con la q REAL de la fisica y
        las derivadas de la consigna (diferencias finitas a 50 Hz)."""
        q = self.enlace.real
        qd = [(a - b) * HZ for a, b in zip(self.q, self.q_ant)] \
            if hasattr(self, 'q_ant') else [0.0, 0.0, 0.0]
        qdd = [(a - b) * HZ for a, b in zip(qd, self.qd)]
        self.q_ant = list(self.q)
        self.qd = qd
        d = self.par.total(q, qd, qdd)
        t = d['total']
        for i in range(3):
            if abs(t[i]) > abs(self.tau_pico[i]):
                self.tau_pico[i] = t[i]
                self.tau_pico_q[i] = list(q)
            self.qd_pico[i] = max(self.qd_pico[i], abs(qd[i]))
            self.pot_pico[i] = max(self.pot_pico[i], abs(t[i] * qd[i]))
            self.carga_pico[i] = max(self.carga_pico[i],
                                     self.par.carga(t[i], qd[i]))
        self._registrar(q, qd, qdd, d)

    def _registrar(self, q, qd, qdd, d):
        """Una fila por tic con todo lo que hace falta para las graficas.

        Solo durante la pasada de medida: en modo bucle el fichero creceria
        sin limite y ya no aporta nada nuevo."""
        if not self.registrar or self.en_bucle:
            return
        p = self.cin.directa(q, self.extremo)
        v = (0.0 if self.p_ant is None
             else math.dist(p, self.p_ant) * HZ)
        self.p_ant = p
        self.serie.append({
            't': self.t_serie, 'fase': self.fase,
            'q_cmd': list(self.q), 'q_med': list(q), 'qd': list(qd),
            'qdd': list(qdd), 'p': p, 'v': v,
            'carga': [self.par.carga(a, b) for a, b in zip(d['total'], qd)],
            'tau': list(d['total']), 'tau_g': list(d['gravedad']),
            'tau_i': list(d['inercia']), 'tau_f': list(d['friccion']),
        })
        self.t_serie += 1.0 / HZ

    def vigilar_desvio(self):
        """Cuanto se aparta de la recta el TCP real mientras va en marcha."""
        p = self.cin.directa(self.enlace.real, self.extremo)
        self.desvio_max = max(self.desvio_max, dist_a_recta(p, self.A, self.B))

    # ----------------------------------------------------------------- medidas
    def guardar(self, k):
        n = max(len(self.muestras), 1)
        q_med = [sum(m[i] for m in self.muestras) / n for i in range(3)]
        self.muestras = []
        f = self.filas[k]
        p = self.cin.directa(q_med, self.extremo)
        f['q_med'] = q_med
        f['p_real'] = p
        f['error'] = math.dist(p, f['p_obj'])
        f['desvio'] = dist_a_recta(p, self.A, self.B)
        f['tau'] = self.par.total(q_med)['total']
        self.get_logger().info(
            'punto %2d/%d  pedido [%7.1f %7.1f %7.1f]  real [%7.1f %7.1f %7.1f] mm'
            '   error %5.2f mm'
            % (f['k'], self.N, *[v * MM for v in f['p_obj']],
               *[v * MM for v in p], f['error'] * MM))
        self.publicar_marcadores()
        self.esfera_gz(200 + f['k'], p, (1.0, 0.35, 0.0), 0.016)

    # ------------------------------------------------------ marcadores de gz
    def _arrancar_marcadores(self):
        """Hilo unico que atiende las peticiones al servicio /marker.

        Tienen que ir EN ORDEN y de una en una: gz service es un proceso
        aparte, y si se lanzan varios a la vez el DELETE_ALL inicial puede
        adelantar a los ADD y borrarlos nada mas dibujarlos."""
        self.cola_gz = queue.Queue()
        self._hilo_gz = threading.Thread(target=self._servir_marcadores,
                                         daemon=True)
        self._hilo_gz.start()

    def _servir_marcadores(self):
        while True:
            req = self.cola_gz.get()
            if req is None:
                return
            try:
                subprocess.run(
                    ['gz', 'service', '-s', '/marker',
                     '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Empty',
                     '--timeout', '2000', '--req', req],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=5.0, check=False)
            except (OSError, ValueError, subprocess.SubprocessError) as e:
                self.marcar_gz = False
                self.get_logger().warn('sin marcadores en Gazebo (%s)' % e)
                return

    def _gz_marker(self, req):
        """Encola un dibujo para Gazebo sin bloquear el bucle de 50 Hz."""
        if self.marcar_gz:
            self.cola_gz.put(req)

    @staticmethod
    def _mat(c):
        r, g, b = c
        return ('material: {ambient: {r: %.2f, g: %.2f, b: %.2f, a: 1}, '
                'diffuse: {r: %.2f, g: %.2f, b: %.2f, a: 1}}' % (r, g, b, r, g, b))

    def esfera_gz(self, ident, punto, color, d):
        self._gz_marker(
            'action: ADD_MODIFY, type: SPHERE, id: %d, ns: "trayectoria", '
            'scale: {x: %.4f, y: %.4f, z: %.4f}, '
            'pose: {position: {x: %.5f, y: %.5f, z: %.5f}, '
            'orientation: {w: 1}}, %s'
            % (ident, d, d, d, punto[0], punto[1], punto[2], self._mat(color)))

    def dibujar_recta_gz(self):
        """La recta objetivo y los 12 puntos pedidos, pintados en el mundo."""
        if not self.marcar_gz:
            return
        self._gz_marker('action: DELETE_ALL, ns: "trayectoria"')
        pts = ', '.join('{x: %.5f, y: %.5f, z: %.5f}' % f['p_obj']
                        for f in self.filas)
        self._gz_marker(
            'action: ADD_MODIFY, type: LINE_STRIP, id: 1, ns: "trayectoria", '
            '%s, point: [%s]' % (self._mat((0.10, 0.90, 0.25)), pts))
        for f in self.filas:
            self.esfera_gz(100 + f['k'], f['p_obj'], (0.10, 0.90, 0.25), 0.010)
        self.get_logger().info(
            'recta y puntos objetivo dibujados en Gazebo '
            '(verde = pedido, naranja = alcanzado)')

    # -------------------------------------------------------------- marcadores
    def _marker(self, tipo, ident, color, ancho):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'trayectoria'
        m.id = ident
        m.type = tipo
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = ancho
        if tipo == Marker.LINE_STRIP:
            m.scale.y = m.scale.z = 0.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.pose.orientation.w = 1.0
        return m

    def publicar_marcadores(self):
        obj = self._marker(Marker.LINE_STRIP, 1, (0.1, 0.9, 0.2, 1.0), 0.004)
        obj.points = [Point(x=p[0], y=p[1], z=p[2])
                      for p in (f['p_obj'] for f in self.filas)]
        self.pub_obj.publish(obj)

        real = self._marker(Marker.SPHERE_LIST, 2, (1.0, 0.35, 0.0, 1.0), 0.012)
        real.points = [Point(x=f['p_real'][0], y=f['p_real'][1], z=f['p_real'][2])
                       for f in self.filas if f.get('p_real')]
        if real.points:
            self.pub_real.publish(real)

    # ----------------------------------------------------------------- informe
    def informe(self):
        deg = math.degrees
        linea = '-' * 118
        print('\n' + linea)
        print('TRAYECTORIA RECTILINEA POR CINEMATICA INVERSA   (modo %s, '
              'interpolacion %s, extremo %s)'
              % (self.modo, self.interp, self.extremo))
        print('A = [%.1f %.1f %.1f] mm      B = [%.1f %.1f %.1f] mm      '
              'largo %.1f mm en %d puntos'
              % (*[v * MM for v in self.A], *[v * MM for v in self.B],
                 math.dist(self.A, self.B) * MM, self.N))
        print(linea)
        print('%3s %26s | %26s | %26s %8s %8s | %15s'
              % ('k', 'pedido X Y Z [mm]', 'CI -> J1 J2 J3 [deg]',
                 'alcanzado X Y Z [mm]', 'error', 'desvio',
                 'par J2 J3 [N.m]'))
        print(linea)
        for f in self.filas:
            r = f.get('p_real')
            tau = f.get('tau')
            print('%3d %26s | %26s | %26s %8s %8s | %15s'
                  % (f['k'],
                     '%8.1f%9.1f%9.1f' % tuple(v * MM for v in f['p_obj']),
                     '%8.2f%9.2f%9.2f' % tuple(deg(v) for v in f['sol']['q']),
                     ('%8.1f%9.1f%9.1f' % tuple(v * MM for v in r)) if r
                     else ' ' * 26,
                     ('%8.2f' % (f['error'] * MM)) if r else '       -',
                     ('%8.2f' % (f['desvio'] * MM)) if r else '       -',
                     ('%7.3f%8.3f' % (tau[1], tau[2])) if tau else ' ' * 15))
        print(linea)

        e = [f['error'] for f in self.filas if f.get('error') is not None]
        d = [f['desvio'] for f in self.filas if f.get('desvio') is not None]
        if e:
            print('error de posicion   medio %5.2f mm   maximo %5.2f mm'
                  % (sum(e) / len(e) * MM, max(e) * MM))
            print('desvio de la recta  medio %5.2f mm   maximo %5.2f mm   '
                  '(en marcha, muestreado a 50 Hz: %5.2f mm)'
                  % (sum(d) / len(d) * MM, max(d) * MM, self.desvio_max * MM))
        print('residuo de la CI (|CD(CI(p)) - p|) maximo: %.6f mm'
              % (max(f['residuo'] for f in self.filas) * MM))
        print(linea + '\n')

        self.informe_par()
        self.informe_cinetico()
        self.escribir(e, d)
        if not self.bucle:
            rclpy.shutdown()

    def informe_cinetico(self):
        """Posicion, velocidad y par: los tres a la vez, que es como hay que
        mirarlos para decidir si un motor sirve."""
        m = self.par.motor
        w0 = self.par.w_vacio()
        lim = (3.5, 3.0, 3.0)          # <limit velocity> del URDF, rad/s
        linea = '-' * 118
        print(linea)
        print('ANALISIS DE VELOCIDAD, POSICION Y PAR')
        print('  velocidad de vacio del eje de salida: %.2f rad/s '
              '(%.0f rpm, %.0f deg/s)' % (w0, m['rpm_vacio'], math.degrees(w0)))
        print(linea)
        print('%5s %11s %9s %10s %10s %11s %10s %9s'
              % ('junta', 'qd max', 'rpm', '% vacio', '% lim URDF',
                 'tau max', 'P mec max', 'carga'))
        print('%5s %11s %9s %10s %10s %11s %10s %9s'
              % ('', '[rad/s]', '', '', '', '[N.m]', '[W]', ''))
        print(linea)
        for i, j in enumerate(('J1', 'J2', 'J3')):
            w = self.qd_pico[i]
            print('%5s %11.3f %9.1f %9.0f%% %9.0f%% %11.4f %10.2f %9.2f%s'
                  % (j, w, w * 60.0 / (2.0 * math.pi), w / w0 * 100.0,
                     w / lim[i] * 100.0, abs(self.tau_pico[i]),
                     self.pot_pico[i], self.carga_pico[i],
                     '' if self.carga_pico[i] <= 1.0 else '  FUERA'))
        print(linea)
        print('carga = tau/tau_bloqueo + |qd|/w_vacio  sobre la recta par-'
              'velocidad del motor; <= 1 es realizable')
        e = [f['error'] for f in self.filas if f.get('error') is not None]
        if e:
            print('posicion: error medio %.3f mm, maximo %.3f mm en los %d '
                  'puntos de paso' % (sum(e) / len(e) * MM, max(e) * MM,
                                      len(e)))
        if self.serie:
            v = max(r['v'] for r in self.serie)
            print('velocidad lineal del TCP: %.1f mm/s de pico '
                  '(%d muestras a %.0f Hz)' % (v * MM, len(self.serie), HZ))
        print(linea + chr(10))

    def informe_par(self):
        m = self.par.motor
        linea = '-' * 118
        print('COMPROBACION DE PAR contra el motor %s  (%.0f V, %.0f rpm, '
              'reduccion %.0f:1)' % (m['nombre'], m['tension'], m['rpm_vacio'],
                                     m['reduccion']))
        print('  par nominal  %.3f N.m (%.2f kgf.cm)      par de bloqueo  '
              '%.3f N.m (%.2f kgf.cm)'
              % (m['par_nominal'], kgfcm(m['par_nominal']),
                 m['par_bloqueo'], kgfcm(m['par_bloqueo'])))
        print(linea)
        print('%5s %12s %12s %12s %12s %10s   %s'
              % ('junta', 'pico [N.m]', 'pico[kgf.cm]', '% nominal',
                 '% bloqueo', 'estado', 'postura del pico J1 J2 J3 [deg]'))
        print(linea)
        for v, qp in zip(self.par.veredicto(self.tau_pico), self.tau_pico_q):
            print('%5s %12.4f %12.3f %11.0f%% %11.0f%% %10s   %s'
                  % (v['junta'], v['tau'], v['kgfcm'],
                     v['uso_nominal'] * 100, v['uso_bloqueo'] * 100,
                     v['estado'],
                     ('%7.2f%8.2f%8.2f' % tuple(math.degrees(a) for a in qp))
                     if qp else '-'))
        print(linea)
        peor = self.par.total((0.0, 0.0, 0.0))['total']
        print('referencia, peor caso del robot (brazo horizontal estirado, '
              'q2 = q3 = 0):  J2 %.3f N.m (%.1f kgf.cm)   J3 %.3f N.m '
              '(%.1f kgf.cm)' % (peor[1], kgfcm(peor[1]), peor[2],
                                 kgfcm(peor[2])))
        print(linea + chr(10))
        self.tau_peor = peor

    def escribir(self, e, d):
        deg = math.degrees
        with open(self.salida + '.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['k', 's', 'obj_x_mm', 'obj_y_mm', 'obj_z_mm',
                        'ci_J1_deg', 'ci_J2_deg', 'ci_J3_deg', 'postura',
                        'soluciones', 'soluciones_validas',
                        'med_J1_deg', 'med_J2_deg', 'med_J3_deg',
                        'real_x_mm', 'real_y_mm', 'real_z_mm',
                        'error_mm', 'desvio_recta_mm',
                        'tau_J2_Nm', 'tau_J3_Nm', 'tau_J2_kgfcm',
                        'tau_J3_kgfcm', 'pct_nominal_J2', 'pct_nominal_J3'])
            for f in self.filas:
                fila = ([f['k'], '%.4f' % f['s']]
                        + ['%.2f' % (v * MM) for v in f['p_obj']]
                        + ['%.3f' % deg(v) for v in f['sol']['q']]
                        + [f['sol']['config'], f['n_sols'], f['n_validas']])
                q, r = f.get('q_med'), f.get('p_real')
                fila += ['%.3f' % deg(v) for v in q] if q else ['', '', '']
                fila += ['%.2f' % (v * MM) for v in r] if r else ['', '', '']
                fila += ['%.3f' % (f['error'] * MM) if r else '',
                         '%.3f' % (f['desvio'] * MM) if r else '']
                tau = f.get('tau')
                nom = self.par.motor['par_nominal']
                fila += (['%.4f' % tau[1], '%.4f' % tau[2],
                          '%.3f' % kgfcm(tau[1]), '%.3f' % kgfcm(tau[2]),
                          '%.1f' % (abs(tau[1]) / nom * 100),
                          '%.1f' % (abs(tau[2]) / nom * 100)]
                         if tau else [''] * 6)
                w.writerow(fila)

        with open(self.salida + '.md', 'w') as fh:
            fh.write('# Trayectoria rectilinea por cinematica inversa\n\n')
            fh.write('Robot PLANTA BRAZO (3R), simulador **%s**, extremo **%s**, '
                     'interpolacion **%s**, postura pedida a la CI **%s**.\n\n'
                     % (self.modo, self.extremo, self.interp, self.preferencia))
            fh.write('- A = (%.1f, %.1f, %.1f) mm\n- B = (%.1f, %.1f, %.1f) mm\n'
                     '- Largo del segmento: %.1f mm, subdividido en %d puntos\n\n'
                     % (*[v * MM for v in self.A], *[v * MM for v in self.B],
                        math.dist(self.A, self.B) * MM, self.N))
            fh.write('| k | Punto pedido X, Y, Z [mm] | CI: J1, J2, J3 [deg] | '
                     'Postura | Sols. (validas) | Alcanzado X, Y, Z [mm] | '
                     'Error [mm] | Desvio de la recta [mm] |\n')
            fh.write('|---|---|---|---|---|---|---|---|\n')
            for f in self.filas:
                r = f.get('p_real')
                c = lambda p: '%.1f, %.1f, %.1f' % tuple(v * MM for v in p)
                fh.write('| %d | %s | %.2f, %.2f, %.2f | %s | %d (%d) | %s | %s'
                         ' | %s |\n'
                         % (f['k'], c(f['p_obj']),
                            *[deg(v) for v in f['sol']['q']],
                            f['sol']['config'], f['n_sols'], f['n_validas'],
                            c(r) if r else '-',
                            '%.2f' % (f['error'] * MM) if r else '-',
                            '%.2f' % (f['desvio'] * MM) if r else '-'))
            if e:
                fh.write('\n| | Medio | Maximo |\n|---|---|---|\n')
                fh.write('| Error de posicion [mm] | %.2f | %.2f |\n'
                         % (sum(e) / len(e) * MM, max(e) * MM))
                fh.write('| Desvio de la recta [mm] | %.2f | %.2f |\n'
                         % (sum(d) / len(d) * MM, max(d) * MM))
                fh.write('\nDesvio maximo medido en marcha (50 Hz): **%.2f mm**.'
                         '\n' % (self.desvio_max * MM))
            m = self.par.motor
            fh.write('\n## Par requerido y comprobacion del motor\n\n')
            fh.write('Motor **%s**: %.0f V, %.0f rpm, reduccion %.0f:1. '
                     'Par nominal **%.3f N.m** (%.2f kgf.cm), par de bloqueo '
                     '**%.3f N.m** (%.2f kgf.cm).\n\n'
                     % (m['nombre'], m['tension'], m['rpm_vacio'],
                        m['reduccion'], m['par_nominal'],
                        kgfcm(m['par_nominal']), m['par_bloqueo'],
                        kgfcm(m['par_bloqueo'])))
            fh.write('| Junta | Par pico [N.m] | [kgf.cm] | % del nominal | '
                     '% del bloqueo | Estado |\n|---|---|---|---|---|---|\n')
            for v in self.par.veredicto(self.tau_pico):
                fh.write('| %s | %.4f | %.3f | %.0f%% | %.0f%% | %s |\n'
                         % (v['junta'], v['tau'], v['kgfcm'],
                            v['uso_nominal'] * 100, v['uso_bloqueo'] * 100,
                            v['estado']))
            fh.write('\nPeor caso del robot (brazo horizontal estirado, '
                     'q2 = q3 = 0): J2 **%.3f N.m** (%.1f kgf.cm), J3 **%.3f '
                     'N.m** (%.1f kgf.cm).\n'
                     % (self.tau_peor[1], kgfcm(self.tau_peor[1]),
                        self.tau_peor[2], kgfcm(self.tau_peor[2])))
            fh.write('\nResiduo maximo de la cinematica inversa '
                     '(|CD(CI(p)) - p|): **%.6f mm** — la CI es exacta; todo el '
                     'error de la tabla lo pone el control de la simulacion.\n'
                     % (max(f['residuo'] for f in self.filas) * MM))
        self.escribir_serie()
        self.get_logger().info('resultados en %s.md y %s.csv'
                               % (self.salida, self.salida))

    def escribir_serie(self):
        """Serie temporal a 50 Hz: es la materia prima de las graficas de
        posicion, velocidad y par."""
        if not self.serie:
            return
        deg = math.degrees
        ruta = self.salida + '_series.csv'
        cab = ['t_s', 'fase']
        for b in ('qcmd_deg', 'qmed_deg', 'qd_deg_s', 'qdd_deg_s2',
                  'tau_Nm', 'tau_grav_Nm', 'tau_iner_Nm', 'tau_fric_Nm',
                  'carga'):
            cab += ['%s_J%d' % (b, i + 1) for i in range(3)]
        cab += ['tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm', 'tcp_v_mm_s']
        with open(ruta, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(cab)
            for r in self.serie:
                fila = ['%.3f' % r['t'], r['fase']]
                for k in ('q_cmd', 'q_med', 'qd', 'qdd'):
                    fila += ['%.5f' % deg(v) for v in r[k]]
                for k in ('tau', 'tau_g', 'tau_i', 'tau_f'):
                    fila += ['%.5f' % v for v in r[k]]
                fila += ['%.4f' % v for v in r['carga']]
                fila += ['%.3f' % (v * MM) for v in r['p']]
                fila += ['%.3f' % (r['v'] * MM)]
                w.writerow(fila)
        self.get_logger().info('serie temporal (%d muestras) en %s'
                               % (len(self.serie), ruta))


def main():
    rclpy.init()
    n = TrayectoriaLineal()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
