#!/usr/bin/env python3
# Nodo: recorre los puntos de prueba y compara cinematica directa e inversa.
import csv
import math
import os

import rclpy
from rclpy.node import Node

import cinematica as cin
from cinematica import Cinematica
from enlace_juntas import EnlaceJuntas
from puntos_prueba import PUNTOS_PRUEBA_RAD
from trayectorias import quintica

HZ = 50.0
MM = 1000.0


class Comparacion(Node):
    def __init__(self):
        super().__init__('comparacion_cinematicas')
        for n, v in (('d1', cin.D1), ('a1', cin.A1), ('L2', cin.L2),
                     ('L3', cin.L3), ('Lg', cin.LG)):
            self.declare_parameter(n, v)
        self.declare_parameter('modo', 'gazebo')
        self.declare_parameter('t_viaje', 2.5)
        self.declare_parameter('t_asentar', 3.0)
        self.declare_parameter('t_medir', 0.5)
        self.declare_parameter('preferencia', 'codo_arriba')
        self.declare_parameter('extremo', 'tcp')
        self.declare_parameter('salida', '~/comparacion_cinematicas')

        g = lambda n: self.get_parameter(n).value
        self.cin = Cinematica(g('d1'), g('a1'), g('L2'), g('L3'), g('Lg'))
        self.modo = str(g('modo')).lower()
        self.t_viaje = float(g('t_viaje'))
        self.t_asentar = float(g('t_asentar'))
        self.t_medir = float(g('t_medir'))
        self.preferencia = str(g('preferencia'))
        self.extremo = str(g('extremo')).lower()
        self.salida = os.path.expanduser(str(g('salida')))

        self.enlace = EnlaceJuntas(self, self.modo)
        self.q = [0.0, 0.0, 0.0]

        self.muestras = []
        self.filas = []
        self.pasos = self._guion()
        self.paso = 0
        self.t = 0.0
        self.arrancado = False
        self.terminado = False
        self.create_timer(1.0 / HZ, self.tic)

        self.get_logger().info(
            'comparacion CD/CI en modo %s: %d puntos, %s, extremo=%s'
            % (self.modo, len(PUNTOS_PRUEBA_RAD), self.preferencia, self.extremo))

    def _guion(self):
        home = (0.0, 0.0, 0.0)
        pasos = [('mover', home, self.t_viaje), ('esperar', None, self.t_asentar)]
        for i, (nombre, q_ref) in enumerate(PUNTOS_PRUEBA_RAD):
            p_obj = self.cin.directa(q_ref, self.extremo)
            sol, todas = self.cin.resolver(p_obj, self.extremo,
                                           self.preferencia, q_actual=home)
            fila = {'i': i + 1, 'nombre': nombre, 'q_ref': q_ref, 'p_obj': p_obj,
                    'sol': sol, 'n_sols': len(todas),
                    'n_validas': sum(s['valida'] for s in todas)}
            self.filas.append(fila)

            pasos += [('mover', q_ref, self.t_viaje),
                      ('esperar', None, self.t_asentar),
                      ('medir', (i, 'dir'), self.t_medir),
                      ('mover', home, self.t_viaje),
                      ('esperar', None, self.t_asentar)]

            if sol is not None:
                pasos += [('mover', sol['q'], self.t_viaje),
                          ('esperar', None, self.t_asentar),
                          ('medir', (i, 'inv'), self.t_medir),
                          ('mover', home, self.t_viaje),
                          ('esperar', None, self.t_asentar)]
        pasos.append(('fin', None, 0.0))
        return pasos

    def tic(self):
        if self.terminado:
            return
        if not self.arrancado:
            if self.modo == 'gazebo' and not self.enlace.hay_medida:
                return
            self.arrancado = True
            self.q_ini = list(self.enlace.real)
            self.get_logger().info('robot listo, empieza el recorrido')

        clase, dato, dur = self.pasos[self.paso]

        if clase == 'fin':
            self.terminado = True
            self.informe()
            return

        if clase == 'mover':
            s = quintica(self.t / dur if dur > 0 else 1.0)
            self.q = [a + (b - a) * s for a, b in zip(self.q_ini, dato)]
        elif clase == 'medir':
            self.muestras.append(list(self.enlace.real))

        self.enlace.enviar(self.q)
        self.t += 1.0 / HZ

        if self.t >= dur:
            if clase == 'medir':
                self.guardar(dato)
            self.t = 0.0
            self.q_ini = list(self.q)
            self.paso += 1
            if self.pasos[self.paso][0] == 'mover':
                self.q_ini = list(self.q)

    def guardar(self, dato):
        i, via = dato
        n = max(len(self.muestras), 1)
        q_med = [sum(m[k] for m in self.muestras) / n for k in range(3)]
        self.muestras = []
        fila = self.filas[i]
        p = self.cin.directa(q_med, self.extremo)
        fila[f'q_{via}'] = q_med
        fila[f'p_{via}'] = p
        fila[f'e_{via}'] = math.dist(p, fila['p_obj'])
        self.get_logger().info(
            '%-26s %s  ->  [%7.1f %7.1f %7.1f] mm   error %5.2f mm'
            % (fila['nombre'], via.upper(), p[0] * MM, p[1] * MM, p[2] * MM,
               fila[f'e_{via}'] * MM))

    def informe(self):
        deg = math.degrees
        cab = ('punto', 'objetivo X  Y  Z [mm]', 'DIRECTA X  Y  Z [mm]', 'err',
               'INVERSA X  Y  Z [mm]', 'err')
        linea = '-' * 118
        print('\n' + linea)
        print('COMPARACION CINEMATICA DIRECTA vs INVERSA   (modo %s, extremo %s,'
              ' postura pedida a la CI: %s)' % (self.modo, self.extremo,
                                               self.preferencia))
        print(linea)
        print('%-26s %26s | %26s %7s | %26s %7s' % cab)
        print(linea)
        for f in self.filas:
            o = f['p_obj']
            d = f.get('p_dir'); iv = f.get('p_inv')
            fmt = lambda p: ('%8.1f%9.1f%9.1f' % (p[0] * MM, p[1] * MM, p[2] * MM)
                             if p else ' ' * 26)
            err = lambda e: ('%7.2f' % (e * MM)) if e is not None else '      -'
            print('%-26s %26s | %26s %7s | %26s %7s'
                  % (f['nombre'], fmt(o), fmt(d), err(f.get('e_dir')),
                     fmt(iv), err(f.get('e_inv'))))
        print(linea)

        print('\nANGULOS  (q_ref: los que se le dan a la DIRECTA;  '
              'q_ci: los que calcula la INVERSA a partir del punto)')
        print(linea)
        print('%-26s %24s | %24s | %-28s %s'
              % ('punto', 'q_ref J1 J2 J3 [deg]', 'q_ci  J1 J2 J3 [deg]',
                 'postura elegida por la CI', 'soluciones'))
        print(linea)
        for f in self.filas:
            sol = f['sol']
            qc = ('%8.2f%8.2f%8.2f' % tuple(deg(v) for v in sol['q'])
                  if sol else '   sin solucion valida  ')
            print('%-26s %24s | %24s | %-28s %d (%d validas)'
                  % (f['nombre'],
                     '%8.2f%8.2f%8.2f' % tuple(deg(v) for v in f['q_ref']),
                     qc, sol['config'] if sol else '-',
                     f['n_sols'], f['n_validas']))
        print(linea)

        e_d = [f['e_dir'] for f in self.filas if f.get('e_dir') is not None]
        e_i = [f['e_inv'] for f in self.filas if f.get('e_inv') is not None]
        if e_d and e_i:
            print('error medio   DIRECTA %6.2f mm    INVERSA %6.2f mm'
                  % (sum(e_d) / len(e_d) * MM, sum(e_i) / len(e_i) * MM))
            print('error maximo  DIRECTA %6.2f mm    INVERSA %6.2f mm'
                  % (max(e_d) * MM, max(e_i) * MM))
        print(linea + '\n')

        self.escribir_ficheros(e_d, e_i)
        rclpy.shutdown()

    def escribir_ficheros(self, e_d, e_i):
        deg = math.degrees
        with open(self.salida + '.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['punto',
                        'obj_x_mm', 'obj_y_mm', 'obj_z_mm',
                        'dir_x_mm', 'dir_y_mm', 'dir_z_mm', 'dir_error_mm',
                        'inv_x_mm', 'inv_y_mm', 'inv_z_mm', 'inv_error_mm',
                        'qref_J1', 'qref_J2', 'qref_J3',
                        'qci_J1', 'qci_J2', 'qci_J3', 'postura_ci',
                        'soluciones', 'soluciones_validas'])
            for f in self.filas:
                sol = f['sol']
                fila = [f['nombre']]
                for k in ('p_obj', 'p_dir', 'p_inv'):
                    p = f.get(k)
                    fila += ['%.2f' % (v * MM) for v in p] if p else ['', '', '']
                    if k != 'p_obj':
                        e = f.get('e_' + k[2:5])
                        fila += ['%.3f' % (e * MM)] if e is not None else ['']
                fila += ['%.3f' % deg(v) for v in f['q_ref']]
                fila += (['%.3f' % deg(v) for v in sol['q']] + [sol['config']]
                         if sol else ['', '', '', 'sin solucion'])
                fila += [f['n_sols'], f['n_validas']]
                w.writerow(fila)

        with open(self.salida + '.md', 'w') as fh:
            fh.write('# Comparacion cinematica directa vs inversa\n\n')
            fh.write('Robot PLANTA BRAZO (3R), simulador **%s**, punto de '
                     'interes **%s**, postura pedida a la CI **%s**.\n\n'
                     % (self.modo, self.extremo, self.preferencia))
            fh.write('| Punto | Objetivo X, Y, Z [mm] | Alcanzado por la DIRECTA'
                     ' [mm] | Error [mm] | Alcanzado por la INVERSA [mm] | '
                     'Error [mm] |\n')
            fh.write('|---|---|---|---|---|---|\n')
            for f in self.filas:
                c = lambda p: ('%.1f, %.1f, %.1f' % tuple(v * MM for v in p)
                               if p else '-')
                e = lambda v: '%.2f' % (v * MM) if v is not None else '-'
                fh.write('| %s | %s | %s | %s | %s | %s |\n'
                         % (f['nombre'], c(f['p_obj']), c(f.get('p_dir')),
                            e(f.get('e_dir')), c(f.get('p_inv')),
                            e(f.get('e_inv'))))
            fh.write('\n| Punto | q_ref (entra a la DIRECTA) [deg] | q_ci '
                     '(sale de la INVERSA) [deg] | Postura elegida | '
                     'Soluciones (validas) |\n|---|---|---|---|---|\n')
            for f in self.filas:
                sol = f['sol']
                fh.write('| %s | %.2f, %.2f, %.2f | %s | %s | %d (%d) |\n'
                         % (f['nombre'], *[deg(v) for v in f['q_ref']],
                            ('%.2f, %.2f, %.2f' % tuple(deg(v) for v in sol['q'])
                             if sol else 'sin solucion'),
                            sol['config'] if sol else '-',
                            f['n_sols'], f['n_validas']))
            if e_d and e_i:
                fh.write('\n| | Directa | Inversa |\n|---|---|---|\n')
                fh.write('| Error medio [mm] | %.2f | %.2f |\n'
                         % (sum(e_d) / len(e_d) * MM, sum(e_i) / len(e_i) * MM))
                fh.write('| Error maximo [mm] | %.2f | %.2f |\n'
                         % (max(e_d) * MM, max(e_i) * MM))
        self.get_logger().info('resultados en %s.md y %s.csv'
                               % (self.salida, self.salida))


def main():
    rclpy.init()
    n = Comparacion()
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
