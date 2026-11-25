#!/usr/bin/env python3
# Nodo: control cinematico por jacobiano inverso.
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

import jacobiano as jac
from cinematica import CINEMATICA, HOME
from enlace_juntas import EnlaceJuntas


def recta(s, A, B):
    A, B = np.asarray(A, float), np.asarray(B, float)
    return A + s * (B - A), (B - A)


def circulo(s, centro, radio, plano='xz'):
    c = np.asarray(centro, float)
    a = 2.0 * math.pi * s
    if plano == 'xy':
        p = c + np.array([radio * math.cos(a), radio * math.sin(a), 0.0])
        d = 2 * math.pi * np.array([-radio * math.sin(a), radio * math.cos(a), 0.0])
    else:
        p = c + np.array([radio * math.cos(a), 0.0, radio * math.sin(a)])
        d = 2 * math.pi * np.array([-radio * math.sin(a), 0.0, radio * math.cos(a)])
    return p, d


def poligonal(s, pts, largos, total):
    d = s * total
    for i in range(len(largos)):
        if d <= largos[i] or i == len(largos) - 1:
            d0 = largos[i - 1] if i else 0.0
            u = (d - d0) / (largos[i] - d0) if largos[i] > d0 else 0.0
            u = min(max(u, 0.0), 1.0)
            seg = pts[i + 1] - pts[i]
            n = np.linalg.norm(seg)
            return pts[i] + u * seg, (seg / n * total if n > 1e-9 else seg * 0.0)
    return pts[-1], np.zeros(3)


def perfil_quintico(u):
    return (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
            30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)


def perfil_cubico(u):
    return 3 * u ** 2 - 2 * u ** 3, 6 * u - 6 * u ** 2


def perfil_trapezoidal(u, fc=0.25):
    fc = min(max(fc, 1e-3), 0.5)
    vc = 1.0 / (1.0 - fc)
    if u < fc:
        return 0.5 * vc / fc * u * u, vc * u / fc
    if u <= 1.0 - fc:
        return 0.5 * vc * fc + vc * (u - fc), vc
    r = 1.0 - u
    return 1.0 - 0.5 * vc / fc * r * r, vc * r / fc

PERFILES = {'quintico': perfil_quintico,
            'cubico': perfil_cubico,
            'trapezoidal': perfil_trapezoidal}


class ControlCinematico(Node):
    def __init__(self):
        super().__init__('control_cinematico')
        p = self.declare_parameter
        p('modo', 'gazebo')
        p('figura', 'tarea')
        p('perfil', 'cubico')
        p('rampa', 0.25)
        p('p_ini', '250,150,250')
        p('p_fin', '250,150,150')
        p('centro', '300,0,250')
        p('radio', 80.0)
        p('duracion', 4.0)
        p('frecuencia', 100.0)
        p('ganancia', 5.0)
        p('amortiguacion', 0.0)
        p('t_home', 3.0)
        p('tol_home', 0.02)
        p('desvio_max', 0.35)
        p('realimentacion', 'medida')
        p('pasadas', 1)
        p('graficas', True)
        p('refresco', 10.0)
        p('guardar', False)
        p('salida', '~/control_cinematico')

        g = lambda n: self.get_parameter(n).value
        self.figura = str(g('figura')).lower()
        self.nombre_perfil = str(g('perfil')).lower()
        if self.nombre_perfil not in PERFILES:
            self.nombre_perfil = 'trapezoidal'
        self.rampa = float(g('rampa'))
        self.perfil = PERFILES[self.nombre_perfil]
        self.A = np.array([float(v) for v in str(g('p_ini')).split(',')]) / 1000.0
        self.B = np.array([float(v) for v in str(g('p_fin')).split(',')]) / 1000.0
        self.C = np.array([float(v) for v in str(g('centro')).split(',')]) / 1000.0
        self.radio = float(g('radio')) / 1000.0
        self.T = float(g('duracion'))
        self.f = float(g('frecuencia'))
        self.K = float(g('ganancia'))
        self.lam = float(g('amortiguacion'))
        self.pasadas = int(g('pasadas'))
        self.ver = bool(g('graficas'))
        self.refresco = float(g('refresco'))
        self.guardar = bool(g('guardar'))
        self.salida = os.path.expanduser(str(g('salida')))

        self.dt = 1.0 / self.f
        self.enlace = EnlaceJuntas(self, str(g('modo')))
        self.cin = CINEMATICA
        self.t_home = float(g('t_home'))
        self.tol_home = float(g('tol_home'))
        self.realim = str(g('realimentacion')).lower()
        self.desvio_max = float(g('desvio_max'))

        p_home = np.array(self.cin.directa(HOME))
        if self.figura == 'tarea':
            cm = np.array([[25.0,  15.0, 25.0],
                           [25.0,  15.0, 15.0],
                           [25.0,  15.0, 25.0]])
            self.puntos = np.vstack([p_home, cm / 100.0, p_home,
                                     [0.20, -0.18, 0.29], p_home])
        elif self.figura == 'circulo':
            n = 36
            self.puntos = np.vstack(
                [p_home] +
                [circulo(k / n, self.C, self.radio)[0] for k in range(n + 1)] +
                [p_home])
        else:
            self.puntos = np.vstack([p_home, self.A, self.B, self.A, p_home])

        self.n_tramos = len(self.puntos) - 1
        largos = np.linalg.norm(np.diff(self.puntos, axis=0), axis=1)
        self.get_logger().info(
            'poligonal de %d puntos (%d tramos, %.0f mm en total), %.1f s por '
            'tramo -> %.1f s' % (len(self.puntos), self.n_tramos,
                                 largos.sum() * 1000, self.T,
                                 self.T * self.n_tramos * self.pasadas))
        for i, pt in enumerate(self.puntos):
            try:
                sol, _ = self.cin.resolver(pt, preferencia='codo_arriba')
                est = ('q = %7.2f %7.2f %7.2f deg'
                       % tuple(math.degrees(a) for a in sol['q'])) if sol \
                      else '*** NO ALCANZABLE ***'
            except Exception:
                est = '*** NO ALCANZABLE ***'
            self.get_logger().info('   P%-2d (%6.1f %6.1f %6.1f) mm   %s'
                                   % (i, *(pt * 1000), est))

        self.q = np.array(HOME, float)
        self.q_ini_home = None
        self.fase = 'home'
        self.t_fase = 0.0
        self.enlace.enviar(self.q)

        self.t = 0.0
        self.log = []
        self.fin = False
        self.timer = None
        self.rotulo = None
        self._aviso_limite = [False, False, False]
        self._aviso_desvio = False
        self._t_dibujo = 0.0
        if self.ver:
            self.abrir_ventana()
        self.timer = self.create_timer(self.dt, self.paso)

        self.get_logger().info(
            'control cinematico por jacobiano inverso | figura %s | perfil %s | '
            '%.1f s/tramo x %d | K=%.1f | lambda=%.3f | %.0f Hz | realim: %s'
            % (self.figura, self.nombre_perfil, self.T, self.pasadas,
               self.K, self.lam, self.f, self.realim))

    def objetivo(self, t):
        total = self.T * self.n_tramos * self.pasadas
        t = min(max(t, 0.0), total)
        k = int(t / self.T)
        if k >= self.n_tramos * self.pasadas:
            k = self.n_tramos * self.pasadas - 1
        tau = (t - k * self.T) / self.T
        tau = min(max(tau, 0.0), 1.0)
        i = k % self.n_tramos
        p0, pf = self.puntos[i], self.puntos[i + 1]

        if self.nombre_perfil == 'trapezoidal':
            sp, sd = self.perfil(tau, self.rampa)
        else:
            sp, sd = self.perfil(tau)
        return p0 + (pf - p0) * sp, (pf - p0) * sd / self.T

    def paso(self):
        if self.fin:
            return

        q_real = np.array(self.enlace.real, float)

        if self.fase == 'home':
            if self.q_ini_home is None:
                if not self.enlace.hay_medida and self.t_fase < 2.0:
                    self.t_fase += self.dt
                    return
                self.q_ini_home = q_real.copy()
                self.t_fase = 0.0
                self.get_logger().info(
                    'yendo al home (%.1f, %.1f, %.1f) deg desde (%.1f, %.1f, %.1f)'
                    % (*np.degrees(HOME), *np.degrees(self.q_ini_home)))
            u = min(self.t_fase / self.t_home, 1.0)
            sp, _ = perfil_quintico(u)
            self.q = self.q_ini_home + (np.array(HOME) - self.q_ini_home) * sp
            self.enlace.enviar(self.q)
            self.t_fase += self.dt
            if u >= 1.0:
                err_home = float(np.max(np.abs(q_real - np.array(HOME))))
                if err_home < self.tol_home:
                    self.fase = 'trayectoria'
                    self.q = q_real.copy() if self.enlace.hay_medida \
                        else np.array(HOME, float)
                    self.get_logger().info(
                        'home alcanzado (error %.2f deg), empieza la trayectoria'
                        % math.degrees(err_home))
                elif self.t_fase > self.t_home + 20.0:
                    self.get_logger().error(
                        'NO se alcanzo el home: quedan %.2f grados de error '
                        '(tolerancia %.2f). No se lanza la trayectoria, los '
                        'datos no servirian. Revisa el lazo de las juntas.'
                        % (math.degrees(err_home), math.degrees(self.tol_home)))
                    self.terminar()
            return

        total = self.T * self.n_tramos * self.pasadas
        if self.t > total + 0.3:
            self.terminar()
            return

        q_lazo = q_real if self.realim == 'medida' else self.q
        x = np.array(self.cin.directa(q_lazo))
        xd, xd_p = self.objetivo(self.t)
        err = xd - x
        xe = xd_p + self.K * err
        qd = jac.vel_juntas(q_lazo, xe, self.lam)
        self.q = self.q + qd * self.dt

        if self.desvio_max > 0 and self.enlace.hay_medida:
            d = self.q - q_real
            exceso = np.abs(d) > self.desvio_max
            if exceso.any():
                self.q[exceso] = (q_real[exceso]
                                  + np.sign(d[exceso]) * self.desvio_max)
                if not self._aviso_desvio:
                    self._aviso_desvio = True
                    self.get_logger().warn(
                        'el control va %.0f grados por delante de las juntas: '
                        'se limita la integracion (anti-windup)'
                        % math.degrees(self.desvio_max))

        for i, (nom, (lo, hi)) in enumerate(self.cin.limites.items()):
            if self.q[i] < lo or self.q[i] > hi:
                if not self._aviso_limite[i]:
                    self._aviso_limite[i] = True
                    self.get_logger().warn(
                        '%s en el limite (%.1f deg, rango %.0f a %.0f): la '
                        'trayectoria se sale del espacio de trabajo'
                        % (nom, math.degrees(self.q[i]),
                           math.degrees(lo), math.degrees(hi)))
                self.q[i] = min(max(self.q[i], lo), hi)
        self.enlace.enviar(self.q)

        if int(self.t) != int(self.t - self.dt):
            self.get_logger().info(
                't=%4.1f s  K=%.1f  error %6.2f mm  medida de Gazebo: %s'
                % (self.t, self.K, float(np.linalg.norm(err)) * 1000.0,
                   'si' if self.enlace.hay_medida else 'NO'))

        m = jac.medidas(q_real)
        x_sim = np.array(self.cin.directa(self.q))
        self.log.append([self.t, *xd, *x, *err, float(np.linalg.norm(err)),
                         *q_real, *qd, m['det'], m['condicion'], *xe, *xd_p,
                         *x_sim, *self.q])
        self.t += self.dt

    def terminar(self):
        if self.fin:
            return
        self.fin = True
        if self.timer is not None:
            self.timer.cancel()
        d = np.array(self.log)
        if len(d) < 5:
            self.get_logger().error('no se registro nada')
            rclpy.shutdown(); return

        if self.guardar:
            os.makedirs(os.path.dirname(self.salida) or '.', exist_ok=True)
            cab = ('t,xd,yd,zd,x,y,z,ex,ey,ez,enorm,q1,q2,q3,qd1,qd2,qd3,'
                   'det,cond,xe_x,xe_y,xe_z,xdp_x,xdp_y,xdp_z,'
                   'xs,ys,zs,qp1,qp2,qp3')
            np.savetxt(self.salida + '.csv', d, delimiter=',', header=cab,
                       comments='')
            self.get_logger().info('CSV en %s.csv' % self.salida)

        e = d[:, 10] * 1000.0
        self.get_logger().info(
            'error del efector: medio %.2f mm, maximo %.2f mm, final %.2f mm'
            % (e.mean(), e.max(), e[-1]))
        self.get_logger().info('determinante minimo |det J| = %.3e'
                               % np.min(np.abs(d[:, 17])))
        if self.ver:
            self.refrescar(forzar=True)
            self.get_logger().info('recorrido terminado; la ventana de graficas '
                                   'se queda abierta, cierrala cuando quieras')
        else:
            rclpy.shutdown()

    def abrir_ventana(self):
        import matplotlib
        matplotlib.use('TkAgg', force=True)
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        self.plt = plt
        plt.ion()

        fig = plt.figure('Control cinematico por JACOBIANO INVERSO',
                         figsize=(16, 8))
        gs = fig.add_gridspec(2, 4)

        a3 = fig.add_subplot(gs[0, 0], projection='3d')
        a3.set_xlabel('x [mm]'); a3.set_ylabel('y [mm]'); a3.set_zlabel('z [mm]')
        a3.set_title('TCP en 3D', fontsize=10)
        self.l3d_d, = a3.plot([], [], [], lw=2, label='pedida')
        self.l3d_r, = a3.plot([], [], [], lw=1.5, ls='--', label='real')
        a3.legend(fontsize=7)

        ayz = fig.add_subplot(gs[0, 1])
        ayz.set_xlabel('y [mm]'); ayz.set_ylabel('z [mm]'); ayz.grid(alpha=.3)
        ayz.set_title('Plano YZ', fontsize=10)
        self.lyz_d, = ayz.plot([], [], lw=2, label='pedida')
        self.lyz_r, = ayz.plot([], [], lw=1.5, ls='--', label='real')
        self.lyz_p, = ayz.plot([], [], 'o', ms=6, c='tab:red')
        ayz.legend(fontsize=7)

        aej = fig.add_subplot(gs[0, 2])
        aej.set_xlabel('t [s]'); aej.set_ylabel('mm'); aej.grid(alpha=.3)
        aej.set_title('Cada eje: pedido vs real', fontsize=10)
        self.lej = []
        for i, n in enumerate('xyz'):
            d_, = aej.plot([], [], lw=1.8, label=n + ' pedida')
            r_, = aej.plot([], [], lw=1, ls='--', label=n + ' real')
            self.lej.append((d_, r_))
        aej.legend(fontsize=6, ncol=3)

        aer = fig.add_subplot(gs[1, 0])
        aer.set_xlabel('t [s]'); aer.set_ylabel('error [mm]'); aer.grid(alpha=.3)
        aer.axhline(0, c='k', lw=.6)
        self.ler = []
        for n in 'xyz':
            ln, = aer.plot([], [], lw=1.3, label='e' + n)
            self.ler.append(ln)
        self.ler_n, = aer.plot([], [], lw=2, c='tab:red', label='norma')
        aer.legend(fontsize=7, ncol=4)
        self.ax_error = aer

        avj = fig.add_subplot(gs[1, 1])
        avj.set_xlabel('t [s]'); avj.set_ylabel('grados/s'); avj.grid(alpha=.3)
        avj.set_title('SALIDA DEL JACOBIANO INVERSO\nq_punto = pinv(J) xe',
                      fontsize=10)
        self.lvj = []
        for i in range(3):
            ln, = avj.plot([], [], lw=1.4, label='J%d' % (i + 1))
            self.lvj.append(ln)
        avj.legend(fontsize=7)

        apv = fig.add_subplot(gs[0, 3])
        apv.set_xlabel('t [s]'); apv.set_ylabel('|x_punto| [mm/s]')
        apv.grid(alpha=.3)
        apv.set_title('Perfil de velocidad: %s' % self.nombre_perfil, fontsize=10)
        self.lpv, = apv.plot([], [], lw=2, c='tab:brown')
        self.ax_perfil = apv

        asc = fig.add_subplot(gs[1, 2:4])
        asc.set_xlabel('t [s]'); asc.set_ylabel('mm/s'); asc.grid(alpha=.3)
        asc.set_title('Senal de control  xe = xd_punto + K(xd - x)', fontsize=10)
        self.lsc = []
        for n in 'xyz':
            ln, = asc.plot([], [], lw=1.4, label='xe ' + n)
            self.lsc.append(ln)
        asc.legend(fontsize=7, ncol=3)

        self.ejes = (a3, ayz, aej, apv, aer, asc, avj)
        fig.tight_layout(rect=[0, 0.10, 1, 1])

        from matplotlib.widgets import Slider
        ax_k = fig.add_axes([0.07, 0.045, 0.30, 0.025])
        ax_l = fig.add_axes([0.07, 0.012, 0.30, 0.025])
        self.sl_k = Slider(ax_k, 'K  [1/s]', 0.0, 20.0, valinit=self.K,
                           valstep=0.1, color='tab:blue')
        self.sl_l = Slider(ax_l, 'lambda', 0.0, 0.20, valinit=self.lam,
                           valstep=0.001, color='tab:orange')

        def _k(v):
            self.K = float(v)
            self.get_logger().info('K = %.2f (cambiado en caliente)' % self.K)

        def _l(v):
            self.lam = float(v)
            self.get_logger().info('lambda = %.3f (cambiado en caliente)' % self.lam)

        self.sl_k.on_changed(_k)
        self.sl_l.on_changed(_l)

        self.rotulo = fig.text(
            0.45, 0.03, '', fontsize=10, family='monospace',
            va='center',
            bbox=dict(boxstyle='round', fc='#eef3fa', ec='#89a'))

        self.fig = fig
        fig.show()

    def refrescar(self, forzar=False):
        if not self.ver or not self.log:
            return
        if not forzar and (self.t - self._t_dibujo) < 1.0 / self.refresco:
            return
        self._t_dibujo = self.t
        d = np.array(self.log)
        t = d[:, 0]
        xd, x = d[:, 1:4] * 1000, d[:, 4:7] * 1000
        err, en = d[:, 7:10] * 1000, d[:, 10] * 1000

        self.l3d_d.set_data_3d(xd[:, 0], xd[:, 1], xd[:, 2])
        self.l3d_r.set_data_3d(x[:, 0], x[:, 1], x[:, 2])
        self.lyz_d.set_data(xd[:, 1], xd[:, 2])
        self.lyz_r.set_data(x[:, 1], x[:, 2])
        self.lyz_p.set_data([x[-1, 1]], [x[-1, 2]])
        for i, (ld, lr) in enumerate(self.lej):
            ld.set_data(t, xd[:, i]); lr.set_data(t, x[:, i])
        for i, ln in enumerate(self.ler):
            ln.set_data(t, err[:, i])
        self.ler_n.set_data(t, en)
        for i, ln in enumerate(self.lvj):
            ln.set_data(t, np.degrees(d[:, 14 + i]))
        xe = d[:, 19:22] * 1000.0
        xdp = d[:, 22:25] * 1000.0
        for i, ln in enumerate(self.lsc):
            ln.set_data(t, xe[:, i])
        self.lpv.set_data(t, np.linalg.norm(xdp, axis=1))

        self.ax_error.set_title(
            'Error: ahora %.2f mm | medio %.2f | maximo %.2f'
            % (en[-1], en.mean(), en.max()), fontsize=10)
        if self.rotulo is not None:
            self.rotulo.set_text(
                'LAZO CERRADO EN LINEA sobre /joint_states (posicion real de '
                'Gazebo)\n'
                'q_punto = pinv(J(q_real)) * [ xd_punto + K (xd - x) ]      '
                'K = %.2f 1/s    lambda = %.3f\n'
                't = %5.2f s   error ahora %6.2f mm   medio %6.2f mm   '
                'medidas recibidas: %d'
                % (self.K, self.lam, t[-1], en[-1], en.mean(), len(d)))
        a3 = self.ejes[0]
        a3.set_xlim(min(xd[:, 0].min(), x[:, 0].min()) - 10,
                    max(xd[:, 0].max(), x[:, 0].max()) + 10)
        a3.set_ylim(min(xd[:, 1].min(), x[:, 1].min()) - 10,
                    max(xd[:, 1].max(), x[:, 1].max()) + 10)
        a3.set_zlim(min(xd[:, 2].min(), x[:, 2].min()) - 10,
                    max(xd[:, 2].max(), x[:, 2].max()) + 10)
        for a in self.ejes[1:]:
            a.relim(); a.autoscale_view()
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            self.ver = False


def main():
    rclpy.init()
    n = ControlCinematico()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.002)
            if n.fin:
                break
            n.refrescar()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if n.ver:
            try:
                n.plt.ioff(); n.plt.show()
            except Exception:
                pass
        try:
            n.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
