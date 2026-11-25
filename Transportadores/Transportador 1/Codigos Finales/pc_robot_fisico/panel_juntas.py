#!/usr/bin/env python3
"""
PANEL DE JUNTAS - mueve el robot FISICO y ENSENA la trayectoria punto a punto.

    cd ~/robot_fisico && python3 panel_juntas.py

Un deslizador por junta.  Lo pones donde quieras y el brazo va.  Por debajo
hay un lazo de POSICION sencillo (proporcional) que convierte el error en la
velocidad que entiende el firmware:

        v = K * (objetivo - medido)        limitada a VEL_MAX de cada junta

El firmware solo acepta consignas de VELOCIDAD, y ademas se para solo si deja
de recibirlas durante medio segundo, asi que el panel manda una cada 50 ms
aunque no muevas nada.

ENSENANZA POR GUIADO:
  J1 y J3 se pueden colocar A MANO: el encoder los sigue.
  J2 NO: su encoder esta en el eje del motor, antes de la correa, asi que
  moverla a mano no se registra.  Esa se coloca con su deslizador.
  Cuando la postura guste, "Guardar punto" la anade a la lista, y
  "Guardar archivo" escribe todo en ~/Desktop/trayectoria_ensenada.txt

Seguridad:
  - los deslizadores no pasan de los topes MEDIDOS en el robot
  - boton PARAR grande, y al cerrar la ventana se para y se cierra el puerto
  - la velocidad maxima se puede bajar con el deslizador de abajo
"""
import sys
import os
import tkinter as tk
from tkinter import ttk

import numpy as np

import cinematica as cin
import config as cfg
from enlace_esp import EnlaceESP
import estado_palas

JUNTAS = ('J1', 'J2', 'J3')
SALIDA = os.path.expanduser('~/Desktop/trayectoria_ensenada.txt')
PAR_J2_MAX = 0.65        # N.m que da el motor de 98:1
M2, R2 = 0.32, 0.073
M3, R3 = 0.26, 0.068
MG, RG = 0.09, 0.030


def par_j2(t2, t3):
    """Par de gravedad en el hombro, en N.m (brazo vacio)."""
    import math as _m
    t2, t3 = _m.radians(t2), _m.radians(t3)
    x2 = R2 * _m.cos(t2)
    x3 = cfg.A2 * _m.cos(t2) + R3 * _m.cos(t2 + t3)
    xg = cfg.A2 * _m.cos(t2) + (cfg.L3 + RG) * _m.cos(t2 + t3)
    return 9.81 * (M2 * x2 + M3 * x3 + MG * xg)
PERIODO_MS = 50
K_POS = 1.5          # 1/s, ganancia del lazo de posicion
BANDA = 0.5          # grados: por debajo de esto se considera llegado
VEL_MAX = {'J1': 60.0, 'J2': 35.0, 'J3': 50.0}     # grados/s


class Panel:
    def __init__(self, raiz):
        self.raiz = raiz
        raiz.title('PLANTA BRAZO - panel de juntas (robot fisico)')
        self.enlace = EnlaceESP()
        self.medido = list(cfg.HOME_GRADOS)
        self.activo = True
        self.escala_vel = 1.0
        self.g_medido = 0.0          # cuentas del gripper, de la telemetria
        self.g_hay = False           # si el firmware las esta mandando
        self.pedir_home = False      # referenciar las palas en el proximo ciclo
        self.g_ref = False           # el firmware CONFIRMO el referenciado
        self.g_espera = None         # (ok0, fallos0) mientras se espera la respuesta

        marco = ttk.Frame(raiz, padding=12)
        marco.grid(sticky='nsew')

        ttk.Label(marco, text='Mueve cada junta con su deslizador. '
                              'El brazo va al objetivo que marques.',
                  font=('TkDefaultFont', 10)).grid(row=0, column=0, columnspan=5,
                                                   sticky='w', pady=(0, 10))

        self.obj = {}
        self.lbl_obj = {}
        self.lbl_med = {}
        self.lbl_err = {}
        for i, j in enumerate(JUNTAS):
            lo, hi = cfg.LIMITES[j]
            f = i + 1
            ttk.Label(marco, text=j, font=('TkDefaultFont', 11, 'bold')
                      ).grid(row=f, column=0, padx=(0, 8), sticky='w')
            v = tk.DoubleVar(value=cfg.HOME_GRADOS[i])
            self.obj[j] = v
            s = ttk.Scale(marco, from_=lo, to=hi, variable=v, length=340,
                          orient='horizontal')
            s.grid(row=f, column=1, padx=4, pady=3)
            self.lbl_obj[j] = ttk.Label(marco, width=9, anchor='e')
            self.lbl_obj[j].grid(row=f, column=2)
            self.lbl_med[j] = ttk.Label(marco, width=9, anchor='e',
                                        foreground='#06c')
            self.lbl_med[j].grid(row=f, column=3)
            self.lbl_err[j] = ttk.Label(marco, width=9, anchor='e',
                                        foreground='#888')
            self.lbl_err[j].grid(row=f, column=4)
            ttk.Label(marco, text='(%.0f a %.0f)' % (lo, hi),
                      foreground='#888').grid(row=f, column=5, padx=(8, 0))

        ttk.Label(marco, text='objetivo', foreground='#888').grid(row=0, column=2)
        ttk.Label(marco, text='medido', foreground='#06c').grid(row=0, column=3)
        ttk.Label(marco, text='error', foreground='#888').grid(row=0, column=4)

        # ------------------------------------------------------ botones
        bot = ttk.Frame(marco)
        bot.grid(row=4, column=0, columnspan=6, pady=(12, 4), sticky='w')
        ttk.Button(bot, text='Ir al HOME', command=self.a_home).pack(side='left')
        ttk.Button(bot, text='Objetivo = donde esta',
                   command=self.sincronizar).pack(side='left', padx=6)
        tk.Button(bot, text='PARAR', command=self.parar, bg='#c22', fg='white',
                  font=('TkDefaultFont', 11, 'bold'), padx=18
                  ).pack(side='left', padx=18)

        # --------------------------------------------- velocidad maxima
        vf = ttk.Frame(marco)
        vf.grid(row=5, column=0, columnspan=6, sticky='w', pady=(6, 0))
        ttk.Label(vf, text='velocidad maxima').pack(side='left')
        self.vesc = tk.DoubleVar(value=100.0)
        ttk.Scale(vf, from_=10, to=100, variable=self.vesc, length=200,
                  orient='horizontal').pack(side='left', padx=8)
        self.lbl_vel = ttk.Label(vf, width=6)
        self.lbl_vel.pack(side='left')

        # ------------------------------------------------------ J4: apertura
        # OJO con los nombres: en este panel "palas" ya significa theta2+theta3,
        # la inclinacion del plano del gripper.  Esto de aqui es otra cosa: la
        # APERTURA de las palas, o sea la posicion del motor N20 en cuentas.
        gf = ttk.LabelFrame(marco, text='  J4   apertura de las palas  ',
                            padding=8)
        gf.grid(row=6, column=0, columnspan=6, sticky='ew', pady=(14, 0))
        ttk.Label(gf, text='J4', font=('TkDefaultFont', 11, 'bold')
                  ).grid(row=0, column=0, padx=(0, 8), sticky='w')
        self.g_obj = tk.DoubleVar(value=cfg.GRIPPER_CERRADO)
        ttk.Scale(gf, from_=cfg.GRIPPER_MIN + cfg.GRIPPER_MARGEN,
                  to=cfg.GRIPPER_MAX - cfg.GRIPPER_MARGEN,
                  variable=self.g_obj, length=340,
                  orient='horizontal').grid(row=0, column=1, padx=4)
        self.lbl_g_obj = ttk.Label(gf, width=9, anchor='e')
        self.lbl_g_obj.grid(row=0, column=2)
        self.lbl_g_med = ttk.Label(gf, width=9, anchor='e', foreground='#06c')
        self.lbl_g_med.grid(row=0, column=3)
        self.lbl_g_err = ttk.Label(gf, width=9, anchor='e', foreground='#888')
        self.lbl_g_err.grid(row=0, column=4)
        ttk.Label(gf, text='cuentas (%d a %d)'
                  % (cfg.GRIPPER_MIN + cfg.GRIPPER_MARGEN,
                     cfg.GRIPPER_MAX - cfg.GRIPPER_MARGEN),
                  foreground='#888').grid(row=0, column=5, padx=(8, 0))
        gb = ttk.Frame(gf)
        gb.grid(row=1, column=0, columnspan=6, sticky='w', pady=(8, 0))
        tk.Button(gb, text='Referenciar palas', command=self.referenciar_palas,
                  bg='#146', fg='white', font=('TkDefaultFont', 10, 'bold'),
                  padx=10).pack(side='left')
        ttk.Button(gb, text='Cerrar palas',
                   command=lambda: self.g_obj.set(cfg.GRIPPER_CERRADO)
                   ).pack(side='left', padx=6)
        ttk.Button(gb, text='Abrir palas',
                   command=lambda: self.g_obj.set(cfg.GRIPPER_ABIERTO)
                   ).pack(side='left')
        self.lbl_g_est = ttk.Label(gb, text='sin referenciar',
                                   foreground='#c60')
        self.lbl_g_est.pack(side='left', padx=12)

        # ------------------------------------------------ ensenar puntos
        ens = ttk.LabelFrame(marco, text='  trayectoria ensenada  ', padding=8)
        ens.grid(row=8, column=0, columnspan=6, sticky='ew', pady=(14, 0))
        bf = ttk.Frame(ens); bf.pack(fill='x')
        tk.Button(bf, text='Guardar punto', command=self.guardar_punto,
                  bg='#161', fg='white', font=('TkDefaultFont', 10, 'bold'),
                  padx=10).pack(side='left')
        ttk.Button(bf, text='Quitar el ultimo',
                   command=self.quitar_punto).pack(side='left', padx=6)
        ttk.Button(bf, text='Guardar archivo',
                   command=self.guardar_archivo).pack(side='left', padx=6)
        ttk.Button(bf, text='Vaciar',
                   command=self.vaciar).pack(side='left')
        self.lbl_n = ttk.Label(bf, text='0 puntos')
        self.lbl_n.pack(side='left', padx=12)
        self.lista = tk.Text(ens, height=7, width=78, font=('monospace', 9))
        self.lista.pack(fill='x', pady=(6, 0))
        self.puntos = []

        self.estado = ttk.Label(marco, text='', foreground='#060')
        self.estado.grid(row=9, column=0, columnspan=6, sticky='w', pady=(10, 0))
        self.tcp = ttk.Label(marco, text='', font=('TkDefaultFont', 10))
        self.tcp.grid(row=10, column=0, columnspan=6, sticky='w')

        raiz.protocol('WM_DELETE_WINDOW', self.cerrar)
        self.sincronizar()
        self.ciclo()

    # ------------------------------------------------------------- acciones
    def a_home(self):
        for i, j in enumerate(JUNTAS):
            self.obj[j].set(cfg.HOME_GRADOS[i])

    def sincronizar(self):
        """Pone el objetivo donde esta el brazo, para que no salga disparado."""
        # se espera una trama de telemetria (llegan cada 50 ms); si no llega
        # en medio segundo se deja la ultima postura conocida
        import time
        t0 = time.time()
        while time.time() - t0 < 0.5:
            tel = self.enlace.leer_telemetria()
            if tel:
                self.medido = list(tel['med'])
                break
            time.sleep(0.02)
        for i, j in enumerate(JUNTAS):
            self.obj[j].set(self.medido[i])
        if self.g_hay:
            self.g_obj.set(max(cfg.GRIPPER_MIN + cfg.GRIPPER_MARGEN,
                               min(cfg.GRIPPER_MAX - cfg.GRIPPER_MARGEN,
                                   self.g_medido)))

    def parar(self):
        # el gripper tambien: su objetivo pasa a ser donde esta, para que no
        # siga empujando hacia la consigna anterior
        self.g_obj.set(max(cfg.GRIPPER_MIN + cfg.GRIPPER_MARGEN,
                           min(cfg.GRIPPER_MAX - cfg.GRIPPER_MARGEN,
                               self.g_medido)))
        self.enlace.enviar([0.0, 0.0, 0.0], gripper=float(self.g_obj.get()))
        self.sincronizar()
        self.estado.config(text='PARADO (objetivo puesto donde esta el brazo)',
                           foreground='#c22')

    def cerrar(self):
        self.activo = False
        if self.g_ref and self.g_hay:
            estado_palas.guardar(self.g_medido, 'panel de juntas')
        try:
            self.enlace.enviar([0.0, 0.0, 0.0])
            self.enlace.cerrar()
        except Exception:
            pass
        self.raiz.destroy()

    # ------------------------------------------------------------------ J4
    def referenciar_palas(self):
        """Pide al firmware que busque el tope de entrada de las palas.

        Tarda poco mas de un segundo y durante ese rato el firmware no
        atiende el lazo de las juntas, asi que las pone a velocidad cero
        antes.  El brazo no se mueve; solo se queda quieto.
        """
        self.pedir_home = True
        self.lbl_g_est.config(text='referenciando...', foreground='#c60')

    # ------------------------------------------------- ensenar la trayectoria
    def guardar_punto(self):
        q = list(self.medido)
        p = cin.directa(np.radians(q)) * 1000.0
        self.puntos.append((q, list(p), q[1] + q[2], par_j2(q[1], q[2]),
                            int(round(self.g_medido))))
        self.refrescar_lista()

    def quitar_punto(self):
        if self.puntos:
            self.puntos.pop()
            self.refrescar_lista()

    def vaciar(self):
        self.puntos = []
        self.refrescar_lista()

    def refrescar_lista(self):
        self.lbl_n.config(text='%d puntos' % len(self.puntos))
        self.lista.delete('1.0', 'end')
        self.lista.insert('end',
            '  #   J1      J2      J3   |   x      y      z   | incl   par'
            '   apert\n')
        for i, (q, p, inc, par, ap) in enumerate(self.puntos):
            self.lista.insert('end',
                '%3d %7.2f %7.2f %7.2f | %6.0f %6.0f %6.0f | %5.1f  %5.2f'
                '  %5d%s\n'
                % (i, q[0], q[1], q[2], p[0], p[1], p[2], inc, par, ap,
                   '  <-- par alto' if par > PAR_J2_MAX else ''))
        self.lista.see('end')

    def guardar_archivo(self):
        if not self.puntos:
            self.estado.config(text='no hay puntos que guardar',
                               foreground='#c60')
            return
        try:
            with open(SALIDA, 'w') as f:
                f.write('# Trayectoria ensenada con el robot fisico\n')
                f.write('# geometria L1=%.3f a1=%.5f L2=%.3f L3=%.5f L4=%.3f\n'
                        % (cfg.L1, cfg.A1, cfg.A2, cfg.L3, cfg.L4))
                f.write('# incl = theta2 + theta3 (0 = palas horizontales)\n')
                f.write('# apert = apertura de las palas, en cuentas de\n')
                f.write('#         encoder del N20.  %d cerrado, %d abierto.\n'
                        % (cfg.GRIPPER_CERRADO, cfg.GRIPPER_ABIERTO))
                f.write('# par = par de gravedad en J2, brazo VACIO, max %.2f N.m\n'
                        % PAR_J2_MAX)
                f.write('#\n# CICLO en el formato de trayectorias.py:\n')
                f.write('CICLO = [\n')
                for i, (q, p, inc, par, ap) in enumerate(self.puntos):
                    f.write("    ('P%-2d', (%7.2f, %7.2f, %7.2f)),"
                            "   # TCP (%6.0f,%6.0f,%6.0f) mm  incl %+5.1f"
                            "  par %.2f  apert %d\n"
                            % (i, q[0], q[1], q[2], p[0], p[1], p[2], inc, par,
                               ap))
                f.write(']\n\n')
                f.write('# Apertura de las palas en cada punto, en cuentas:\n')
                f.write('APERTURA = [\n')
                for i, (q, p, inc, par, ap) in enumerate(self.puntos):
                    f.write("    ('P%-2d', %d),\n" % (i, ap))
                f.write(']\n')
            self.estado.config(text='guardado en %s  (%d puntos)'
                                    % (SALIDA, len(self.puntos)),
                               foreground='#060')
        except Exception as e:
            self.estado.config(text='no se pudo guardar: %s' % e,
                               foreground='#c22')

    # ----------------------------------------------------------- lazo a 20 Hz
    def ciclo(self):
        if not self.activo:
            return
        tel = self.enlace.leer_telemetria()
        if tel:
            self.medido = list(tel['med'])
            hay = True
            if 'g' in tel:
                self.g_medido = tel['g']
                self.g_hay = True
        else:
            hay = False
        if self.g_espera is not None:
            ok0, f0 = self.g_espera
            if getattr(self.enlace, 'ref_fallos', 0) > f0:
                self.g_espera = None
                self.lbl_g_est.config(text='NO se referenciaron (motor/cremallera)',
                                      foreground='#c22')
            elif getattr(self.enlace, 'ref_ok', 0) > ok0:
                self.g_espera = None
                self.g_ref = True
                self.lbl_g_est.config(text='referenciado', foreground='#060')

        esc = self.vesc.get() / 100.0
        self.lbl_vel.config(text='%.0f %%' % self.vesc.get())

        v = []
        for i, j in enumerate(JUNTAS):
            objetivo = float(self.obj[j].get())
            lo, hi = cfg.LIMITES[j]
            objetivo = min(max(objetivo, lo), hi)
            e = objetivo - self.medido[i]
            vel = 0.0 if abs(e) < BANDA else K_POS * e
            tope = VEL_MAX[j] * esc
            vel = min(max(vel, -tope), tope)
            v.append(vel)
            self.lbl_obj[j].config(text='%8.2f' % objetivo)
            self.lbl_med[j].config(text='%8.2f' % self.medido[i])
            self.lbl_err[j].config(text='%+8.2f' % e)

        # ------------------------------------------------------------ J4
        if self.pedir_home:
            self.pedir_home = False
            # 2026-11-24: si quedaron FUERA (o no se sabe), referenciado solo
            # hacia dentro; y 'referenciado' solo cuando el firmware lo confirma.
            codigo, motivo = estado_palas.consigna_referenciar()
            print('[palas] %s' % motivo)
            self.g_espera = (getattr(self.enlace, 'ref_ok', 0),
                             getattr(self.enlace, 'ref_fallos', 0))
            self.g_ref = False
            self.enlace.enviar([0.0, 0.0, 0.0], gripper=codigo)
            self.g_obj.set(cfg.GRIPPER_MIN + cfg.GRIPPER_MARGEN)
        else:
            g_obj = float(self.g_obj.get())
            self.enlace.enviar(v, gripper=g_obj)
            self.lbl_g_obj.config(text='%8.0f' % g_obj)
            if self.g_hay:
                self.lbl_g_med.config(text='%8.0f' % self.g_medido)
                self.lbl_g_err.config(text='%+8.0f' % (g_obj - self.g_medido))
            else:
                self.lbl_g_med.config(text='   --')
                self.lbl_g_err.config(text='   --')
                self.lbl_g_est.config(text='el firmware no manda J4',
                                      foreground='#c60')

        p = cin.directa(np.radians(self.medido)) * 1000.0
        inc = self.medido[1] + self.medido[2]
        par = par_j2(self.medido[1], self.medido[2])
        self.tcp.config(
            text='TCP  x=%7.1f  y=%7.1f  z=%7.1f mm     palas %+6.1f deg'
                 '     par J2 %.2f N.m %s'
                 % (p[0], p[1], p[2], inc, par,
                    '(ALTO, max %.2f)' % PAR_J2_MAX if par > PAR_J2_MAX else ''),
            foreground='#c22' if par > PAR_J2_MAX else '#000')
        if not hay:
            self.estado.config(text='sin telemetria de la ESP32...',
                               foreground='#c60')
        elif any(abs(x) > 0.1 for x in v):
            self.estado.config(text='moviendo:  %s' %
                               '   '.join('%s %+6.1f deg/s' % (n, x)
                                          for n, x in zip(JUNTAS, v)),
                               foreground='#060')
        else:
            self.estado.config(text='en posicion', foreground='#060')

        self.raiz.after(PERIODO_MS, self.ciclo)


def main():
    raiz = tk.Tk()
    try:
        Panel(raiz)
    except SystemExit:
        return
    except Exception as e:
        print('[ERROR] %s' % e)
        return
    raiz.mainloop()


if __name__ == '__main__':
    main()
