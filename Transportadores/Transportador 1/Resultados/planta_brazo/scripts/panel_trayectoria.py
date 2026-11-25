#!/usr/bin/env python3
# Nodo GUI: planeacion de trayectorias (recta o ciclo de 14 puntos) y perfiles.
import math
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import rclpy
from rclpy.node import Node

import cinematica as cin
from cinematica import Cinematica
from enlace_juntas import EnlaceJuntas
import perfiles

HZ = 50.0
HZ_PLOT = 100.0
MM = 1000.0
JUNTAS = ('J1', 'J2', 'J3')
COLOR = {'J1': '#2a78d6', 'J2': '#eb6834', 'J3': '#1baf7a'}
ESTILO = {'J1': '-', 'J2': '--', 'J3': '-.'}
TINTA = '#0b0b0b'

CICLO = [
    ('P0  home',             (   0,   0,    0)),
    ('P1  levantar',         (   0,  60,  -90)),
    ('P2  girar a recogida', (  75,  60,  -90)),
    ('P3  aproximar',        (  75,  20,  -40)),
    ('P4  recoger',          (  75, -35,   10)),
    ('P5  retirar',          (  75,  20,  -40)),
    ('P6  elevar',           (  75,  60,  -90)),
    ('P7  girar a descarga', ( -75,  60,  -90)),
    ('P8  aproximar',        ( -75,  20,  -40)),
    ('P9  soltar',           ( -75, -35,   10)),
    ('P10 retirar',          ( -75,  20,  -40)),
    ('P11 elevar',           ( -75,  60,  -90)),
    ('P12 volver al frente', (   0,  60,  -90)),
    ('P13 home',             (   0,   0,    0)),
]

TRAYECTORIAS = ('Recta A-B (cinematica inversa)',
                'Ciclo de transporte (14 puntos)')
PERFILES = ('Trapezoidal', 'Cubico')
MODOS = ('tf automatico: se respetan velocidad Y aceleracion maximas',
         'tf fijo + velocidad de crucero   (tipo 2 de los apuntes)',
         'tf fijo + aceleracion de crucero (tipo 1 de los apuntes)')


class PanelTrayectoria(Node):
    def __init__(self):
        super().__init__('panel_trayectoria')
        for n, v in (('d1', cin.D1), ('a1', cin.A1), ('L2', cin.L2),
                     ('L3', cin.L3), ('Lg', cin.LG)):
            self.declare_parameter(n, v)
        self.declare_parameter('modo', 'gazebo')
        self.declare_parameter('marcadores_gz', True)

        g = lambda n: self.get_parameter(n).value
        self.cin = Cinematica(g('d1'), g('a1'), g('L2'), g('L3'), g('Lg'))
        self.modo = str(g('modo')).lower()
        self.enlace = EnlaceJuntas(self, self.modo)
        self.marcar = bool(g('marcadores_gz')) and self.modo == 'gazebo'

        self.q = [0.0, 0.0, 0.0]
        self.cola = queue.Queue()
        threading.Thread(target=self._servir_marcadores, daemon=True).start()
        self.create_timer(1.0 / HZ, self._publicar)
        self.plan = None
        self.idx = 0
        self.corriendo = False
        self.repetir = False

    def _publicar(self):
        if self.corriendo and self.plan:
            self.q = list(self.plan[self.idx]['q'])
            self.idx += 1
            if self.idx >= len(self.plan):
                self.idx = 0
                if not self.repetir:
                    self.corriendo = False
        self.enlace.enviar(self.q)

    def arrancar(self, plan, repetir=False):
        self.plan = plan
        self.idx = 0
        self.repetir = repetir
        self.corriendo = True

    def detener(self):
        self.corriendo = False

    def ir_a(self, q):
        self.corriendo = False
        self.q = list(q)

    def _servir_marcadores(self):
        while True:
            req = self.cola.get()
            if req is None:
                return
            try:
                subprocess.run(
                    ['gz', 'service', '-s', '/marker',
                     '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Empty',
                     '--timeout', '2000', '--req', req],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=5.0, check=False)
            except (OSError, ValueError, subprocess.SubprocessError):
                self.marcar = False
                return

    def _mat(self, c):
        r, g, b = c
        return ('material: {ambient: {r: %.2f, g: %.2f, b: %.2f, a: 1}, '
                'diffuse: {r: %.2f, g: %.2f, b: %.2f, a: 1}}'
                % (r, g, b, r, g, b))

    def dibujar(self, puntos, hitos):
        if not self.marcar:
            return
        self.cola.put('action: DELETE_ALL, ns: "trayectoria"')
        pts = ', '.join('{x: %.5f, y: %.5f, z: %.5f}' % p for p in puntos[::4])
        self.cola.put(
            'action: ADD_MODIFY, type: LINE_STRIP, id: 1, ns: "trayectoria", '
            '%s, point: [%s]' % (self._mat((0.10, 0.90, 0.25)), pts))
        for i, p in enumerate(hitos):
            self.cola.put(
                'action: ADD_MODIFY, type: SPHERE, id: %d, ns: "trayectoria", '
                'scale: {x: 0.014, y: 0.014, z: 0.014}, '
                'pose: {position: {x: %.5f, y: %.5f, z: %.5f}, '
                'orientation: {w: 1}}, %s'
                % (100 + i, p[0], p[1], p[2], self._mat((1.0, 0.35, 0.0))))


def hitos_recta(cin, A, B, n):
    hitos, fallos = [], []
    q_prev = [0.0, 0.0, 0.0]
    for k in range(n):
        s = k / (n - 1.0)
        p = tuple(a + (b - a) * s for a, b in zip(A, B))
        sol, todas = cin.resolver(p, 'tcp', 'codo_arriba', q_actual=q_prev)
        if sol is None:
            razon = ('fuera del alcance' if not todas
                     else 'se sale de los limites articulares')
            fallos.append('punto %d [%.0f %.0f %.0f] mm: %s'
                          % (k + 1, *[v * MM for v in p], razon))
        else:
            q_prev = list(sol['q'])
            hitos.append(list(sol['q']))
    return hitos, fallos


def hitos_ciclo():
    return [[math.radians(a) for a in q] for _, q in CICLO], []


def perfil_del_tramo(dq_dom, tipo, modo, vel, acel, tf):
    if abs(dq_dom) < perfiles.TOL:
        return perfiles.Cubico(max(tf, 0.2)), None

    if modo == 0:
        if tipo == 'cubico':
            t = perfiles.tf_cubico(dq_dom, vel, acel)
            return perfiles.Cubico(t), None
        t, tc = perfiles.Trapezoidal.tiempo_minimo(dq_dom, vel, acel)
        return perfiles.Trapezoidal(t, tc), None

    if tipo == 'cubico':
        return perfiles.Cubico(tf), None
    if modo == 1:
        return perfiles.Trapezoidal.por_velocidad(dq_dom, tf, vel), None
    return perfiles.Trapezoidal.por_aceleracion(dq_dom, tf, acel), None


def rango_admisible(dq_dom, tf):
    d = abs(dq_dom) / tf
    return d, 2.0 * d


def calcular(cin, hitos, tipo, modo, vel, acel, tf):
    muestras, cortes, avisos = [], [], []
    t0 = 0.0
    dt = 1.0 / HZ_PLOT
    for k in range(len(hitos) - 1):
        qa, qb = hitos[k], hitos[k + 1]
        dq = [b - a for a, b in zip(qa, qb)]
        dom = max(dq, key=abs)
        try:
            perfil, _ = perfil_del_tramo(dom, tipo, modo, vel, acel, tf)
        except ValueError as e:
            lo, hi = rango_admisible(dom, tf)
            avisos.append('tramo %d (%.1f deg): %s. Admisible entre %.1f y '
                          '%.1f deg/s'
                          % (k + 1, math.degrees(dom), e,
                             math.degrees(lo), math.degrees(hi)))
            perfil = perfiles.Cubico(tf)
        n = max(int(round(perfil.tf * HZ_PLOT)), 2)
        for m in range(n):
            t = m * dt
            sv, sd, sdd = perfil.s(t), perfil.sd(t), perfil.sdd(t)
            q = [a + sv * d for a, d in zip(qa, dq)]
            muestras.append({
                't': t0 + t,
                'q': q,
                'qd': [sd * d for d in dq],
                'qdd': [sdd * d for d in dq],
                'p': cin.directa(q, 'tcp'),
                'tramo': k + 1,
                'perfil': perfil,
            })
        t0 += perfil.tf
        cortes.append(t0)
    if muestras:
        q = list(hitos[-1])
        muestras.append({'t': t0, 'q': q, 'qd': [0.0] * 3, 'qdd': [0.0] * 3,
                         'p': cin.directa(q, 'tcp'),
                         'tramo': len(hitos) - 1,
                         'perfil': muestras[-1]['perfil']})
    return muestras, cortes, avisos


def derivar(valores, dt):
    n = len(valores)
    if n < 2:
        return [0.0] * n
    out = [0.0] * n
    for i in range(n):
        if i == 0:
            out[i] = (valores[1] - valores[0]) / dt
        elif i == n - 1:
            out[i] = (valores[-1] - valores[-2]) / dt
        else:
            out[i] = (valores[i + 1] - valores[i - 1]) / (2.0 * dt)
    return out


def main():
    rclpy.init()
    nodo = PanelTrayectoria()
    threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True).start()

    raiz = tk.Tk()
    raiz.title('PLANTA BRAZO - planeacion de trayectorias  (%s)' % nodo.modo)
    raiz.configure(padx=10, pady=10)

    estado = {'muestras': [], 'hitos': []}

    izq = tk.Frame(raiz)
    izq.grid(row=0, column=0, sticky='ns', padx=(0, 12))
    der = tk.Frame(raiz)
    der.grid(row=0, column=1, sticky='nsew')
    raiz.columnconfigure(1, weight=1)
    raiz.rowconfigure(0, weight=1)

    f = 0
    tk.Label(izq, text='TRAYECTORIA', font=('TkDefaultFont', 10, 'bold'),
             fg='#046').grid(row=f, column=0, columnspan=2, sticky='w'); f += 1

    cb_tray = ttk.Combobox(izq, values=TRAYECTORIAS, state='readonly', width=30)
    cb_tray.current(0)
    cb_tray.grid(row=f, column=0, columnspan=2, sticky='we', pady=(2, 6)); f += 1

    marco_recta = tk.Frame(izq)
    marco_recta.grid(row=f, column=0, columnspan=2, sticky='we'); f += 1
    ent = {}
    for i, (nom, val) in enumerate((('A  x,y,z [mm]', '300, -150, 250'),
                                    ('B  x,y,z [mm]', '300, 150, 250'))):
        tk.Label(marco_recta, text=nom, width=13, anchor='w').grid(
            row=i, column=0, sticky='w')
        e = tk.Entry(marco_recta, width=18)
        e.insert(0, val)
        e.grid(row=i, column=1, sticky='we', pady=1)
        ent[nom[0]] = e
    tk.Label(marco_recta, text='puntos', width=13, anchor='w').grid(
        row=2, column=0, sticky='w')
    ent['N'] = tk.Entry(marco_recta, width=18)
    ent['N'].insert(0, '12')
    ent['N'].grid(row=2, column=1, sticky='we', pady=1)

    ttk.Separator(izq, orient='horizontal').grid(
        row=f, column=0, columnspan=2, sticky='we', pady=8); f += 1

    tk.Label(izq, text='PERFIL DE VELOCIDAD',
             font=('TkDefaultFont', 10, 'bold'), fg='#046').grid(
                 row=f, column=0, columnspan=2, sticky='w'); f += 1

    cb_perf = ttk.Combobox(izq, values=PERFILES, state='readonly', width=30)
    cb_perf.current(0)
    cb_perf.grid(row=f, column=0, columnspan=2, sticky='we', pady=(2, 4)); f += 1

    cb_modo = ttk.Combobox(izq, values=MODOS, state='readonly', width=42)
    cb_modo.current(0)
    cb_modo.grid(row=f, column=0, columnspan=2, sticky='we', pady=2); f += 1

    marco_val = tk.Frame(izq)
    marco_val.grid(row=f, column=0, columnspan=2, sticky='we', pady=2); f += 1
    campos = (('vel',  'velocidad  [deg/s]',   '71.4'),
              ('acel', 'aceleracion [deg/s2]', '79.4'),
              ('tf',   'tf por tramo [s]',     '3.0'))
    ent_v = {}
    for i, (clave, etiq, val) in enumerate(campos):
        tk.Label(marco_val, text=etiq, width=19, anchor='w').grid(
            row=i, column=0, sticky='w')
        e = tk.Entry(marco_val, width=12)
        e.insert(0, val)
        e.grid(row=i, column=1, sticky='we', pady=1)
        ent_v[clave] = e

    ttk.Separator(izq, orient='horizontal').grid(
        row=f, column=0, columnspan=2, sticky='we', pady=8); f += 1

    msg = tk.Label(izq, text='', font=('TkFixedFont', 9), justify='left',
                   anchor='nw', wraplength=250, height=9)
    msg.grid(row=f + 1, column=0, columnspan=2, sticky='we', pady=(8, 0))

    barra = tk.Frame(izq)
    barra.grid(row=f, column=0, columnspan=2, sticky='we'); f += 2

    libreta = ttk.Notebook(der)
    libreta.pack(fill='both', expand=True)
    tab_j = tk.Frame(libreta)
    tab_w = tk.Frame(libreta)
    libreta.add(tab_j, text='  Espacio de juntas  ')
    libreta.add(tab_w, text='  Espacio de trabajo  ')

    fig_j = Figure(figsize=(7.4, 6.2), dpi=96)
    ax_j = [fig_j.add_subplot(3, 1, i + 1) for i in range(3)]
    lienzo_j = FigureCanvasTkAgg(fig_j, master=tab_j)
    lienzo_j.get_tk_widget().pack(fill='both', expand=True)

    fig_w = Figure(figsize=(7.4, 6.2), dpi=96)
    ax_w = [fig_w.add_subplot(3, 1, i + 1) for i in range(3)]
    lienzo_w = FigureCanvasTkAgg(fig_w, master=tab_w)
    lienzo_w.get_tk_widget().pack(fill='both', expand=True)

    def estilo(ax, ylab, xlab=None):
        ax.grid(alpha=0.25, lw=0.5)
        ax.set_ylabel(ylab, fontsize=8)
        ax.tick_params(labelsize=7)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        if xlab:
            ax.set_xlabel(xlab, fontsize=8)

    def leer_punto(texto):
        v = [float(x) / MM for x in texto.replace(';', ',').split(',')]
        if len(v) != 3:
            raise ValueError('hacen falta tres numeros: x, y, z')
        return tuple(v)

    def construir_hitos():
        if cb_tray.current() == 0:
            A = leer_punto(ent['A'].get())
            B = leer_punto(ent['B'].get())
            n = max(2, int(ent['N'].get()))
            return hitos_recta(nodo.cin, A, B, n)
        return hitos_ciclo()

    def calcular_y_graficar():
        try:
            hitos, fallos = construir_hitos()
        except ValueError as e:
            msg.config(text='Error en los datos:\n%s' % e, fg='#b4302a')
            return
        if fallos:
            msg.config(text='Sin solucion de cinematica inversa:\n' +
                            '\n'.join(fallos[:4]), fg='#b4302a')
            return
        if len(hitos) < 2:
            msg.config(text='Hacen falta al menos dos puntos.', fg='#b4302a')
            return

        tipo = 'cubico' if cb_perf.current() == 1 else 'trapezoidal'
        modo = cb_modo.current()
        try:
            vel = math.radians(float(ent_v['vel'].get()))
            acel = math.radians(float(ent_v['acel'].get()))
            tf = float(ent_v['tf'].get())
            if vel <= 0 or acel <= 0 or tf <= 0:
                raise ValueError('los tres valores deben ser positivos')
        except ValueError as e:
            msg.config(text='Error en los datos:\n%s' % e, fg='#b4302a')
            return

        muestras, cortes, avisos = calcular(nodo.cin, hitos, tipo, modo,
                                            vel, acel, tf)
        estado['muestras'] = muestras
        estado['cortes'] = cortes
        estado['hitos'] = hitos
        pintar(muestras, cortes, hitos, tipo, avisos)
        nodo.dibujar([m['p'] for m in muestras],
                     [nodo.cin.directa(h, 'tcp') for h in hitos])

    def pintar(muestras, cortes, hitos, tipo, avisos):
        t = [m['t'] for m in muestras]
        dt = 1.0 / HZ_PLOT

        for a in ax_j:
            a.clear()
        for idx, (a, ylab) in enumerate(zip(
                ax_j, ('Posicion [deg]', 'Velocidad [deg/s]',
                       'Aceleracion [deg/s2]'))):
            clave = ('q', 'qd', 'qdd')[idx]
            for i, j in enumerate(JUNTAS):
                a.plot(t, [math.degrees(m[clave][i]) for m in muestras],
                       color=COLOR[j], ls=ESTILO[j], lw=1.2, label=j)
            estilo(a, ylab, 'Tiempo [s]' if idx == 2 else None)
            a.legend(ncol=3, fontsize=7, loc='upper right', framealpha=0.9)
        for c in cortes[:-1]:
            for a in ax_j:
                a.axvline(c, color='0.85', lw=0.5, zorder=0)
        ax_j[0].set_title('%s  -  %d tramos' % (tipo, len(hitos) - 1),
                          fontsize=9, loc='left')
        fig_j.tight_layout(h_pad=0.5)
        lienzo_j.draw()

        for a in ax_w:
            a.clear()
        ejes = list(zip(*[m['p'] for m in muestras]))
        for (nom, y), j in zip(zip(('X', 'Y', 'Z'), ejes), JUNTAS):
            ax_w[0].plot(t, [v * MM for v in y], color=COLOR[j],
                         ls=ESTILO[j], lw=1.2, label=nom)
        estilo(ax_w[0], 'Posicion [mm]')
        ax_w[0].legend(ncol=3, fontsize=7, loc='upper right', framealpha=0.9)

        vel = [0.0]
        for k in range(1, len(muestras)):
            vel.append(math.dist(muestras[k]['p'], muestras[k - 1]['p']) / dt)
        ax_w[1].plot(t, [v * MM for v in vel], color=COLOR['J1'], lw=1.3)
        estilo(ax_w[1], 'Velocidad [mm/s]')
        ax_w[1].annotate('max. %.0f mm/s' % (max(vel) * MM),
                         xy=(0.99, 0.85), xycoords='axes fraction',
                         ha='right', fontsize=7,
                         bbox=dict(facecolor='white', edgecolor='none', pad=1))

        acc = derivar(vel, dt)
        ax_w[2].plot(t, [a * MM for a in acc], color=COLOR['J2'], lw=1.1)
        estilo(ax_w[2], 'Aceleracion [mm/s2]', 'Tiempo [s]')

        for a in ax_w:
            for c in cortes[:-1]:
                a.axvline(c, color='0.85', lw=0.5, zorder=0)
        ax_w[0].set_title('Punta del efector', fontsize=9, loc='left')
        fig_w.tight_layout(h_pad=0.5)
        lienzo_w.draw()

        pico_j = [max(abs(math.degrees(m['qd'][i])) for m in muestras)
                  for i in range(3)]
        pico_a = [max(abs(math.degrees(m['qdd'][i])) for m in muestras)
                  for i in range(3)]
        tfs = sorted({round(m['perfil'].tf, 3) for m in muestras})
        if len(tfs) == 1:
            dur = 'tf = %.2f s en los %d tramos' % (tfs[0], len(hitos) - 1)
        else:
            dur = 'tf entre %.2f y %.2f s (%d tramos)' % (
                tfs[0], tfs[-1], len(hitos) - 1)
        lineas = ['perfil %s' % tipo,
                  dur,
                  'duracion total %.2f s' % muestras[-1]['t'],
                  '',
                  'pico de velocidad [deg/s]',
                  '  J1 %6.1f  J2 %6.1f  J3 %6.1f' % tuple(pico_j),
                  'pico de aceleracion [deg/s2]',
                  '  J1 %6.1f  J2 %6.1f  J3 %6.1f' % tuple(pico_a),
                  'pico lineal del TCP: %.0f mm/s' % (max(vel) * MM)]
        if avisos:
            lineas += ['', 'AVISOS:'] + ['  ' + a for a in avisos[:3]]
        msg.config(text='\n'.join(lineas),
                   fg='#9a6c00' if avisos else '#177a52')

    def ejecutar(repetir=False):
        if not estado['muestras']:
            calcular_y_graficar()
        if not estado['muestras']:
            return
        plan = []
        paso = max(int(round(HZ_PLOT / HZ)), 1)
        for m in estado['muestras'][::paso]:
            plan.append({'q': m['q']})
        nodo.arrancar(plan, repetir)
        msg.config(text=msg.cget('text') + '\n\nejecutando en %s%s'
                   % (nodo.modo, ' (en bucle)' if repetir else ''),
                   fg='#046')

    def home():
        nodo.detener()
        nodo.ir_a([0.0, 0.0, 0.0])

    tk.Button(barra, text='Calcular y graficar', width=18,
              command=calcular_y_graficar).grid(row=0, column=0, pady=2)
    tk.Button(barra, text='Ejecutar', width=8,
              command=lambda: ejecutar(False)).grid(row=1, column=0,
                                                    sticky='w', pady=2)
    tk.Button(barra, text='En bucle', width=8,
              command=lambda: ejecutar(True)).grid(row=1, column=0,
                                                   sticky='e', pady=2)
    tk.Button(barra, text='Detener', width=8,
              command=nodo.detener).grid(row=2, column=0, sticky='w', pady=2)
    tk.Button(barra, text='Home', width=8,
              command=home).grid(row=2, column=0, sticky='e', pady=2)

    def cambio_modo(_e=None):
        m = cb_modo.current()
        cubico = cb_perf.current() == 1
        ent_v['tf'].config(state='disabled' if m == 0 else 'normal')
        if m == 0:
            ent_v['vel'].config(state='normal')
            ent_v['acel'].config(state='normal')
        elif cubico:
            ent_v['vel'].config(state='disabled')
            ent_v['acel'].config(state='disabled')
        else:
            ent_v['vel'].config(state='normal' if m == 1 else 'disabled')
            ent_v['acel'].config(state='normal' if m == 2 else 'disabled')

    def cambio_perfil(_e=None):
        cambio_modo()
    cb_perf.bind('<<ComboboxSelected>>', cambio_perfil)
    cb_modo.bind('<<ComboboxSelected>>', cambio_modo)
    cambio_modo()

    def cambio_tray(_e=None):
        for w in marco_recta.winfo_children():
            w.configure(state='normal' if cb_tray.current() == 0 else 'disabled')
    cb_tray.bind('<<ComboboxSelected>>', cambio_tray)

    calcular_y_graficar()

    def cerrar():
        nodo.detener()
        raiz.destroy()
    raiz.protocol('WM_DELETE_WINDOW', cerrar)

    try:
        raiz.mainloop()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
