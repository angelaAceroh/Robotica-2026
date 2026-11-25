#!/usr/bin/env python3
# Nodo GUI: sliders de juntas, cinematica directa e inversa.
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker

import cinematica as cin
from cinematica import Cinematica
from enlace_juntas import EnlaceJuntas
from puntos_prueba import PUNTOS_PRUEBA_RAD
from trayectorias import PUNTOS, PUNTOS_RAD, interpolar

JUNTAS = [
    ('J1', -180.0, 180.0),
    ('J2',  -90.0,  90.0),
    ('J3', -149.0, 149.0),
]

POSTURAS = {
    'codo arriba': 'codo_arriba',
    'codo abajo': 'codo_abajo',
    'la mas cercana': 'cercana',
}


class PanelBrazo(Node):
    def __init__(self):
        super().__init__('gui_articulaciones')
        for n, v in (('d1', cin.D1), ('a1', cin.A1), ('L2', cin.L2),
                     ('L3', cin.L3), ('Lg', cin.LG)):
            self.declare_parameter(n, v)
        self.declare_parameter('modo', 'rviz')
        g = lambda n: self.get_parameter(n).value
        self.cin = Cinematica(g('d1'), g('a1'), g('L2'), g('L3'), g('Lg'))
        self.d1, self.a1 = g('d1'), g('a1')
        self.L2, self.L3, self.Lg = g('L2'), g('L3'), g('Lg')
        self.modo = str(g('modo')).lower()

        self.enlace = EnlaceJuntas(self, self.modo,
                                   juntas=[j[0] for j in JUNTAS])
        self.pub_limpiar = self.create_publisher(Empty, 'traza/limpiar', 10)
        self.pub_objetivo = self.create_publisher(Marker, 'objetivo_ci', 10)
        self.q = [0.0, 0.0, 0.0]
        self.objetivo = None
        self.create_timer(1.0 / 30.0, self.publicar)
        self.get_logger().info(f'panel en modo {self.modo}')

    def publicar(self):
        self.enlace.enviar(self.q)

    def marcar_objetivo(self, p):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = 'objetivo_ci', 0, Marker.SPHERE
        m.action = Marker.ADD if p else Marker.DELETEALL
        m.scale.x = m.scale.y = m.scale.z = 0.025
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.9
        m.pose.orientation.w = 1.0
        if p:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
        self.pub_objetivo.publish(m)

    def fk(self, q, largo3):
        extremo = 'tcp' if largo3 > self.L3 else 'brida'
        return self.cin.directa(q, extremo)


def main():
    rclpy.init()
    nodo = PanelBrazo()
    threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True).start()
    gazebo = nodo.modo == 'gazebo'

    raiz = tk.Tk()
    raiz.title('PLANTA BRAZO - panel de control ({})'.format(
        'Gazebo, consignas al controlador' if gazebo else 'RViz, cinematico'))
    raiz.configure(padx=12, pady=12)
    fila = 0

    tk.Label(raiz, text='CINEMATICA DIRECTA     angulos  ->  punto',
             font=('TkDefaultFont', 11, 'bold'), fg='#046').grid(
                 row=fila, column=0, sticky='w'); fila += 1

    escalas = []
    for nombre, lo, hi in JUNTAS:
        marco = tk.Frame(raiz)
        marco.grid(row=fila, column=0, sticky='we', pady=2); fila += 1
        tk.Label(marco, text=nombre, width=4,
                 font=('TkDefaultFont', 12, 'bold')).pack(side='left')
        s = tk.Scale(marco, from_=lo, to=hi, resolution=0.5, orient='horizontal',
                     length=400,
                     label='{}   [{:.0f}, {:.0f}] grados{}'.format(
                         nombre, lo, hi, '  (consigna)' if gazebo else ''))
        s.set(0.0)
        s.pack(side='left')
        escalas.append(s)

    traj = {'activa': False, 'i': 0, 't': 0.0}
    DT = 0.05

    marco_vel = tk.Frame(raiz)
    marco_vel.grid(row=fila, column=0, sticky='we', pady=(6, 0)); fila += 1
    tk.Label(marco_vel, text='Tiempo por tramo').pack(side='left')
    vel = tk.Scale(marco_vel, from_=0.5, to=6.0, resolution=0.1,
                   orient='horizontal', length=250)
    vel.set(2.5 if gazebo else 2.0)
    vel.pack(side='left', padx=(6, 0))
    tk.Label(marco_vel, text='s').pack(side='left')

    estado = tk.Label(raiz, text='detenido', fg='#555')
    estado.grid(row=fila, column=0, sticky='w'); fila += 1

    ttk.Separator(raiz, orient='horizontal').grid(
        row=fila, column=0, sticky='we', pady=8); fila += 1
    tk.Label(raiz, text='CINEMATICA INVERSA     punto  ->  angulos',
             font=('TkDefaultFont', 11, 'bold'), fg='#640').grid(
                 row=fila, column=0, sticky='w'); fila += 1

    marco_ci = tk.Frame(raiz)
    marco_ci.grid(row=fila, column=0, sticky='we', pady=(2, 0)); fila += 1
    entradas = {}
    for eje in ('X', 'Y', 'Z'):
        tk.Label(marco_ci, text=f'  {eje}').pack(side='left')
        e = tk.Entry(marco_ci, width=8, justify='right')
        e.pack(side='left')
        entradas[eje] = e
    tk.Label(marco_ci, text='mm   postura').pack(side='left', padx=(4, 4))
    postura = ttk.Combobox(marco_ci, values=list(POSTURAS), width=14,
                           state='readonly')
    postura.set('codo arriba')
    postura.pack(side='left')

    marco_ci2 = tk.Frame(raiz)
    marco_ci2.grid(row=fila, column=0, sticky='we', pady=(4, 0)); fila += 1

    aviso = tk.Label(raiz, text='', font=('TkFixedFont', 10), justify='left',
                     fg='#555')
    aviso.grid(row=fila, column=0, sticky='w', pady=(4, 0)); fila += 1

    def poner_punto(p):
        for eje, v in zip('XYZ', p):
            entradas[eje].delete(0, 'end')
            entradas[eje].insert(0, '%.1f' % (v * 1000.0))

    def tomar_actual():
        poner_punto(nodo.fk(nodo.enlace.real if gazebo else nodo.q,
                            nodo.L3 + nodo.Lg))

    def ir_al_punto():
        traj['activa'] = False
        boton_traj.config(text='Trayectoria')
        try:
            p = tuple(float(entradas[e].get()) / 1000.0 for e in 'XYZ')
        except ValueError:
            aviso.config(text='  el punto tiene que ser tres numeros en mm',
                         fg='#b00')
            return
        q_ahora = nodo.enlace.real if gazebo else list(nodo.q)
        sol, todas = nodo.cin.resolver(p, 'tcp', POSTURAS[postura.get()],
                                       q_actual=q_ahora)
        if sol is None:
            if not todas:
                rmin, rmax = nodo.cin.alcance('tcp')
                aviso.config(
                    text='  punto FUERA DEL ALCANCE: no hay ninguna solucion.\n'
                         '  distancia al hombro admisible %.0f a %.0f mm'
                         % (rmin * 1000, rmax * 1000), fg='#b00')
            else:
                juntas = sorted({j for s in todas for j in s['fuera']})
                aviso.config(
                    text='  hay %d solucion(es) geometricas pero ninguna cabe en '
                         'los limites\n  articulares (topa en %s)'
                         % (len(todas), ', '.join(juntas)), fg='#b00')
            return
        nodo.objetivo = p
        nodo.marcar_objetivo(p)
        for s, v in zip(escalas, sol['q']):
            s.set(round(math.degrees(v), 1))
        aviso.config(
            text='  CI resuelta: q = [%.2f, %.2f, %.2f] grados   postura %s\n'
                 '  ese punto tenia %d solucion(es) geometrica(s), %d dentro de '
                 'limites%s'
                 % (*[math.degrees(v) for v in sol['q']], sol['config'],
                    len(todas), sum(s['valida'] for s in todas),
                    '   [SINGULAR: brazo estirado o plegado]'
                    if sol['singular'] else ''),
            fg='#070')

    tk.Button(marco_ci2, text='Ir al punto (inversa)', width=20,
              command=ir_al_punto).pack(side='left')
    tk.Button(marco_ci2, text='Copiar TCP actual', width=18,
              command=tomar_actual).pack(side='left', padx=6)
    tk.Label(marco_ci2, text='punto de prueba').pack(side='left', padx=(10, 4))
    nombres = [n for n, _ in PUNTOS_PRUEBA_RAD]
    combo_p = ttk.Combobox(marco_ci2, values=nombres, width=24, state='readonly')
    combo_p.pack(side='left')

    def elegir_prueba(_evt=None):
        i = nombres.index(combo_p.get())
        poner_punto(nodo.cin.directa(PUNTOS_PRUEBA_RAD[i][1], 'tcp'))
        aviso.config(text='  punto cargado; pulsa "Ir al punto (inversa)"',
                     fg='#555')
    combo_p.bind('<<ComboboxSelected>>', elegir_prueba)

    ttk.Separator(raiz, orient='horizontal').grid(
        row=fila, column=0, sticky='we', pady=8); fila += 1

    def home():
        traj['activa'] = False
        boton_traj.config(text='Trayectoria')
        nodo.objetivo = None
        nodo.marcar_objetivo(None)
        for s in escalas:
            s.set(0.0)

    def alternar():
        traj['activa'] = not traj['activa']
        if traj['activa']:
            nodo.objetivo = None
            nodo.marcar_objetivo(None)
            traj['i'], traj['t'] = 0, 0.0
            for k, s in enumerate(escalas):
                s.set(math.degrees(PUNTOS_RAD[0][k]))
        boton_traj.config(text='Detener' if traj['activa'] else 'Trayectoria')

    def borrar_traza():
        nodo.pub_limpiar.publish(Empty())

    barra = tk.Frame(raiz)
    barra.grid(row=fila, column=0, sticky='w', pady=(0, 4)); fila += 1
    boton_traj = tk.Button(barra, text='Trayectoria', width=12, command=alternar)
    boton_traj.pack(side='left')
    tk.Button(barra, text='Home (0,0,0)', width=12, command=home).pack(side='left', padx=6)
    tk.Button(barra, text='Borrar traza', width=12, command=borrar_traza).pack(side='left')
    tk.Button(barra, text='Salir', width=8, command=raiz.destroy).pack(side='left', padx=6)

    etiqueta = tk.Label(raiz, text='', font=('TkFixedFont', 11), justify='left')
    etiqueta.grid(row=fila, column=0, sticky='w', pady=(6, 0)); fila += 1

    def refrescar():
        if traj['activa']:
            T = float(vel.get())
            q = interpolar(traj['i'], traj['t'], T)
            for s, v in zip(escalas, q):
                s.set(round(math.degrees(v), 1))
            traj['t'] += DT
            if traj['t'] >= T:
                traj['t'] = 0.0
                traj['i'] = (traj['i'] + 1) % len(PUNTOS_RAD)
            estado.config(
                text=f"trayectoria: tramo {traj['i'] + 1}/{len(PUNTOS)}  ->  "
                     f"punto {PUNTOS[(traj['i'] + 1) % len(PUNTOS)]} grados",
                fg='#0a7')
        else:
            estado.config(text='manual (mueve los deslizadores)', fg='#555')

        nodo.q = [math.radians(s.get()) for s in escalas]

        if gazebo:
            q_real = nodo.enlace.real
            bx, by, bz = nodo.fk(q_real, nodo.L3)
            tx, ty, tz = nodo.fk(q_real, nodo.L3 + nodo.Lg)
            err = [math.degrees(e) for e in nodo.enlace.error]
            texto = (
                'MEDIDO en Gazebo   J1 {:7.1f}   J2 {:7.1f}   J3 {:7.1f}  grados\n'
                'error seguimiento  J1 {:7.2f}   J2 {:7.2f}   J3 {:7.2f}  grados\n'
                'brida (fin Brazo3)  X {:8.1f}  Y {:8.1f}  Z {:8.1f}  mm\n'
                'TCP  (dedos gripper) X {:7.1f}  Y {:8.1f}  Z {:8.1f}  mm'
            ).format(*[math.degrees(v) for v in q_real], *err,
                     bx * 1000, by * 1000, bz * 1000,
                     tx * 1000, ty * 1000, tz * 1000)
            if not nodo.enlace.hay_medida:
                texto = 'esperando el estado de Gazebo...\n' + texto
        else:
            q_real = nodo.q
            bx, by, bz = nodo.fk(nodo.q, nodo.L3)
            tx, ty, tz = nodo.fk(nodo.q, nodo.L3 + nodo.Lg)
            texto = (
                'brida (fin Brazo3)   X {:8.1f}   Y {:8.1f}   Z {:8.1f}  mm\n'
                'TCP  (dedos gripper) X {:8.1f}   Y {:8.1f}   Z {:8.1f}  mm'
            ).format(bx * 1000, by * 1000, bz * 1000,
                     tx * 1000, ty * 1000, tz * 1000)

        if nodo.objetivo is not None:
            d = math.dist((tx, ty, tz), nodo.objetivo)
            texto += ('\nobjetivo pedido a la CI  X {:7.1f}  Y {:8.1f}  '
                      'Z {:8.1f}  mm   ->  distancia {:5.2f} mm').format(
                          *[v * 1000 for v in nodo.objetivo], d * 1000)
        etiqueta.config(text=texto)
        raiz.after(int(DT * 1000), refrescar)

    tomar_actual()
    refrescar()
    try:
        raiz.mainloop()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
