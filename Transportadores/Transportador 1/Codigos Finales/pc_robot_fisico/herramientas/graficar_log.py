#!/usr/bin/env python3
"""
Grafica lo que se ENVIA y lo que se LEE durante una ejecucion, y compara
varias ejecuciones entre si.

UNA SOLA CORRIDA
    python3 mover_robot_vel.py --trayectoria ciclo --log real.csv
    python3 graficar_log.py real.csv

    POSICION    lo planificado contra lo que miden los encoders
    VELOCIDAD   lo que se manda por el puerto contra lo que mide el robot
    SEGUIMIENTO el error y el factor de escalado de tiempo

CAMINO DEL EFECTOR  (lo que hace visible la cinematica inversa)
    python3 graficar_log.py --cartesiano recta.csv

COMPARAR DOS O MAS
    python3 graficar_log.py --comparar sim.csv real.csv
    python3 graficar_log.py --comparar trapez.csv cubico.csv

    Superpone las corridas y saca la tabla de diferencias.  Es la forma de
    enseñar simulacion contra robot real, o un perfil contra el otro.
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

JUNTAS = ('J1', 'J2', 'J3')
COLOR = {'J1': '#2a78d6', 'J2': '#eb6834', 'J3': '#1baf7a'}
plt.rcParams.update({
    'font.size': 8, 'axes.labelsize': 8, 'legend.fontsize': 7,
    'xtick.labelsize': 7, 'ytick.labelsize': 7,
    'axes.grid': True, 'grid.alpha': 0.25, 'grid.linewidth': 0.4,
    'axes.spines.top': False, 'axes.spines.right': False,
    'figure.dpi': 150, 'savefig.facecolor': 'white',
})


def leer(ruta):
    with open(ruta, newline='') as fh:
        filas = list(csv.DictReader(fh))
    return {k: [float(f[k]) for f in filas] for k in filas[0]}


def main(ruta, salida=None):
    d = leer(ruta)
    t = d['t_s']
    salida = salida or ruta.rsplit('.', 1)[0] + '.png'

    fig, ax = plt.subplots(3, 1, figsize=(8.5, 7.2), sharex=True)

    # --- POSICION: planificada contra medida ---
    for i, j in enumerate(JUNTAS, 1):
        ax[0].plot(t, d['qplan_deg_J%d' % i], color=COLOR[j], lw=1.6,
                   alpha=0.35, label='%s plan' % j)
        ax[0].plot(t, d['qmed_deg_J%d' % i], color=COLOR[j], lw=1.1,
                   label='%s medida' % j)
    ax[0].set_ylabel('Posición [°]')
    ax[0].legend(ncol=3, loc='upper right', framealpha=0.95, fontsize=6.5)
    ax[0].set_title('LO LEÍDO: posición de los encoders (línea fina) contra '
                    'el plan (línea gruesa clara)', fontsize=8, loc='left')

    # --- VELOCIDAD: enviada contra medida ---
    for i, j in enumerate(JUNTAS, 1):
        ax[1].plot(t, d['venv_deg_s_J%d' % i], color=COLOR[j], lw=1.3,
                   label='%s enviada' % j)
        ax[1].plot(t, d['vmed_deg_s_J%d' % i], color=COLOR[j], lw=0.8,
                   ls=':', alpha=0.8, label='%s medida' % j)
    ax[1].set_ylabel('Velocidad [°/s]')
    ax[1].legend(ncol=3, loc='upper right', framealpha=0.95, fontsize=6.5)
    ax[1].set_title('LO ENVIADO: consigna de velocidad por el puerto serie '
                    '(continua) y lo que mide el robot (punteada)',
                    fontsize=8, loc='left')

    # --- SEGUIMIENTO ---
    ax[2].plot(t, d['error'], color='#b4302a', lw=1.2, label='error')
    ax[2].set_ylabel('Error')
    ax[2].set_xlabel('Tiempo [s]')
    a2 = ax[2].twinx()
    a2.plot(t, d['factor'], color='#66748a', lw=1.0, ls='--',
            label='factor de tiempo')
    a2.set_ylabel('Factor de escalado', fontsize=8)
    a2.set_ylim(0, 1.05)
    a2.grid(False)
    l1, n1 = ax[2].get_legend_handles_labels()
    l2, n2 = a2.get_legend_handles_labels()
    ax[2].legend(l1 + l2, n1 + n2, loc='upper right', framealpha=0.95,
                 fontsize=6.5)
    ax[2].set_title('Error de seguimiento y cuánto se frena el plan para '
                    'esperar al robot', fontsize=8, loc='left')

    for a in ax:
        a.set_xlim(t[0], t[-1])
    fig.tight_layout(h_pad=0.7)
    fig.savefig(salida, bbox_inches='tight')
    print('grafica en', salida)

    # --- resumen numerico ---
    print()
    print('%-4s %12s %12s %12s' % ('', 'vel enviada', 'vel medida', 'error pos'))
    print('%-4s %12s %12s %12s' % ('', 'pico [°/s]', 'pico [°/s]', 'max [°]'))
    for i, j in enumerate(JUNTAS, 1):
        ve = max(abs(v) for v in d['venv_deg_s_J%d' % i])
        vm = max(abs(v) for v in d['vmed_deg_s_J%d' % i])
        ep = max(abs(a - b) for a, b in zip(d['qplan_deg_J%d' % i],
                                            d['qmed_deg_J%d' % i]))
        print('%-4s %12.1f %12.1f %12.1f' % (j, ve, vm, ep))
    print()
    print('ciclos registrados : %d  en %.1f s' % (len(t), t[-1]))
    print('factor de tiempo   : medio %.2f, minimo %.2f'
          % (sum(d['factor']) / len(d['factor']), min(d['factor'])))


def cartesiano(ruta, salida=None):
    """Grafica el camino del efector: donde se le pidio ir y donde fue.

    Es lo que hace visible la cinematica inversa: el plan esta en X, Y, Z
    y los angulos salen de resolverla.  La pseudoinversa del jacobiano
    convierte la velocidad cartesiana en velocidad de las juntas.
    """
    import sys as _s, os as _o
    _s.path.insert(0, _o.path.expanduser('~/robot_fisico'))
    import cinematica as cin
    import numpy as np

    d = leer(ruta)
    salida = salida or ruta.rsplit('.', 1)[0] + '_cartesiano.png'
    t = d['t_s']
    MM = 1000.0

    def tcp(pref):
        return np.array([cin.directa(np.radians([d['%s_J1' % pref][k],
                                                 d['%s_J2' % pref][k],
                                                 d['%s_J3' % pref][k]]))
                         for k in range(len(t))]) * MM

    p_plan = tcp('qplan_deg')
    p_med = tcp('qmed_deg')

    fig = plt.figure(figsize=(8.5, 7.4))
    g = fig.add_gridspec(3, 2, height_ratios=[1.25, 1, 1])

    # --- camino en planta ---
    a0 = fig.add_subplot(g[0, 0])
    a0.plot(p_plan[:, 0], p_plan[:, 1], color='#66748a', lw=2.2, alpha=.45,
            label='pedido')
    a0.plot(p_med[:, 0], p_med[:, 1], color='#2a78d6', lw=1.2, label='real')
    a0.set_xlabel('X [mm]'); a0.set_ylabel('Y [mm]')
    a0.set_title('Camino del efector, en planta', fontsize=8, loc='left')
    a0.set_aspect('equal', adjustable='datalim')
    a0.legend(fontsize=7, framealpha=.95)

    # --- camino en alzado ---
    a1 = fig.add_subplot(g[0, 1])
    r_plan = np.hypot(p_plan[:, 0], p_plan[:, 1])
    r_med = np.hypot(p_med[:, 0], p_med[:, 1])
    a1.plot(r_plan, p_plan[:, 2], color='#66748a', lw=2.2, alpha=.45)
    a1.plot(r_med, p_med[:, 2], color='#2a78d6', lw=1.2)
    a1.set_xlabel('Radio [mm]'); a1.set_ylabel('Z [mm]')
    a1.set_title('Camino del efector, en alzado', fontsize=8, loc='left')
    a1.set_aspect('equal', adjustable='datalim')

    # --- X, Y, Z contra el tiempo ---
    a2 = fig.add_subplot(g[1, :])
    for k, (n, c) in enumerate(zip('XYZ', ('#2a78d6', '#eb6834', '#1baf7a'))):
        a2.plot(t, p_plan[:, k], color=c, lw=2.0, alpha=.4)
        a2.plot(t, p_med[:, k], color=c, lw=1.1, label=n)
    a2.set_ylabel('Posición [mm]')
    a2.legend(ncol=3, fontsize=7, loc='upper right', framealpha=.95)
    a2.set_title('Coordenadas del efector: pedidas (gruesa clara) y reales',
                 fontsize=8, loc='left')

    # --- error cartesiano ---
    a3 = fig.add_subplot(g[2, :])
    err = np.linalg.norm(p_plan - p_med, axis=1)
    a3.plot(t, err, color='#b4302a', lw=1.2)
    a3.set_ylabel('Error [mm]'); a3.set_xlabel('Tiempo [s]')
    a3.set_title('Distancia entre donde debia estar la punta y donde estaba',
                 fontsize=8, loc='left')
    a3.annotate('medio %.1f mm   maximo %.1f mm' % (err.mean(), err.max()),
                xy=(.99, .88), xycoords='axes fraction', ha='right',
                fontsize=7.5,
                bbox=dict(facecolor='white', edgecolor='none', pad=1.5))

    for a in (a2, a3):
        a.set_xlim(t[0], t[-1])
    fig.tight_layout(h_pad=.9)
    fig.savefig(salida, bbox_inches='tight')
    print('grafica cartesiana en', salida)
    print('  error de posicion del efector: medio %.2f mm, maximo %.2f mm'
          % (err.mean(), err.max()))
    print('  recorrido pedido %.0f mm, recorrido real %.0f mm'
          % (np.linalg.norm(np.diff(p_plan, axis=0), axis=1).sum(),
             np.linalg.norm(np.diff(p_med, axis=0), axis=1).sum()))


def resumen(d, nombre):
    """Numeros de una corrida, para la tabla comparativa."""
    t = d['t_s']
    r = {'nombre': nombre, 'dur': t[-1], 'ciclos': len(t),
         'factor': sum(d['factor']) / len(d['factor']),
         'err_medio': sum(d['error']) / len(d['error']),
         'err_max': max(d['error'])}
    for i, j in enumerate(JUNTAS, 1):
        r['venv_' + j] = max(abs(v) for v in d['venv_deg_s_J%d' % i])
        r['vmed_' + j] = max(abs(v) for v in d['vmed_deg_s_J%d' % i])
        r['epos_' + j] = max(abs(a - b) for a, b in
                             zip(d['qplan_deg_J%d' % i], d['qmed_deg_J%d' % i]))
    return r


def comparar(rutas, salida='comparacion.png'):
    datos = [(r.rsplit('/', 1)[-1].rsplit('.', 1)[0], leer(r)) for r in rutas]
    estilos = ['-', '--', '-.', ':']

    fig, ax = plt.subplots(3, 1, figsize=(8.5, 7.6), sharex=False)

    for k, (nom, d) in enumerate(datos):
        ls = estilos[k % len(estilos)]
        t = d['t_s']
        for i, j in enumerate(JUNTAS, 1):
            ax[0].plot(t, d['qmed_deg_J%d' % i], color=COLOR[j], ls=ls,
                       lw=1.2, label='%s %s' % (j, nom) if i else None)
            ax[1].plot(t, d['vmed_deg_s_J%d' % i], color=COLOR[j], ls=ls,
                       lw=1.0)
        ax[2].plot(t, d['error'], ls=ls, lw=1.2, color='#b4302a',
                   label=nom)

    ax[0].set_ylabel('Posición medida [°]')
    ax[0].set_title('Posición de las juntas en cada corrida', fontsize=8,
                    loc='left')
    ax[0].legend(ncol=3, loc='upper right', fontsize=6, framealpha=0.95)
    ax[1].set_ylabel('Velocidad medida [°/s]')
    ax[1].set_title('Velocidad que alcanza el robot', fontsize=8, loc='left')
    ax[2].set_ylabel('Error de seguimiento')
    ax[2].set_xlabel('Tiempo [s]')
    ax[2].set_title('Error en cada corrida', fontsize=8, loc='left')
    ax[2].legend(loc='upper right', fontsize=7, framealpha=0.95)
    fig.tight_layout(h_pad=0.8)
    fig.savefig(salida, bbox_inches='tight')
    print('grafica comparativa en', salida)

    # ---- tabla ----
    res = [resumen(d, n) for n, d in datos]
    print()
    anchos = max(len(r['nombre']) for r in res) + 2
    def fila(tit, clave, fmt='%8.1f'):
        print(('%-22s' % tit) + ''.join((fmt % r[clave]).rjust(anchos)
                                        for r in res))
    print(('%-22s' % '') + ''.join(r['nombre'].rjust(anchos) for r in res))
    print('-' * (22 + anchos * len(res)))
    fila('duracion [s]', 'dur', '%8.1f')
    fila('ciclos', 'ciclos', '%8d')
    fila('factor de tiempo', 'factor', '%8.2f')
    fila('error medio', 'err_medio', '%8.2f')
    fila('error maximo', 'err_max', '%8.2f')
    print()
    for j in JUNTAS:
        fila('%s vel enviada pico' % j, 'venv_' + j)
        fila('%s vel medida pico' % j, 'vmed_' + j)
        fila('%s error de posicion' % j, 'epos_' + j)
        print()
    if len(res) == 2:
        a, b = res
        print('Diferencias de %s respecto a %s:' % (b['nombre'], a['nombre']))
        print('  tarda %+.0f %% mas' % ((b['dur'] / a['dur'] - 1) * 100))
        if a['err_medio'] > 1e-9:
            print('  error medio %+.0f %%'
                  % ((b['err_medio'] / a['err_medio'] - 1) * 100))
        for j in JUNTAS:
            if a['vmed_' + j] > 1e-6:
                print('  %s alcanza el %.0f %% de la velocidad de %s'
                      % (j, b['vmed_' + j] / a['vmed_' + j] * 100, a['nombre']))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--cartesiano' in sys.argv:
        if not args:
            print('falta el fichero'); sys.exit(1)
        cartesiano(args[0], args[1] if len(args) > 1 else None)
    elif '--comparar' in sys.argv:
        if len(args) < 2:
            print('hacen falta al menos dos ficheros')
            sys.exit(1)
        comparar(args)
    elif args:
        main(args[0], args[1] if len(args) > 1 else None)
    else:
        print(__doc__)
        sys.exit(1)
