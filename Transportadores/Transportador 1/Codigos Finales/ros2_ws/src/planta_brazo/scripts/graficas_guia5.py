#!/usr/bin/env python3
"""
GRAFICAS Y METRICAS DE LA GUIA 5 - control cinematico.

    python3 graficas_guia5.py registro.csv [-s salida]

Genera lo que pide la guia:

  1_trayectoria_3d.png   las TRES trayectorias superpuestas en 3D:
                         deseada (planeada), simulada por el control
                         cinematico, y real (la que ejecuta la fisica)
  2_articulaciones.png   las tres juntas contra el tiempo, planeada vs real
  3_errores.png          error del efector por eje y su norma, y error de
                         cada junta

Y por consola las metricas: maximo, RMS y final, del efector y de cada junta.

El CSV lo genera control_cinematico.py con guardar:=true.
"""
import argparse
import os

import numpy as np


def cargar(ruta):
    d = np.loadtxt(ruta, delimiter=',', skiprows=1)
    if d.shape[1] < 31:
        raise SystemExit(
            'El CSV tiene %d columnas y hacen falta 31.\n'
            'Es de una version anterior, sin la trayectoria simulada.\n'
            'Vuelve a lanzarlo con guardar:=true para regenerarlo.'
            % d.shape[1])
    return {
        't':      d[:, 0],
        'xd':     d[:, 1:4] * 1000.0,     # deseada [mm]
        'x_real': d[:, 4:7] * 1000.0,     # real, de la fisica [mm]
        'x_sim':  d[:, 25:28] * 1000.0,   # simulada por el control [mm]
        'q_real': np.degrees(d[:, 11:14]),
        'q_plan': np.degrees(d[:, 28:31]),
        'det':    d[:, 17],
    }


def metricas(e):
    """maximo, RMS y valor final de una serie de error."""
    return np.max(np.abs(e)), np.sqrt(np.mean(e ** 2)), e[-1]


def informe(D):
    en = np.linalg.norm(D['xd'] - D['x_real'], axis=1)
    es = np.linalg.norm(D['xd'] - D['x_sim'], axis=1)
    print('=' * 68)
    print('METRICAS DE ERROR   (%d muestras, %.1f s)' % (len(D['t']), D['t'][-1]))
    print('=' * 68)
    print()
    print('Posicion del efector [mm]')
    print('  %-34s %8s %8s %8s' % ('', 'maximo', 'RMS', 'final'))
    for nom, e in (('deseada vs REAL (fisica)', en),
                   ('deseada vs SIMULADA (control)', es),
                   ('simulada vs real', np.linalg.norm(D['x_sim'] - D['x_real'], axis=1))):
        print('  %-34s %8.2f %8.2f %8.2f' % ((nom,) + metricas(e)))
    print()
    print('  por eje, deseada vs real:')
    for i, n in enumerate('xyz'):
        print('    %-32s %8.2f %8.2f %8.2f'
              % (n, *metricas(D['xd'][:, i] - D['x_real'][:, i])))
    print()
    print('Articulaciones [grados]   (planeada vs real)')
    print('  %-34s %8s %8s %8s' % ('', 'maximo', 'RMS', 'final'))
    for i, n in enumerate(('J1', 'J2', 'J3')):
        print('  %-34s %8.2f %8.2f %8.2f'
              % ((n,) + metricas(D['q_plan'][:, i] - D['q_real'][:, i])))
    print()
    print('Jacobiano: |det J| minimo = %.3e  (cero seria singularidad)'
          % np.min(np.abs(D['det'])))
    return en, es


def graficar(D, base):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D            # noqa: F401

    t = D['t']

    # ------------------------------------------------- 1) trayectoria en 3D
    fig = plt.figure(figsize=(9, 7.5))
    a = fig.add_subplot(111, projection='3d')
    a.plot(*D['xd'].T,     lw=2.5, color='tab:blue',   label='deseada (planeada)')
    a.plot(*D['x_sim'].T,  lw=1.8, color='tab:orange', ls='--',
           label='simulada (control cinematico)')
    a.plot(*D['x_real'].T, lw=1.4, color='tab:red',    ls=':',
           label='real (ejecutada)')
    a.scatter(*D['xd'][0],  s=60, color='green', marker='o', label='inicio')
    a.scatter(*D['xd'][-1], s=60, color='black', marker='s', label='fin')
    a.set_xlabel('x [mm]'); a.set_ylabel('y [mm]'); a.set_zlabel('z [mm]')
    a.set_title('Trayectoria del efector en 3D')
    a.legend(loc='upper left', fontsize=9)
    a.view_init(elev=22, azim=-130)
    fig.tight_layout(); fig.savefig(base + '_1_trayectoria_3d.png', dpi=140)

    # ------------------------------------------------- 2) articulaciones
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, n in enumerate(('J1', 'J2', 'J3')):
        ax[i].plot(t, D['q_plan'][:, i], lw=2, color='tab:blue', label='planeada')
        ax[i].plot(t, D['q_real'][:, i], lw=1.4, color='tab:red', ls='--',
                   label='real')
        ax[i].set_ylabel('%s [grados]' % n); ax[i].grid(alpha=.3)
        ax[i].legend(fontsize=8, loc='best')
    ax[0].set_title('Articulaciones contra el tiempo')
    ax[2].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(base + '_2_articulaciones.png', dpi=140)

    # ------------------------------------------------- 3) errores
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, n in enumerate('xyz'):
        ax[0].plot(t, D['xd'][:, i] - D['x_real'][:, i], lw=1.3,
                   label='error ' + n)
    ax[0].axhline(0, color='k', lw=.6)
    ax[0].set_ylabel('error por eje [mm]'); ax[0].grid(alpha=.3)
    ax[0].legend(fontsize=8); ax[0].set_title('Error de posicion del efector')

    en = np.linalg.norm(D['xd'] - D['x_real'], axis=1)
    es = np.linalg.norm(D['xd'] - D['x_sim'], axis=1)
    ax[1].plot(t, en, lw=1.6, color='tab:red', label='deseada vs real')
    ax[1].plot(t, es, lw=1.4, color='tab:orange', ls='--',
               label='deseada vs simulada')
    ax[1].axhline(en.mean(), ls=':', color='gray', lw=1,
                  label='media aritmetica %.2f mm' % en.mean())
    ax[1].set_ylabel('norma del error [mm]'); ax[1].grid(alpha=.3)
    ax[1].legend(fontsize=8)

    for i, n in enumerate(('J1', 'J2', 'J3')):
        ax[2].plot(t, D['q_plan'][:, i] - D['q_real'][:, i], lw=1.3, label=n)
    ax[2].axhline(0, color='k', lw=.6)
    ax[2].set_ylabel('error por junta [grados]'); ax[2].set_xlabel('t [s]')
    ax[2].grid(alpha=.3); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(base + '_3_errores.png', dpi=140)

    print()
    for n in ('1_trayectoria_3d', '2_articulaciones', '3_errores'):
        print('  %s_%s.png' % (base, n))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv')
    ap.add_argument('-s', '--salida', default=None,
                    help='ruta base de las imagenes (por defecto, la del CSV)')
    a = ap.parse_args()
    base = a.salida or os.path.splitext(a.csv)[0]
    D = cargar(a.csv)
    informe(D)
    graficar(D, base)


if __name__ == '__main__':
    main()
