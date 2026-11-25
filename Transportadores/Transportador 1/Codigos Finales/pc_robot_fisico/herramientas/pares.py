#!/usr/bin/env python3
"""
PAR DE GRAVEDAD en las juntas del robot FISICO, y comparacion con lo que
dan los motores a traves de la correa.

Ojo: este calculo usa la geometria REAL del robot (a2 = 150 mm,
a3 = 105 mm), que es bastante mas pequena que la del modelo de Gazebo
(146 y 286 mm).  El par sale, por tanto, bastante menor.

Lo unico que falta por medir son las MASAS.  Pesa cada pieza en una
balanza de cocina y mete los numeros en MASAS; mientras tanto van unas
estimaciones razonables para una estructura impresa en 3D de este tamano.

    python3 pares.py                 con las masas estimadas
    python3 pares.py 0.12 0.09 0.15  con las masas medidas (brazo2 brazo3 gripper)
"""
import math
import sys

import os
import sys

# config.py esta en la carpeta de arriba (pc_robot_fisico/)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import config as cfg  # noqa: E402

G = 9.80665

# masas ESTIMADAS [kg] - sustituir por las medidas
MASAS = {
    'brazo2': 0.130,     # hombro -> codo
    'brazo3': 0.095,     # codo -> brida
    'gripper': 0.150,    # efector completo
}
# centros de masa, fraccion de la longitud del eslabon desde su propia junta
CDM = {'brazo2': 0.50, 'brazo3': 0.50, 'gripper': 1.0}


def terminos(m2, m3, mg):
    """Descompone tau_J2 = g*(A*cos q2 + B*cos(q2+q3)).

    A  lo que gira solo con el hombro
    B  lo que gira con hombro y codo
    """
    a2, a3 = cfg.A2, cfg.A3
    c2 = CDM['brazo2'] * a2
    c3 = CDM['brazo3'] * a3
    cg = CDM['gripper'] * a3
    A = m2 * c2 + (m3 + mg) * a2
    B = m3 * c3 + mg * cg
    return A, B


def tau_j2(q2, q3, m2, m3, mg):
    A, B = terminos(m2, m3, mg)
    return G * (A * math.cos(q2) + B * math.cos(q2 + q3))


def tau_j3(q2, q3, m3, mg):
    _, B = terminos(0.0, m3, mg)
    return G * B * math.cos(q2 + q3)


def informe(m2, m3, mg):
    A, B = terminos(m2, m3, mg)
    nom = cfg.PAR_JUNTA_NOMINAL
    blo = cfg.PAR_JUNTA_BLOQUEO_EST

    print('Masas: brazo2 %.3f   brazo3 %.3f   gripper %.3f kg   '
          '(total en voladizo %.3f)' % (m2, m3, mg, m2 + m3 + mg))
    print('  A = %.5f kg.m    B = %.5f kg.m' % (A, B))
    print()
    print('Par disponible en la junta (correa %.0f:1, rendimiento %.2f):'
          % (cfg.N_CORREA, cfg.ETA_CORREA))
    print('  nominal  %.3f N.m      bloqueo (estimado)  %.3f N.m' % (nom, blo))
    print()
    print('Peor caso: brazo horizontal estirado (q2 = q3 = 0)')
    t2 = tau_j2(0.0, 0.0, m2, m3, mg)
    t3 = tau_j3(0.0, 0.0, m3, mg)
    for j, t in (('J2 hombro', t2), ('J3 codo', t3)):
        estado = ('OK' if t <= nom else
                  'por encima del nominal' if t <= blo else 'NO ALCANZA')
        print('  %-10s %6.3f N.m  = %4.0f %% del nominal, %4.0f %% del bloqueo'
              '   %s' % (j, t, t / nom * 100, t / blo * 100, estado))
    print()
    print('A lo largo del ciclo de transporte:')
    import trayectorias as tray
    peor2 = peor3 = 0.0
    for nombre, q in tray.CICLO:
        q2, q3 = math.radians(q[1]), math.radians(q[2])
        peor2 = max(peor2, abs(tau_j2(q2, q3, m2, m3, mg)))
        peor3 = max(peor3, abs(tau_j3(q2, q3, m3, mg)))
    print('  J2 maximo %.3f N.m (%3.0f %% del nominal)' % (peor2, peor2 / nom * 100))
    print('  J3 maximo %.3f N.m (%3.0f %% del nominal)' % (peor3, peor3 / nom * 100))
    return t2


def barrido():
    """Cuanta masa aguanta el hombro antes de quedarse corto."""
    nom = cfg.PAR_JUNTA_NOMINAL
    print('\nCuanto peso admite el hombro (brazo estirado, caso peor):')
    print('%10s %10s %10s %12s' % ('total [kg]', 'tau [N.m]', '% nominal', 'veredicto'))
    for escala in (0.6, 0.8, 1.0, 1.2, 1.5, 2.0):
        m2 = MASAS['brazo2'] * escala
        m3 = MASAS['brazo3'] * escala
        mg = MASAS['gripper'] * escala
        t = tau_j2(0.0, 0.0, m2, m3, mg)
        print('%10.3f %10.3f %9.0f %% %12s'
              % (m2 + m3 + mg, t, t / nom * 100,
                 'OK' if t <= nom else 'se queda corto'))


if __name__ == '__main__':
    print(cfg.resumen())
    print()
    if len(sys.argv) == 4:
        m2, m3, mg = (float(x) for x in sys.argv[1:4])
    else:
        m2, m3, mg = (MASAS['brazo2'], MASAS['brazo3'], MASAS['gripper'])
        print('[usando masas ESTIMADAS: pesalas y pasalas como argumentos]\n')
    informe(m2, m3, mg)
    barrido()
