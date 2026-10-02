#!/usr/bin/env python3
"""
EJECUCION DE LA TRAYECTORIA EN EL ROBOT FISICO.

Junta las tres cosas que pide el proyecto:

    cinematica INVERSA   -> de los puntos cartesianos salen los angulos
    perfil TRAPEZOIDAL   -> o CUADRATICO (cubico), elegible
    envio a la ESP32     -> en grados de JUNTA; el firmware aplica la correa

Uso:
    python3 mover_robot.py                          ciclo + trapezoidal
    python3 mover_robot.py --trayectoria recta      recta A-B por inversa
    python3 mover_robot.py --perfil cubico
    python3 mover_robot.py --simulado               sin robot, solo imprime
    python3 mover_robot.py --repetir 3
    python3 mover_robot.py --vel 90 --acel 120

Antes de mover nada comprueba limites y alcance, e imprime el plan para que
se pueda revisar.  Con Ctrl+C para en seco y deja el robot quieto.
"""
import argparse
import math
import sys

import numpy as np

import cinematica as cin
import config as cfg
import perfiles
import trayectorias as tray
from enlace_esp import EnlaceESP, Reloj


# ------------------------------------------------------------------- plan
def planificar(hitos, tipo, vel, acel):
    """Muestrea la trayectoria al periodo del lazo.

    vel y acel en grados/s y grados/s2, referidos a la junta que mas se
    mueve en cada tramo (la dominante): es la que marca el ritmo y las
    otras se escalan con el mismo s(t), para que las tres lleguen a la vez.
    """
    v, a = math.radians(vel), math.radians(acel)
    muestras, cortes = [], []
    t0 = 0.0
    dt = cfg.PERIODO

    for k in range(len(hitos) - 1):
        qa, qb = np.asarray(hitos[k]), np.asarray(hitos[k + 1])
        dq = qb - qa
        dom = dq[int(np.argmax(np.abs(dq)))]

        if abs(dom) < 1e-9:
            perfil = perfiles.Cubico(0.3)
        elif tipo == 'cubico':
            perfil = perfiles.Cubico(perfiles.tf_cubico(dom, v, a))
        else:
            tf, tc = perfiles.Trapezoidal.tiempo_minimo(dom, v, a)
            perfil = perfiles.Trapezoidal(tf, tc)

        n = max(int(round(perfil.tf / dt)), 2)
        for m in range(n):
            t = m * dt
            s = perfil.s(t)
            muestras.append({'t': t0 + t, 'q': qa + s * dq, 'tramo': k,
                             'qd': perfil.sd(t) * dq})
        t0 += perfil.tf
        cortes.append(t0)

    muestras.append({'t': t0, 'q': np.asarray(hitos[-1]),
                     'tramo': len(hitos) - 2, 'qd': np.zeros(3)})
    return muestras, cortes


def revisar(muestras, nombres):
    """Comprueba limites y velocidades antes de mover el robot."""
    problemas = []
    vmax = np.zeros(3)
    for m in muestras:
        g = np.degrees(m['q'])
        fuera = cin.en_limites(g)
        if fuera:
            problemas.append('t=%.2f s: %s' % (m['t'], '; '.join(fuera)))
        vmax = np.maximum(vmax, np.abs(np.degrees(m['qd'])))
    for i, (j, v) in enumerate(zip(('J1', 'J2', 'J3'), vmax)):
        if v > cfg.JUNTA_VEL_MAX:
            problemas.append('%s pide %.0f deg/s, el motor da %.0f'
                             % (j, v, cfg.JUNTA_VEL_MAX))
    return problemas[:8], vmax


# --------------------------------------------------------------- ejecucion
def ejecutar(muestras, enlace, servo_por_tramo, repetir=1):
    reloj = Reloj(cfg.PERIODO)
    total = len(muestras)
    servo = cfg.SERVO_NEUTRO
    try:
        for vuelta in range(repetir):
            print('\n--- vuelta %d de %d ---' % (vuelta + 1, repetir))
            for i, m in enumerate(muestras):
                servo = servo_por_tramo.get(m['tramo'], servo)
                enlace.enviar(np.degrees(m['q']), servo)
                reloj.esperar()
                if i % 20 == 0:
                    tel = enlace.leer_telemetria()
                    extra = ''
                    if tel:
                        err = [r - q for r, q in zip(tel['ref'], tel['med'])]
                        extra = '   error medido %6.2f %6.2f %6.2f deg' % tuple(err)
                    print('  %5.1f %% | t=%6.2f s | q = %7.1f %7.1f %7.1f deg%s'
                          % (i / total * 100, m['t'], *np.degrees(m['q']), extra))
    except KeyboardInterrupt:
        print('\n[!] interrumpido: dejo el robot donde esta')
    finally:
        if reloj.retrasos:
            print('[aviso] %d de %d ciclos llegaron tarde (%.1f %%). Si es '
                  'mucho, sube PERIODO en config.py'
                  % (reloj.retrasos, reloj.k, reloj.retrasos / reloj.k * 100))


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trayectoria', choices=('ciclo', 'recta'), default='ciclo')
    ap.add_argument('--perfil', choices=('trapezoidal', 'cubico'),
                    default='trapezoidal')
    ap.add_argument('--vel', type=float, default=cfg.VEL_MAX,
                    help='velocidad de crucero [deg/s]')
    ap.add_argument('--acel', type=float, default=cfg.ACEL_MAX,
                    help='aceleracion de las rampas [deg/s2]')
    ap.add_argument('--puntos', type=int, default=12,
                    help='subdivisiones de la recta')
    ap.add_argument('--repetir', type=int, default=1)
    ap.add_argument('--simulado', action='store_true',
                    help='no abre el puerto: solo imprime')
    ap.add_argument('--solo-plan', action='store_true',
                    help='calcula y muestra el plan, no mueve nada')
    args = ap.parse_args()

    print(cfg.resumen())

    # ---- puntos de paso ----
    try:
        if args.trayectoria == 'recta':
            hitos, nombres = tray.recta_por_defecto(args.puntos)
            servo_por_tramo = {}
        else:
            hitos, nombres = tray.ciclo_juntas()
            servo_por_tramo = {k - 1: v
                               for k, v in tray.SERVO_POR_PUNTO.items()}
    except ValueError as e:
        print('\n[ERROR] la trayectoria no es viable:\n  %s' % e)
        return 1

    muestras, cortes = planificar(hitos, args.perfil, args.vel, args.acel)
    problemas, vmax = revisar(muestras, nombres)

    print('\nPLAN  trayectoria %s, perfil %s' % (args.trayectoria, args.perfil))
    print('  %d puntos de paso, %d tramos, %.2f s en total'
          % (len(hitos), len(hitos) - 1, muestras[-1]['t']))
    print('  %d consignas a %.0f ms' % (len(muestras), cfg.PERIODO * 1000))
    print('  velocidad pico   J1 %5.1f   J2 %5.1f   J3 %5.1f  deg/s' % tuple(vmax))
    print('  margen del motor J1 %4.0f%%   J2 %4.0f%%   J3 %4.0f%%  de %.0f deg/s'
          % (*(v / cfg.JUNTA_VEL_MAX * 100 for v in vmax), cfg.JUNTA_VEL_MAX))

    if problemas:
        print('\n[ERROR] el plan no es admisible:')
        for p in problemas:
            print('   -', p)
        return 1
    print('  limites y velocidades: OK')

    if args.solo_plan:
        return 0

    try:
        enlace = EnlaceESP(simulado=args.simulado)
    except RuntimeError as e:
        print('\n[ERROR] %s' % e)
        return 1

    with enlace:
        ejecutar(muestras, enlace, servo_por_tramo, args.repetir)
        # dejar el robot en el ultimo punto, sin corriente de arrastre
        enlace.enviar(np.degrees(hitos[-1]), cfg.SERVO_NEUTRO)
    print('\nhecho.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
