#!/usr/bin/env python3
"""
EJECUCION DE LA TRAYECTORIA EN EL ROBOT FISICO  -  control cinematico

Reparto del trabajo, tal y como pidio la profesora:

    PC (este programa)                 ESP32 (control_vel.py)
    --------------------------------   ----------------------------
    trayectoria + perfil de velocidad  lazo de VELOCIDAD por motor
    cinematica inversa diferencial     (el controlador del Simulink)
    correccion de posicion             PWM y sentido del L298N
         |  "v1,v2,v3,servo" cada 50 ms
         +------------------------------>

Dos modos:

  ciclo    los 14 puntos de paso en el espacio de juntas.  La correccion
           se hace en juntas:   qd_cmd = qd_deseada + K*(q_deseada - q_medida)

  recta    el segmento A-B en cartesiano.  Es el esquema de control
           cinematico de los apuntes, con el jacobiano:
               xe_punto = xd_punto + K*(xd - x)
               q_punto  = pinv(J) * xe_punto

Uso:
    python3 mover_robot_vel.py --solo-plan
    python3 mover_robot_vel.py --simulado
    python3 mover_robot_vel.py                       ciclo + trapezoidal
    python3 mover_robot_vel.py --trayectoria recta
    python3 mover_robot_vel.py --vel 60 --acel 100
"""
import argparse
import math
import sys
import time

import numpy as np

import cinematica as cin
import config as cfg
import perfiles
import trayectorias as tray
from enlace_esp import EnlaceESP, Reloj

# Velocidad maxima fiable de cada junta, MEDIDA con barrido() en el robot.
# Por encima de esto el lazo no sigue; por debajo de unos 20-30 deg/s hay
# stick-slip, pero el lazo exterior de posicion lo va corrigiendo.
VEL_MAX_JUNTA = {'J1': 120.0, 'J2': 40.0, 'J3': 60.0}

K_JUNTAS = 2.0       # ganancia de la correccion en el espacio de juntas [1/s]
K_CART = 2.0         # idem en cartesiano

# ESCALADO DE TIEMPO
# El plan avanzaba con el reloj de pared pasara lo que pasara: si el robot
# se quedaba atras, la trayectoria seguia corriendo y el error ya no se
# recuperaba.  Con esto el reloj de la trayectoria se FRENA cuando el error
# crece, y el robot recorre el CAMINO con fidelidad aunque tarde mas.
# Es lo que se hace en los robots reales que no pueden seguir el ritmo.
ERR_REF_JUNTAS = 4.0      # grados de error a los que el plan va a media velocidad
ERR_REF_CART = 0.015      # metros, idem
FACTOR_MIN = 0.12         # nunca se para del todo


def leer_postura(enlace, intentos=40, por_defecto=None):
    """Lee donde esta el brazo AHORA, antes de planificar nada."""
    if enlace.simulado:
        return np.radians(cfg.POSE_INICIAL)
    for _ in range(intentos):
        tel = enlace.leer_telemetria()
        if tel:
            return np.radians(tel['med'])
        time.sleep(0.05)
    return None


def plan_juntas(hitos, tipo, vel, acel):
    """Muestrea la trayectoria articular: posicion y velocidad deseadas."""
    v, a = math.radians(vel), math.radians(acel)
    muestras = []
    t0 = 0.0
    dt = cfg.PERIODO
    for k in range(len(hitos) - 1):
        qa, qb = np.asarray(hitos[k]), np.asarray(hitos[k + 1])
        dq = qb - qa
        dom = dq[int(np.argmax(np.abs(dq)))]
        if abs(dom) < 1e-9:
            perfil = perfiles.Cubico(0.4)
        elif tipo == 'cubico':
            perfil = perfiles.Cubico(perfiles.tf_cubico(dom, v, a))
        else:
            tf, tc = perfiles.Trapezoidal.tiempo_minimo(dom, v, a)
            perfil = perfiles.Trapezoidal(tf, tc)
        n = max(int(round(perfil.tf / dt)), 2)
        for m in range(n):
            t = m * dt
            muestras.append({'t': t0 + t,
                             'q': qa + perfil.s(t) * dq,
                             'qd': perfil.sd(t) * dq,
                             'tramo': k})
        t0 += perfil.tf
    muestras.append({'t': t0, 'q': np.asarray(hitos[-1]),
                     'qd': np.zeros(3), 'tramo': len(hitos) - 2})
    return muestras


def revisar(muestras):
    """Comprueba que no se pidan velocidades que el robot no da."""
    avisos = []
    vmax = np.zeros(3)
    for m in muestras:
        vmax = np.maximum(vmax, np.abs(np.degrees(m['qd'])))
    for i, (j, v) in enumerate(zip(('J1', 'J2', 'J3'), vmax)):
        tope = VEL_MAX_JUNTA[j]
        if v > tope:
            avisos.append('%s pide %.0f deg/s y solo sigue hasta %.0f'
                          % (j, v, tope))
    return avisos, vmax


def ejecutar(muestras, enlace, servo_por_tramo, repetir, cartesiano):
    """Lazo de control cinematico: corrige posicion y manda velocidad."""
    reloj = Reloj(cfg.PERIODO)
    n = len(muestras)
    servo = cfg.SERVO_NEUTRO
    q_med = np.asarray(muestras[0]['q'])        # hasta que llegue telemetria
    factor = 1.0
    errores = []
    sin_tel = 0
    # En modo simulado no hay telemetria: se integra la velocidad mandada
    # para tener una medida falsa pero coherente, y asi el ensayo en seco
    # comprueba el lazo de verdad en vez de ver el error crecer sin fin.
    simulado = enlace.simulado

    err_ref = ERR_REF_CART if cartesiano else ERR_REF_JUNTAS
    try:
        for vuelta in range(repetir):
            if repetir > 1:
                print('\n--- vuelta %d de %d ---' % (vuelta + 1, repetir))
            k = 0.0                    # indice "virtual" en el plan
            i = 0
            while k < n - 1:
                m = muestras[int(k)]
                tel = enlace.leer_telemetria()
                if tel:
                    q_med = np.radians(tel['med'])
                elif not simulado:
                    sin_tel += 1

                servo = servo_por_tramo.get(m['tramo'], servo)

                if cartesiano:
                    x_d = cin.directa(m['q'])
                    xd_d = cin.jacobiano(m['q']) @ m['qd']
                    x = cin.directa(q_med)
                    xe = xd_d * factor + K_CART * (x_d - x)
                    qd = np.linalg.pinv(cin.jacobiano(q_med)) @ xe
                    err = float(np.linalg.norm(x_d - x))
                else:
                    qd = m['qd'] * factor + K_JUNTAS * (m['q'] - q_med)
                    err = float(np.max(np.abs(np.degrees(m['q'] - q_med))))
                errores.append(err)

                # el reloj del plan se frena si el robot se queda atras
                factor = 1.0 / (1.0 + err / err_ref)
                if factor < FACTOR_MIN:
                    factor = FACTOR_MIN
                k += factor

                g = np.degrees(qd)
                for c, j in enumerate(('J1', 'J2', 'J3')):
                    t = VEL_MAX_JUNTA[j]
                    g[c] = max(-t, min(t, g[c]))

                enlace.enviar(g, servo)
                if simulado:
                    q_med = q_med + np.radians(g) * cfg.PERIODO
                reloj.esperar()

                if i % 20 == 0:
                    print('  %5.1f%% t=%6.2f  x%.2f  q_med %7.1f %7.1f %7.1f  '
                          'v_cmd %6.1f %6.1f %6.1f  err %s'
                          % (k / n * 100, reloj.transcurrido, factor,
                             *np.degrees(q_med), *g,
                             ('%.1f mm' % (err * 1000)) if cartesiano
                             else ('%.1f deg' % err)))
                i += 1
    except KeyboardInterrupt:
        print('\n[!] interrumpido')
    finally:
        for _ in range(5):
            enlace.enviar([0.0, 0.0, 0.0], servo)
            time.sleep(cfg.PERIODO)
        if errores:
            # en cartesiano los errores van en metros; se pasan a mm
            esc = 1000.0 if cartesiano else 1.0
            print('\nerror de seguimiento: medio %.2f  maximo %.2f  %s'
                  % (sum(errores) / len(errores) * esc, max(errores) * esc,
                     'mm' if cartesiano else 'grados'))
        if sin_tel:
            print('[aviso] %d ciclos sin telemetria de %d' % (sin_tel, n))
        if reloj.retrasos:
            print('[aviso] %d de %d ciclos llegaron tarde'
                  % (reloj.retrasos, reloj.k))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trayectoria', choices=('ciclo', 'recta'), default='ciclo')
    ap.add_argument('--perfil', choices=('trapezoidal', 'cubico'),
                    default='trapezoidal')
    ap.add_argument('--vel', type=float, default=45.0,
                    help='velocidad de crucero de la junta dominante [deg/s]')
    ap.add_argument('--acel', type=float, default=80.0)
    ap.add_argument('--puntos', type=int, default=12)
    ap.add_argument('--repetir', type=int, default=1)
    ap.add_argument('--simulado', action='store_true')
    ap.add_argument('--solo-plan', action='store_true')
    args = ap.parse_args()

    print(cfg.resumen())

    try:
        if args.trayectoria == 'recta':
            hitos, _ = tray.recta_por_defecto(args.puntos)
            servo_por_tramo = {}
            cartesiano = True
        else:
            hitos, _ = tray.ciclo_juntas()
            servo_por_tramo = {k - 1: v for k, v in tray.SERVO_POR_PUNTO.items()}
            cartesiano = False
    except ValueError as e:
        print('\n[ERROR] trayectoria no viable: %s' % e)
        return 1

    muestras = plan_juntas(hitos, args.perfil, args.vel, args.acel)
    avisos, vmax = revisar(muestras)

    print('\nPLAN  %s, perfil %s, correccion en %s'
          % (args.trayectoria, args.perfil,
             'cartesiano (jacobiano)' if cartesiano else 'juntas'))
    print('  %d puntos de paso, %.1f s, %d consignas a %.0f ms'
          % (len(hitos), muestras[-1]['t'], len(muestras), cfg.PERIODO * 1000))
    print('  velocidad pico   J1 %5.1f   J2 %5.1f   J3 %5.1f  deg/s' % tuple(vmax))
    print('  tope medido      J1 %5.1f   J2 %5.1f   J3 %5.1f  deg/s'
          % tuple(VEL_MAX_JUNTA[j] for j in ('J1', 'J2', 'J3')))
    if avisos:
        print('\n[AVISO] el robot no sigue todo lo que se le pide:')
        for a in avisos:
            print('   -', a)
        print('   Se recortara al tope. Baja --vel para evitarlo.')
    else:
        print('  todo dentro de lo que el robot sigue: OK')

    if args.solo_plan:
        return 0

    print('\nIMPORTANTE: el brazo tiene que estar en la postura de arranque')
    print('  J1 = %.0f   J2 = %.0f   J3 = %.0f  grados' % cfg.POSE_INICIAL)

    try:
        enlace = EnlaceESP(simulado=args.simulado)
    except RuntimeError as e:
        print('\n[ERROR] %s' % e)
        return 1

    with enlace:
        # --- APROXIMACION ---
        # La trayectoria no tiene por que empezar donde esta el brazo: la
        # recta A-B arranca en una postura distinta a la de reposo.  Si se
        # lanza sin mas, el lazo ve de golpe 20 grados de error y pega un
        # tiron.  Por eso se lee la postura real y se anade un tramo suave
        # desde ahi hasta el primer punto.
        q0 = leer_postura(enlace)
        if q0 is None:
            print('[ERROR] no llega telemetria de la ESP32.')
            print('  Comprueba que no hay otro programa con el puerto'
                  ' (Thonny) y vuelve a intentarlo.')
            return 1
        salto = np.degrees(np.abs(q0 - np.asarray(muestras[0]['q'])))
        print('\nbrazo en  J1 %.1f  J2 %.1f  J3 %.1f  grados'
              % tuple(np.degrees(q0)))
        if salto.max() > 1.0:
            print('  el primer punto esta a %.0f, %.0f, %.0f grados: se'
                  ' anade aproximacion' % tuple(salto))
            aprox = plan_juntas([q0, np.asarray(muestras[0]['q'])],
                                args.perfil, min(args.vel, 25.0), args.acel)
            # el tiempo de la trayectoria se corre detras de la aproximacion
            t_ap = aprox[-1]['t']
            for m in muestras:
                m['t'] += t_ap
            muestras = aprox[:-1] + muestras
            print('  aproximacion de %.1f s, total %.1f s'
                  % (t_ap, muestras[-1]['t']))

        ejecutar(muestras, enlace, servo_por_tramo, args.repetir, cartesiano)
    print('\nhecho.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
