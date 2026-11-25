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
               q_punto  = pinv(J) * xe_punto      (ver jacobiano.py)

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
import jacobiano as jac
import perfiles
import trayectorias as tray
from enlace_esp import EnlaceESP, Reloj
from espejo_ros import EspejoROS
import estado_palas

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


def leer_postura(enlace, intentos=60, por_defecto=None):
    """Lee donde esta el brazo AHORA, antes de planificar nada.

    Exige DOS lecturas seguidas que coincidan en 1 grado.  El 2026-11-19 una
    sola linea mal leida al abrir el puerto dijo que el brazo estaba en
    (0, 0, 0) cuando estaba en el home, y el plan arranco desde ahi: el brazo
    salio a toda velocidad hacia una postura falsa.
    """
    if enlace.simulado:
        return np.radians(cfg.POSE_INICIAL)
    anterior = None
    for _ in range(intentos):
        tel = enlace.leer_telemetria()
        if tel:
            q = np.asarray(tel['med'])
            if anterior is not None and np.max(np.abs(q - anterior)) < 1.0:
                return np.radians(q)
            anterior = q
        time.sleep(0.05)
    return None


def anadir_aproximacion(muestras, q0, perfil, vel, acel):
    """Antepone un tramo suave desde donde ESTA el brazo hasta el primer punto.

    La trayectoria no tiene por que empezar donde esta el brazo.  Si se lanza
    sin mas, el lazo ve de golpe muchos grados de error y pega un tiron.  La
    usan main() y nodo_transportador1.py.
    """
    salto = np.degrees(np.abs(q0 - np.asarray(muestras[0]['q'])))
    print('\nbrazo en  J1 %.1f  J2 %.1f  J3 %.1f  grados' % tuple(np.degrees(q0)))
    if salto.max() <= 1.0:
        return muestras
    print('  el primer punto esta a %.0f, %.0f, %.0f grados: se anade aproximacion'
          % tuple(salto))
    aprox = plan_juntas([q0, np.asarray(muestras[0]['q'])],
                        perfil, min(vel, 25.0), acel)
    for a in aprox:          # no es un tramo de la trayectoria: que no
        a['tramo'] = -1      # dispare las palas del tramo 0
    # el tiempo de la trayectoria se corre detras de la aproximacion
    t_ap = aprox[-1]['t']
    for m in muestras:
        m['t'] += t_ap
    print('  aproximacion de %.1f s, total %.1f s' % (t_ap, muestras[-1]['t']))
    return aprox[:-1] + muestras


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


BANDA_J1 = 1.5       # grados.  Con J1 en espera (plan quieto en J1): dentro de
                     # la banda no se corrige; fuera, al menos V_MIN_J1.
V_MIN_J1 = 12.0      # deg/s = FF_RAMPA de J1 en el firmware.  Por debajo, la
                     # rampa de rozamiento (que evita el baile) no da fuerza
                     # para despegar, y J1 se quedaba con 2.5-3.9 grados de
                     # error FIJO: ~24 mm de desvio en la punta.  La banda
                     # muerta es la que impide que esto vuelva a ser el ciclo
                     # limite de antes.
TOL_LLEGADA = 2.5    # grados: el brazo "ha llegado" al punto si TODAS las
                     # juntas estan a menos de esto del punto...
V_LLEGADA = 3.0      # ...y se mueven a menos de esto (deg/s)
ESPERA_MAX = 6.0     # s: si no llega en este tiempo, se mueven las palas igual
                     # y se avisa, para no dejar la trayectoria colgada
SALTO_MAX = 15.0     # grados en un ciclo de 50 ms = 300 deg/s, el VEL_ABSURDA
                     # del firmware.  Mas que eso no es un movimiento, es una
                     # trama mal leida.


CIEGO_MAX = 10       # ciclos (0.5 s) sin una medida fiable -> se PARA todo


def _plausible(med_grados, q_med_rad):
    """Una junta no puede moverse mas de SALTO_MAX en un ciclo.

    El limite es FIJO a proposito.  Se probo hacerlo crecer con los ciclos sin
    dato (para no bloquearse si J3 va rapida y se pierden tramas) y era
    PEOR: una ESP32 reiniciada manda posiciones de home, y al cabo de unos
    ciclos el limite relajado las aceptaba como buenas (probado con una ESP32
    falsa: error de 164 grados).  Con el limite fijo, el peor caso es quedarse
    bloqueado, y eso lo convierte CIEGO_MAX en una parada segura.
    """
    return np.max(np.abs(np.asarray(med_grados)
                         - np.degrees(q_med_rad))) <= SALTO_MAX


def referenciar_palas(enlace, limite=12.0):
    """Pide referenciar las palas y ESPERA a que el firmware diga como fue.

    Mientras referencia, la ESP32 esta bloqueada (hasta ~9 s en el peor caso)
    y no manda telemetria.

    2026-11-24: se exige el mensaje '# gripper referenciado' del firmware.
    Antes bastaban dos tramas con el gripper cerca de 0, y si las palas NO SE
    MOVIAN la cuenta seguia en 0 desde el reinicio: un fallo pasaba por
    exito.  Y si quedaron FUERA la ultima vez (estado_palas.json), se pide el
    referenciado SOLO HACIA DENTRO, que si no encuentra el tope no las saca.
    """
    codigo, motivo = estado_palas.consigna_referenciar()
    print('referenciando las palas (%s)...' % motivo)
    ok0 = getattr(enlace, 'ref_ok', 0)
    f0 = getattr(enlace, 'ref_fallos', 0)
    enlace.enviar([0.0, 0.0, 0.0], gripper=codigo)
    t0 = time.time()
    buenas = 0
    while time.time() - t0 < limite:
        tel = enlace.leer_telemetria()          # tambien cuenta los mensajes
        if getattr(enlace, 'ref_fallos', 0) > f0:
            print('  [ERROR] el firmware NO encontro el tope en %.1f s: las palas no'
                  ' se movieron hacia dentro.' % (time.time() - t0))
            if codigo == cfg.GRIPPER_HOME_SIN_RETIRADA:
                print('  (no se sacaron: siguen donde estaban)')
            print('  Revisa el motor N20 y la cremallera antes de relanzar.')
            return False
        if (getattr(enlace, 'ref_ok', 0) > ok0 and tel and 'g' in tel
                and abs(tel['g']) <= 30):
            buenas += 1
            if buenas >= 2:
                print('  referenciadas en %.1f s, palas en %d cuentas'
                      % (time.time() - t0, tel['g']))
                estado_palas.guardar(tel['g'], 'referenciadas')
                return True
        time.sleep(0.05)
    print('  [aviso] el firmware no contesto al referenciado en %.0f s' % limite)
    return False


def mover_palas(enlace, apertura, simulado, tol=30, limite=4.0,
                sin_avance=2.0, avance_min=5):
    # Se da por FALLIDO si en `sin_avance` s no avanzan `avance_min` cuentas,
    # o si se pasa del `limite` absoluto.  Antes era un tiempo fijo de 2 s, y
    # el 2026-11-19 corto un movimiento que ya iba bien: recogiendo de debajo
    # de la caja estuvieron 1.7 s pegadas, se soltaron, y a los 2 s se paro
    # todo con las palas YA moviendose (455 449 391 351 269 231).
    # Arrastrandose avanzaban ~11 cuentas en 1.35 s: por eso 5 en 2 s.
    # tol tiene que ser MAYOR que la G_TOL del firmware (25): si no, el
    # firmware da las palas por llegadas y se para, y aqui se espera el
    # limite entero para nada.  Con 20 cada cierre costaba 3 s de pausa.
    """Para el brazo, mueve las palas, y espera a que lleguen.

    Se hace con el brazo QUIETO a proposito.  Moverlas durante la trayectoria
    (medido el 2026-11-13, misma trayectoria con y sin palas):

        tramas de telemetria perdidas   118 de 414   contra   34 de 414
        ciclos que llegaron tarde        51          contra    0
        error maximo                     19.17 deg   contra    7.44 deg

    y el pico de error caia exactamente en el instante en que arrancaba el
    motor del gripper.  La causa probable es ruido del motor sobre la linea
    serie; lo que lo mata en origen es un condensador de 100 nF entre los
    bornes del motor, que no tenemos.  Parar el brazo lo evita sin piezas, y
    ademas es como funciona cualquier pinza: no se abre la mano en marcha.
    """
    if simulado:
        return apertura, True
    print('   palas -> %d cuentas' % apertura)
    t0 = time.time()
    ultima = None
    traza = []                  # cuentas leidas, para ver COMO falla
    ref_avance = None           # cuentas y momento del ultimo avance real
    t_avance = t0
    while time.time() - t0 < limite:
        enlace.enviar([0.0, 0.0, 0.0], gripper=apertura)
        tel = enlace.leer_telemetria()
        if tel and 'g' in tel:
            ultima = tel['g']
            traza.append(int(ultima))
            if ref_avance is None or abs(ultima - ref_avance) >= avance_min:
                ref_avance = ultima
                t_avance = time.time()
            if abs(ultima - apertura) <= tol:
                print('      llegaron en %.2f s  (%d cuentas)'
                      % (time.time() - t0, ultima))
                return int(round(ultima)), True
        if time.time() - t_avance > sin_avance:
            print('      [aviso] %.1f s sin avanzar' % sin_avance)
            break
        time.sleep(cfg.PERIODO)
    # aqui solo se llega si NO llegaron (por tiempo o por falta de avance)
    print('      [aviso] no llegaron en %.1f s  (%s cuentas)'
          % (time.time() - t0, '?' if ultima is None else '%d' % ultima))
    print('      traza: %s' % ' '.join(str(c) for c in traza))
    if ultima is None:          # sin dato: se deja el objetivo, no se inventa
        return apertura, False
    # Aparcarlas DONDE ESTAN: si se sigue mandando el objetivo original, el
    # firmware (tolerancia 25 cuentas) sigue persiguiendolo el resto de la
    # trayectoria, alternando empuje y freno cada ciclo.
    return int(round(ultima)), False


def ejecutar(muestras, enlace, servo_por_tramo, repetir, cartesiano,
             espejo=None, log=None, apertura_por_tramo=None,
             cancelar=None):
    """Lazo de control cinematico: corrige posicion y manda velocidad.

    `apertura_por_tramo`: {indice de tramo -> apertura de las palas en
    cuentas}.  Se manda en el quinto campo de la trama.  El indice es el del
    PUNTO menos uno, igual que hacia el servo, asi que la orden se da durante
    el tramo que LLEVA a ese punto: las palas llegan ya colocadas.
    """
    reloj = Reloj(cfg.PERIODO)
    n = len(muestras)
    servo = cfg.SERVO_NEUTRO
    apertura = None                 # None = no se manda, el gripper no se toca
    objetivo_palas = None           # ultimo valor leido de la tabla
    pendiente = None                # apertura esperando a que el brazo llegue
    abortado = False                # las palas fallaron: se para todo
    ciegos = 0                      # ciclos seguidos sin medida fiable
    reinicios0 = getattr(enlace, 'reinicios', 0)
    t_espera = 0.0
    g_ultimo = None                 # ultima cuenta de palas medida YA referenciada

    # Sin referenciar, las cuentas del gripper van sobre un cero arbitrario y
    # el firmware no aplica sus limites.  Asi que si esta trayectoria manda
    # aperturas, primero se referencia contra el tope de entrada.
    if apertura_por_tramo and not enlace.simulado:
        if not referenciar_palas(enlace):
            print('[ERROR] las palas no se referenciaron: no sigo, porque sin')
            print('  referencia el firmware no aplica los limites de las palas.')
            return True        # PARADO: antes devolvia None y se tomaba por "hecho"
    q_med = np.asarray(muestras[0]['q'])        # hasta que llegue telemetria
    factor = 1.0
    errores = []
    sin_tel = 0
    descartadas = 0          # tramas que saltaban mas de lo posible
    # En modo simulado no hay telemetria: se integra la velocidad mandada
    # para tener una medida falsa pero coherente, y asi el ensayo en seco
    # comprueba el lazo de verdad en vez de ver el error crecer sin fin.
    simulado = enlace.simulado

    err_ref = ERR_REF_CART if cartesiano else ERR_REF_JUNTAS
    # registro de TODO lo que se manda y se lee, para poder graficarlo
    filas = []
    try:
        for vuelta in range(repetir):
            if abortado:
                break
            if repetir > 1:
                print('\n--- vuelta %d de %d ---' % (vuelta + 1, repetir))
            k = 0.0                    # indice "virtual" en el plan
            i = 0
            while k < n - 1:
                if cancelar is not None and cancelar.is_set():
                    # /parada del nodo maestro (nodo_transportador1.py)
                    print('\n[PARADA] cancelado desde fuera.  El brazo se'
                          ' queda quieto.')
                    abortado = True
                    break
                m = muestras[int(k)]
                tel = enlace.leer_telemetria()
                if tel and _plausible(tel['med'], q_med):
                    q_med = np.radians(tel['med'])
                    ciegos = 0
                elif tel:
                    descartadas += 1
                    ciegos += 1
                elif not simulado:
                    sin_tel += 1
                    ciegos += 1
                if getattr(enlace, 'reinicios', 0) > reinicios0:
                    print()
                    print('[PARADA] la ESP32 se ha REINICIADO a mitad de la'
                          ' trayectoria.  El brazo se queda quieto.')
                    print('  Ultima posicion conocida: J1 %.1f  J2 %.1f  J3 %.1f'
                          % tuple(np.degrees(q_med)))
                    print('  Ahora cree que el brazo esta en el home: NO lances'
                          ' nada sin llevarlo antes al home de verdad.')
                    abortado = True
                    break
                if ciegos > CIEGO_MAX:
                    # NUNCA seguir mandando ordenes sin saber donde esta el
                    # brazo.  Se para aqui.
                    print()
                    print('[PARADA] %.1f s sin una medida fiable de la ESP32.'
                          '  El brazo se queda quieto.' % (ciegos * cfg.PERIODO))
                    print('  Ultima posicion conocida: J1 %.1f  J2 %.1f  J3 %.1f'
                          % tuple(np.degrees(q_med)))
                    print('  Si la ESP32 se reinicio, ahora cree que el brazo esta'
                          ' en el home: NO lances nada sin comprobarlo.')
                    abortado = True
                    break

                servo = servo_por_tramo.get(m['tramo'], servo)
                if apertura_por_tramo:
                    # objetivo = lo que dice la tabla;  apertura = lo que se
                    # manda, que tras llegar es la posicion MEDIDA.  Son dos
                    # cosas distintas: si se comparase la tabla con lo
                    # mandado, el tramo volveria a disparar en cada ciclo.
                    #
                    # 2026-11-19: las palas se mueven AL LLEGAR al punto, y solo
                    # cuando el brazo MEDIDO esta ahi (ver mas abajo).  Antes se
                    # movian al EMPEZAR el tramo que llevaba al punto, o sea
                    # antes de llegar: la apertura de P1 se ejecutaba en P0.
                    nuevo = apertura_por_tramo.get(m['tramo'], objetivo_palas)
                    if nuevo != objetivo_palas and pendiente is None:
                        pendiente = nuevo
                        t_espera = time.time()

                if cartesiano:
                    x_d = cin.directa(m['q'])
                    xd_d = jac.vel_tcp(m['q'], m['qd'])
                    x = cin.directa(q_med)
                    xe = xd_d * factor + K_CART * (x_d - x)
                    # jacobiano INVERSO: velocidad del TCP -> velocidad de juntas
                    qd = jac.vel_juntas(q_med, xe)
                    err = float(np.linalg.norm(x_d - x))
                else:
                    qd = m['qd'] * factor + K_JUNTAS * (m['q'] - q_med)
                    # J1 en espera: banda muerta + correccion minima
                    if abs(m['qd'][0]) < 1e-6:
                        e1 = math.degrees(m['q'][0] - q_med[0])
                        if abs(e1) <= BANDA_J1:
                            qd[0] = 0.0
                        else:
                            v1 = max(K_JUNTAS * abs(e1), V_MIN_J1)
                            qd[0] = math.radians(v1 if e1 > 0 else -v1)
                    err = float(np.max(np.abs(np.degrees(m['q'] - q_med))))
                errores.append(err)

                # el reloj del plan se frena si el robot se queda atras
                factor = 1.0 / (1.0 + err / err_ref)
                if factor < FACTOR_MIN:
                    factor = FACTOR_MIN
                if pendiente is None:
                    k += factor
                else:
                    # El plan se queda QUIETO en el primer punto del tramo, que
                    # es el punto de llegada con velocidad cero: el lazo de
                    # posicion lleva el brazo hasta ahi.  Las palas no se mueven
                    # hasta que el brazo esta en su sitio y quieto.
                    vmed = np.abs(tel['vel']) if tel else np.zeros(3)
                    llegado = (not cartesiano and err < TOL_LLEGADA
                               and float(np.max(vmed)) < V_LLEGADA)
                    espera = time.time() - t_espera
                    if llegado or espera > ESPERA_MAX or simulado:
                        if not llegado and not simulado:
                            print('   [aviso] el brazo no llego en %.0f s (error'
                                  ' %.1f grados); muevo las palas igual'
                                  % (ESPERA_MAX, err))
                        else:
                            print('   llego a P%d en %.2f s (error %.1f grados)'
                                  % (m['tramo'], espera, err))
                        objetivo_palas = pendiente
                        pendiente = None
                        apertura, ok = mover_palas(enlace, objetivo_palas,
                                                   simulado)
                        if not ok:
                            # 2026-11-19, pedido por Julian: si las palas no
                            # llegan, la trayectoria NO sigue.  Antes seguia, y
                            # el brazo giro hacia la caja y se traslado con las
                            # palas a medio sacar (408 de 510).  Se para AQUI,
                            # sin volver al home: un movimiento automatico con
                            # las palas en un estado desconocido es peor.
                            print()
                            print('[PARADA] las palas no llegaron a %d (estan en'
                                  ' %d).  El brazo se queda quieto donde esta.'
                                  % (objetivo_palas, apertura))
                            print('  Revisa que nada las bloquee antes de volver'
                                  ' a lanzar.')
                            abortado = True
                            break

                g = np.degrees(qd)
                for c, j in enumerate(('J1', 'J2', 'J3')):
                    t = VEL_MAX_JUNTA[j]
                    g[c] = max(-t, min(t, g[c]))

                v_med = np.array(tel['vel']) if tel else np.zeros(3)
                if apertura_por_tramo and not simulado and tel and 'g' in tel:
                    g_ultimo = tel['g']
                filas.append({
                    't': reloj.transcurrido,
                    'q_plan': np.degrees(m['q']).tolist(),      # lo planificado
                    'qd_plan': np.degrees(m['qd']).tolist(),
                    'v_env': list(g),                           # lo ENVIADO
                    'q_med': np.degrees(q_med).tolist(),        # lo LEIDO
                    'v_med': v_med.tolist(),
                    'err': err, 'factor': factor, 'servo': servo,
                    'apertura': apertura if apertura is not None else -1,
                    'g_med': tel['g'] if tel and 'g' in tel else -1,
                })
                enlace.enviar(g, servo, gripper=apertura)
                if espejo is not None:
                    espejo.publicar(q_med)
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
            enlace.enviar([0.0, 0.0, 0.0], servo, gripper=apertura)
            time.sleep(cfg.PERIODO)
        if g_ultimo is not None:
            estado_palas.guardar(g_ultimo, 'fin de trayectoria')
            print('palas: quedan en %d cuentas (apuntado en estado_palas.json)' % g_ultimo)
        if errores:
            # en cartesiano los errores van en metros; se pasan a mm
            esc = 1000.0 if cartesiano else 1.0
            print('\nerror de seguimiento: medio %.2f  maximo %.2f  %s'
                  % (sum(errores) / len(errores) * esc, max(errores) * esc,
                     'mm' if cartesiano else 'grados'))
        if sin_tel:
            print('[aviso] %d ciclos sin telemetria de %d' % (sin_tel, n))
        if descartadas:
            print('[aviso] %d tramas descartadas por saltar mas de %.0f grados'
                  ' en un ciclo' % (descartadas, SALTO_MAX))
        if reloj.retrasos:
            print('[aviso] %d de %d ciclos llegaron tarde'
                  % (reloj.retrasos, reloj.k))
        if log and filas:
            import csv
            with open(log, 'w', newline='') as fh:
                w = csv.writer(fh)
                cab = ['t_s']
                for b in ('qplan_deg', 'qdplan_deg_s', 'venv_deg_s',
                          'qmed_deg', 'vmed_deg_s'):
                    cab += ['%s_J%d' % (b, i + 1) for i in range(3)]
                cab += ['error', 'factor', 'servo', 'apertura', 'g_med']
                w.writerow(cab)
                for f in filas:
                    w.writerow(['%.3f' % f['t']]
                               + ['%.3f' % v for v in f['q_plan']]
                               + ['%.3f' % v for v in f['qd_plan']]
                               + ['%.3f' % v for v in f['v_env']]
                               + ['%.3f' % v for v in f['q_med']]
                               + ['%.3f' % v for v in f['v_med']]
                               + ['%.4f' % f['err'], '%.3f' % f['factor'],
                                  '%d' % f['servo'],
                                  '%d' % f['apertura'],
                                  '%d' % f['g_med']])
            print('registro de %d ciclos en %s' % (len(filas), log))
            print('  graficalo con:  python3 graficar_log.py %s' % log)
    return abortado


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trayectoria',
                    choices=('ciclo', 'recta', 'caja', 'ensenado'),
                    default='ciclo',
                    help='caja: pretrayectoria ensenada a mano, provisional')
    ap.add_argument('--perfil', choices=('trapezoidal', 'cubico'),
                    default='trapezoidal')
    ap.add_argument('--vel', type=float, default=45.0,
                    help='velocidad de crucero de la junta dominante [deg/s]')
    ap.add_argument('--acel', type=float, default=80.0)
    ap.add_argument('--puntos', type=int, default=12)
    ap.add_argument('--repetir', type=int, default=1)
    ap.add_argument('--sin-palas', action='store_true',
                    help='no manda aperturas: el gripper no se mueve')
    ap.add_argument('--simulado', action='store_true')
    ap.add_argument('--solo-plan', action='store_true')
    ap.add_argument('--log', metavar='FICHERO.csv',
                    help='guarda todo lo enviado y leido, para graficarlo')
    ap.add_argument('--ros', action='store_true',
                    help='publicar la posicion medida para que Gazebo siga '
                         'al robot real (hace falta source de ROS 2)')
    args = ap.parse_args()

    print(cfg.resumen())

    try:
        apertura_por_tramo = None
        if args.trayectoria == 'recta':
            hitos, _ = tray.recta_por_defecto(args.puntos)
            servo_por_tramo = {}
            cartesiano = True
        elif args.trayectoria == 'caja':
            hitos, _ = tray.ciclo_caja_juntas()
            servo_por_tramo = {}
            cartesiano = False
        elif args.trayectoria == 'ensenado':
            hitos, nombres = tray.ciclo_ensenado_juntas()
            servo_por_tramo = {}
            # la apertura del punto k se aplica AL LLEGAR a el, que es donde
            # empieza el tramo k
            apertura_por_tramo = (None if args.sin_palas else
                                  dict(tray.APERTURA_POR_PUNTO))
            cartesiano = False
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
        muestras = anadir_aproximacion(muestras, q0, args.perfil, args.vel,
                                       args.acel)

        espejo = EspejoROS() if args.ros else None
        parado = False
        try:
            parado = ejecutar(muestras, enlace, servo_por_tramo, args.repetir,
                              cartesiano, espejo, args.log,
                              apertura_por_tramo=apertura_por_tramo)
        finally:
            if espejo is not None:
                espejo.cerrar()
    if parado:
        print('\nPARADO.  El brazo se quedo donde estaba.  Mira el motivo arriba.')
        return 2
    print('\nhecho.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
