"""
PRUEBA DE ENCODERS  -  MicroPython en la ESP32

NO mueve los motores.  Solo lee los tres encoders, para comprobar el
cableado antes de dar corriente a nada.

    import test_encoders              lectura continua
    test_encoders.main()              volver a lanzarla
    test_encoders.cero()              pone los contadores a cero
    test_encoders.comprobar('J1')     comprueba PPR girando 90 grados

Esta version usa machine.Encoder, que es el periferico PCNT del ESP32:
cuenta por HARDWARE, en cuadratura completa (x4), sin gastar CPU y sin
perder flancos.  Es lo que hacia falta: contando por interrupciones en
MicroPython, los 14000 flancos por segundo de los tres motores se
comerian el 42 % de la CPU y empezarian a perderse cuentas.
"""
import time

from machine import Encoder, Pin

# ---- pines del montaje: (canal A amarillo, canal B blanco) ----
PINES = [
    ('J1 base',   34, 35),
    ('J2 hombro', 32, 33),
    ('J3 codo',   26, 25),   # intercambiados: contaba al reves del convenio DH
]

PPR_BASE = 540          # pulsos/vuelta del eje de SALIDA, por canal
FASES = 4               # cuadratura completa
N_CORREA = 3            # poleas 60/20
POR_VUELTA = PPR_BASE * FASES * N_CORREA        # 6480 cuentas por vuelta
GRADOS = 360.0 / POR_VUELTA                     # 0.0556 deg por cuenta

_encs = None


def encoders():
    """Crea los tres encoders una sola vez."""
    global _encs
    if _encs is None:
        _encs = []
        for i, (nombre, pa, pb) in enumerate(PINES):
            e = Encoder(i, Pin(pa), Pin(pb), phases=FASES)
            _encs.append((nombre, e))
    return _encs


def cero():
    for _, e in encoders():
        e.value(0)
    print('contadores a cero')


def leer():
    return [(n, e.value()) for n, e in encoders()]


def main(segundos=120):
    encoders()
    print('contador por HARDWARE (PCNT), cuadratura x%d' % FASES)
    print('una vuelta de junta = %d cuentas   (%.4f grados por cuenta)'
          % (POR_VUELTA, GRADOS))
    print()
    print('Gira cada junta A MANO.  Ctrl-C para salir.')
    print()
    print('%-11s %10s %9s %9s' % ('junta', 'cuentas', 'grados', 'vueltas'))
    print('-' * 42)

    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < segundos * 1000:
            for nombre, n in leer():
                print('%-11s %10d %9.2f %9.3f'
                      % (nombre, n, n * GRADOS, n / POR_VUELTA))
            print('-' * 42)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print('fin.')


def comprobar(junta='J1', grados=90):
    """Comprueba el PPR y la relacion de poleas girando un angulo CONOCIDO.

    NO hace falta una vuelta entera: ninguna junta llega a 360 grados
    (J1 +-180, J2 +-90, J3 +-150).  Con 90 grados sobra, y se miden bien
    con una escuadra.

        test_encoders.comprobar('J1')        90 grados, el valor por defecto
        test_encoders.comprobar('J2', 45)    si no cabe mas

    Cuanto mayor sea el angulo, mejor sale la medida: el error de tu
    pulso al juzgarlo se reparte entre mas cuentas.
    """
    esperado = POR_VUELTA * grados / 360.0
    for nombre, e in encoders():
        if nombre.upper().startswith(junta.upper().split()[0]):
            e.value(0)
            print('Contador de %s a cero.' % nombre)
            print('Gira esa junta %g grados, despacio, y pulsa Ctrl-C.'
                  % grados)
            print('Esperado: %.0f cuentas' % esperado)
            try:
                while True:
                    n = e.value()
                    print('  %8d cuentas  =  %7.2f grados      '
                          % (n, n * GRADOS), end='\r')
                    time.sleep(0.2)
            except KeyboardInterrupt:
                n = abs(e.value())
                print()
                print('MEDIDO : %d cuentas' % n)
                print('ESPERADO: %.0f cuentas' % esperado)
                if esperado > 0:
                    r = n / esperado
                    print('relacion medido/esperado: %.3f' % r)
                    if abs(r - 1) < 0.05:
                        print('  OK: PPR 540 y poleas 3:1 confirmados')
                    elif abs(r - 1.0 / 3) < 0.08:
                        print('  Sale un TERCIO: la correa NO es 3:1, parece 1:1')
                    elif abs(r - 3) < 0.25:
                        print('  Sale el TRIPLE: hay un 3 de mas en la cuenta')
                    else:
                        print('  No cuadra. Cuentas por vuelta reales: %.0f'
                              % (n * 360.0 / grados))
                        print('  Revisa PPR_BASE (ahora %d) o N_CORREA (ahora %d)'
                              % (PPR_BASE, N_CORREA))
            return
    print('no encuentro la junta %r. Son: %s'
          % (junta, [n for n, _ in PINES]))


# alias, por si acaso
una_vuelta = comprobar


main()
