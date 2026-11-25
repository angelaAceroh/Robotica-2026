#!/usr/bin/env python3
"""Monitor en vivo del encoder del gripper, para girar a mano y ver.

    python3 ~/robot_fisico/vigilar_encoder.py [segundos]

No crea los PWM, asi que la 22 y la 23 quedan como entradas y el L298N no
frena el motor: con el jumper de ENB fuera, el eje gira libre.

Muestra las cuentas del PCNT y, aparte, los niveles crudos de C1 y C2.  Eso
separa dos fallos distintos:
  - los niveles cambian pero las cuentas no  -> el PCNT esta mal configurado
  - los niveles no cambian                   -> el encoder no genera pulsos
    (alimentacion insuficiente, cable suelto, o imanes/sensor averiados)
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # esp_repl.py esta en esta misma carpeta
from esp_repl import ESP

SEG = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

e = ESP()
try:
    e.ejecutar(
        "from machine import Pin, Encoder\n"
        "_a = Pin(36, Pin.IN)\n"
        "_b = Pin(39, Pin.IN)\n"
        "try:\n"
        "    _e.deinit()\n"
        "except Exception:\n"
        "    pass\n"
        "_e = Encoder(3, _a, _b, phases=4)\n"
        "_e.value(0)")
    print('listo.  GIRA EL EJE A MANO durante %.0f s' % SEG)
    print('%-8s %-10s %-6s %-6s %s' % ('t(s)', 'cuentas', 'C1', 'C2', ''))
    t0 = time.time()
    c_ant, niveles = None, set()
    cambios_c, cambios_n = 0, 0
    while time.time() - t0 < SEG:
        r = e.ejecutar("print(_e.value(), _a.value(), _b.value())").strip().split()
        c, a, b = int(r[0]), int(r[1]), int(r[2])
        if c_ant is not None and c != c_ant:
            cambios_c += 1
        if (a, b) not in niveles:
            cambios_n += 1
        niveles.add((a, b))
        c_ant = c
        marca = '  <-- SE MUEVE' if cambios_c else ''
        print('%-8.1f %-10d %-6d %-6d %s' % (time.time() - t0, c, a, b, marca))
        time.sleep(0.15)
    print()
    print('cambios de cuenta        : %d' % cambios_c)
    print('combinaciones C1/C2 vistas: %s' % sorted(niveles))
    if cambios_c:
        print('>>> EL ENCODER FUNCIONA.  El fallo esta en la potencia del motor.')
    elif len(niveles) > 1:
        print('>>> los niveles cambian pero el PCNT no cuenta: es la unidad 3.')
    else:
        print('>>> ni las cuentas ni los niveles se movieron.')
        print('    Si de verdad giraste el eje, el encoder no genera pulsos:')
        print('    lo mas probable es que 3,3 V no le alcancen.')
finally:
    e.cerrar()
