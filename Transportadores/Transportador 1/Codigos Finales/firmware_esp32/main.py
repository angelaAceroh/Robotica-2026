"""
Arranque automatico de la ESP32.

Al abrir el puerto serie desde el PC, la placa se reinicia (es el DTR/RTS
del conversor USB).  Si el lazo hubiera que lanzarlo a mano, se perderia
en cada conexion.  Por eso main.py lo arranca solo.

Los motores quedan APAGADOS hasta que llega la primera consigna, asi que
encender la placa no mueve nada.

Para recuperar el REPL (por ejemplo en Thonny): Ctrl-C.
"""
import control_vel

control_vel.run(verboso=True)
