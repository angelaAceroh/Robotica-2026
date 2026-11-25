#!/usr/bin/env python3
"""
Enlace serie con la ESP32.

Mejoras sobre el codigo anterior:
  - Autodetecta el puerto en vez de tener 'COM4' escrito a mano.
  - Manda SIEMPRE grados de JUNTA (el firmware aplica la correa 3:1).
    Antes ControlCinematico.py mandaba RADIANES y final.py grados: el
    firmware los interpretaba como grados en los dos casos, asi que con
    radianes el robot se movia unas 57 veces menos de lo pedido.
  - Reloj de verdad: el periodo se cuenta desde un instante de referencia,
    no con time.sleep(dt) despues de trabajar. Con sleep el lazo se iba
    un 10 % largo, porque el calculo y el envio no se descontaban.
  - Modo simulado: sin ESP32 conectada imprime en vez de fallar, para
    poder probar la trayectoria entera en seco.
"""
import glob
import sys
import time

import config as cfg

try:
    import serial
except ImportError:
    serial = None


def buscar_puerto():
    """Devuelve el primer puerto que parezca una ESP32."""
    if cfg.PUERTO:
        return cfg.PUERTO
    patrones = ('/dev/ttyUSB*', '/dev/ttyACM*')
    for pat in patrones:
        encontrados = sorted(glob.glob(pat))
        if encontrados:
            return encontrados[0]
    # los COM* solo tienen sentido en Windows; en Linux devolver None para
    # que el mensaje de error sea util en vez de 'no such file: COM3'
    if sys.platform.startswith('win'):
        for c in cfg.CANDIDATOS:
            if c.startswith('COM'):
                return c
    return None



# Lo que escribe el firmware: '%.2f' para angulos y velocidades, '%d' para
# las cuentas del gripper.  '.00' (un numero cortado por delante) no casa.
import re as _re
_NUMERO = _re.compile(r'^-?\d+\.\d\d$')
_ENTERO = _re.compile(r'^-?\d+$')

class EnlaceESP:
    def __init__(self, puerto=None, simulado=False, verboso=True):
        self.verboso = verboso
        self.simulado = simulado
        self.ser = None
        self.enviados = 0

        self._buf = ''
        if simulado:
            print('[enlace] MODO SIMULADO: no se abre puerto, solo se imprime')
            return

        if serial is None:
            raise RuntimeError(
                'falta pyserial.  Instalalo con:  pip install pyserial\n'
                'o corre con --simulado para probar sin robot.')

        p = puerto or buscar_puerto()
        if p is None:
            raise RuntimeError(
                'no encuentro la ESP32.  Conectala y comprueba con:\n'
                '    ls /dev/ttyUSB* /dev/ttyACM*\n'
                'Si aparece pero da permiso denegado:\n'
                '    sudo usermod -a -G dialout $USER   (y reinicia sesion)')
        self.ser = serial.Serial(p, cfg.BAUDIOS, timeout=0.05)
        # Reset por HARDWARE con DTR/RTS, como hace esptool.  Abrir el
        # puerto a secas NO reinicia la placa con este adaptador: si se
        # habia quedado en el REPL (por un Ctrl-C anterior), ahi seguia y
        # main.py no llegaba a arrancar.  Sin eso no hay telemetria.
        self.ser.setDTR(False)
        self.ser.setRTS(True)
        time.sleep(0.2)
        self.ser.setRTS(False)
        time.sleep(3.0)                 # arranque de MicroPython y main.py
        self.ser.reset_input_buffer()
        self._buf = ''
        self.reinicios = 0          # veces que se vio arrancar el firmware
        self.ref_ok = 0             # '# gripper referenciado' recibidos
        self.ref_fallos = 0         # '# gripper NO pudo referenciar' recibidos
        print('[enlace] conectado a %s a %d baudios' % (p, cfg.BAUDIOS))

    # ------------------------------------------------------------- envio
    def enviar(self, q_grados, servo=None, gripper=None):
        """q_grados: las tres juntas EN GRADOS DE JUNTA.

        `gripper`: posicion de las palas en CUENTAS de encoder, o
        cfg.GRIPPER_HOME para pedirle al firmware que las referencie.

        El cuarto campo era el servo, que ya no existe: el firmware lo
        ignora.  Se mantiene en la trama para que los programas de
        trayectoria sigan mandando lo que mandan sin cambiar nada.
        """
        if gripper is not None:
            trama = '%.3f,%.3f,%.3f,%d,%.0f\n' % (
                q_grados[0], q_grados[1], q_grados[2],
                int(servo) if servo is not None else 0, gripper)
        elif servo is None:
            trama = '%.3f,%.3f,%.3f\n' % tuple(q_grados)
        else:
            trama = '%.3f,%.3f,%.3f,%d\n' % (q_grados[0], q_grados[1],
                                             q_grados[2], int(servo))
        if self.simulado:
            if self.verboso and self.enviados % 10 == 0:
                print('   -> ' + trama.strip())
        else:
            self.ser.write(trama.encode())
        self.enviados += 1

    def leer_telemetria(self):
        """Ultima linea completa de la ESP32, sin bloquear.

        El firmware de velocidad manda:  th1,vel1,th2,vel2,th3,vel3[,g]
        asi que las POSICIONES son los indices pares y las velocidades los
        impares.  (El de posicion mandaba ref,theta,... de ahi el lio.)

        Se lee solo lo que ya haya llegado y se parsean lineas completas de
        un buffer propio: readline() podia bloquear hasta 50 ms, que es el
        periodo entero del lazo, y hacia que la mitad de los ciclos
        llegaran tarde.
        """
        if self.simulado or self.ser is None:
            return None
        n = self.ser.in_waiting
        if n:
            self._buf += self.ser.read(n).decode('utf-8', 'ignore')
        if '\n' not in self._buf:
            return None
        trozos = self._buf.split('\n')
        self._buf = trozos[-1]              # lo que quede a medias
        datos = None
        for linea in trozos[:-1]:
            linea = linea.strip()
            if linea.startswith('# control de VELOCIDAD listo'):
                # el firmware lo escribe al arrancar: si sale a mitad de una
                # trayectoria, la ESP32 se ha reiniciado y sus posiciones ya
                # no valen (vuelve a creer que el brazo esta en el home)
                self.reinicios += 1
            if linea.startswith('# gripper referenciado'):
                self.ref_ok += 1
            if linea.startswith('# gripper NO pudo referenciar'):
                self.ref_fallos += 1
            if linea and not linea.startswith('#') and not linea.startswith('>'):
                datos = linea               # nos quedamos con la mas reciente
        if not datos:
            return None
        return self._parsear(datos)

    @staticmethod
    def _parsear(datos):
        """Valida el FORMATO de la trama, no solo cuantos campos tiene.

        El firmware escribe  %.2f x6  y  %d  al final (las cuentas del gripper):

            0.00,0.00,90.00,0.00,-90.00,0.00,0

        2026-11-19: antes se aceptaba cualquier linea con 6 campos o mas.  Al
        anadir el septimo campo, una linea a la que le faltaba el PRIMERO --
        p.ej. la que se pilla a medias al abrir el puerto, o una que el ruido
        recorta -- seguia teniendo 6 campos y se aceptaba con todo desplazado:

            0.00,90.00,0.00,-90.00,0.00,0   ->  J1=0  J2=0  J3=0

        Asi, al arrancar, el programa creyo que el brazo estaba en (0,0,0) y lo
        lanzo a toda velocidad hacia esa postura falsa; y en mitad de una
        trayectoria, cada linea corrupta era un ciclo con un error enorme y un
        tiron.  Era el 'baile' que aparecio despues de montar el gripper.
        Ahora una trama solo vale si tiene EXACTAMENTE la forma de arriba.
        """
        trozos = datos.split(',')
        if len(trozos) not in (6, 7):   # 6 = firmware sin gripper
            return None
        if not all(_NUMERO.match(x) for x in trozos[:6]):
            return None
        if len(trozos) == 7 and not _ENTERO.match(trozos[6]):
            return None
        try:
            v = [float(x) for x in trozos]
        except ValueError:
            return None
        d = {'med': v[0:6:2], 'vel': v[1:6:2]}
        if len(v) == 7:
            d['g'] = v[6]
        return d

    def cerrar(self):
        if self.ser is not None:
            self.ser.close()
            print('[enlace] puerto cerrado')

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.cerrar()


class Reloj:
    """Mantiene el periodo real, descontando lo que tarda el trabajo."""

    def __init__(self, periodo):
        self.periodo = periodo
        self.t0 = time.perf_counter()
        self.k = 0
        self.retrasos = 0

    def esperar(self):
        self.k += 1
        objetivo = self.t0 + self.k * self.periodo
        falta = objetivo - time.perf_counter()
        if falta > 0:
            time.sleep(falta)
        else:
            self.retrasos += 1
            # 2026-11-19: si va atrasado MAS de un periodo, se saltan los
            # ciclos perdidos en vez de recuperarlos de golpe.  Antes, tras
            # una pausa de 3 s (mover las palas), las ~60 llamadas siguientes
            # salian sin esperar: el indice del plan avanzaba 60 pasos en
            # milisegundos con el brazo quieto, y el lazo pegaba un tiron de
            # hasta 35 grados para alcanzarlo.
            if -falta > self.periodo:
                self.k = int((time.perf_counter() - self.t0) / self.periodo)

    @property
    def transcurrido(self):
        return time.perf_counter() - self.t0


if __name__ == '__main__':
    print('puerto detectado:', buscar_puerto() or 'ninguno')
    sim = '--simulado' in sys.argv
    with EnlaceESP(simulado=sim) as e:
        for i in range(5):
            e.enviar([i * 2.0, i * 1.0, -i * 1.0], cfg.SERVO_NEUTRO)
            time.sleep(0.1)
        print('enviadas', e.enviados, 'tramas')
