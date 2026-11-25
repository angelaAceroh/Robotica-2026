"""REPL crudo de MicroPython por pyserial. Lo minimo de mpremote."""
import time, serial

class ESP:
    def __init__(self, puerto='/dev/ttyUSB0', baud=115200):
        self.s = serial.Serial(puerto, baud, timeout=1.0)
        # 2026-11-12: eran 2.5 s y un solo par de Ctrl-C, y fallaba con un
        # 'no entro en REPL crudo' porque main.py todavia estaba arrancando.
        time.sleep(4.0)                      # el DTR reinicia la placa
        for _ in range(6):                   # Ctrl-C: cortar el lazo de main
            self.s.reset_input_buffer()
            self.s.write(b'\x03')
            time.sleep(0.4)
        self.s.reset_input_buffer()
        for intento in range(3):
            self.s.write(b'\x01')            # Ctrl-A: REPL crudo
            if b'raw REPL' in self._hasta(b'>', 3.0):
                return
            time.sleep(0.5)
        raise RuntimeError('no entro en REPL crudo')

    def _hasta(self, fin, seg):
        t0 = time.time(); buf = b''
        while time.time() - t0 < seg:
            buf += self.s.read(self.s.in_waiting or 1)
            if buf.endswith(fin):
                break
        return buf

    def ejecutar(self, codigo, seg=10.0):
        self.s.write(codigo.encode() + b'\x04')
        r = self._hasta(b'\x04>', seg)
        if not r.startswith(b'OK'):
            raise RuntimeError('la placa no acepto el codigo: %r' % r[:80])
        cuerpo = r[2:-2]
        salida, _, err = cuerpo.partition(b'\x04')
        if err.strip():
            raise RuntimeError('error en la placa:\n' + err.decode('utf-8', 'ignore'))
        return salida.decode('utf-8', 'ignore')

    def suma(self, ruta):
        """Tamano y suma de comprobacion de un fichero de la placa."""
        return self.ejecutar(
            "s=0\nn=0\nf=open(%r,'rb')\n"
            "while True:\n"
            "    b=f.read(256)\n"
            "    if not b: break\n"
            "    n+=len(b)\n"
            "    for c in b: s=(s*31+c)%%1000000007\n"
            "f.close()\nprint(n,s)" % ruta, seg=30.0).strip()

    def leer(self, ruta):
        import binascii
        trozos = []
        pos = 0
        while True:
            r = self.ejecutar(
                "import binascii\nf=open(%r,'rb')\nf.seek(%d)\n"
                "print(binascii.hexlify(f.read(512)).decode())\nf.close()"
                % (ruta, pos), seg=15.0).strip()
            if not r:
                break
            trozos.append(binascii.unhexlify(r))
            pos += len(trozos[-1])
            if len(trozos[-1]) < 512:
                break
        return b''.join(trozos)

    def escribir(self, ruta, datos):
        import binascii
        self.ejecutar("f=open(%r,'wb')" % ruta)
        for i in range(0, len(datos), 512):
            h = binascii.hexlify(datos[i:i+512]).decode()
            self.ejecutar("import binascii\nf.write(binascii.unhexlify('%s'))" % h, seg=15.0)
        self.ejecutar("f.close()")

    def recargar(self, modulo):
        """Fuerza a que el siguiente import lea del disco.

        2026-11-12: abrir el puerto no siempre reinicia la placa (si el DTR ya
        estaba en el mismo estado), asi que un modulo subido se queda con la
        version vieja en sys.modules y el import no la refresca.
        """
        return self.ejecutar(
            "import sys\n"
            "sys.modules.pop(%r, None)\n"
            "print('descargado %s')" % (modulo, modulo))

    def reiniciar(self):
        self.s.write(b'\x02')                # Ctrl-B: REPL normal
        time.sleep(0.3)
        self.s.write(b'\x04')                # Ctrl-D: reinicio por software

    def cerrar(self):
        self.s.close()

def suma_local(datos):
    s = 0
    for c in datos:
        s = (s * 31 + c) % 1000000007
    return '%d %d' % (len(datos), s)
