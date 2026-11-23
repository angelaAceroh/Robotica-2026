"""garra.py: la garra (la manita) del SCARA, un servo en el GPIO 33 (ESP32, MicroPython).

Es el codigo probado de la garra (19/11: GPIO 33, 50 Hz, duty 26..128 de 0..1023
para 0..180 grados), cambiado para que no abra y cierre sola cada 2 s: se mueve
cuando se le manda (GARRA.md).

    1   abre      ABIERTA, 10 grados: como se monto en HOME
    0   cierra    CERRADA, 50 grados: «chocolate», agarra el chocolate

Al arrancar se pone ABIERTA.

El servo va al 3V3 de la ESP32 (19/11, no se puede cambiar) y le falta fuerza. Por eso
(GARRA.md, §7):
  - va DE GOLPE, como el codigo probado (VELOCIDAD = None): es como mas empuja. Con
    rampa (VELOCIDAD en grados/s) no se movio nunca.
  - a FRECUENCIA 50 pulsos/s, como el codigo probado (0.6 ms abierta, 1.05 ms
    cerrada). A las 20:32 parecio moverse mas a 200, pero con el falso contacto de abajo
    esa prueba no vale; la unica vez que se le vio moverse con corriente medida (21:31)
    fue a 50, con el codigo probado.
  - al arrancar de golpe tira tanta corriente del 3V3 que el detector de caidas de
    tension de la ESP32 la reiniciaba (66 veces). Tras cada orden el detector se
    apaga AGUANTAR_MS y luego se vuelve a encender, en paso(): en el traycho el
    brazo esta quieto mientras la garra se mueve, y cuando el brazo va, el detector
    esta puesto (si cae la tension, la placa se reinicia y los motores se paran).
  - EMPUJON: la senal solo EMPUJON_MS en cada orden, y luego se suelta (duty 0):
    sujetando, un servo bloqueado tira corriente sin parar y tumbo la placa (21:34,
    15 s despues de cerrar, con el detector ya puesto). La mano se queda donde la deja,
    y cada orden vuelve a empujar aunque ya este ahi (suelta, se ha podido mover).
  - VIGILA SU 3V3 (GPIO 32 con pull-up y pull-down, al ADC: ~0.43 del 3V3) mientras
    empuja, y dice si el servo ha tirado corriente: el 19/11 la causa resulto ser un
    FALSO CONTACTO en el rojo/marron (sin tocar, 0 caidas; moviendo los dupont, 22 y
    «se abrio»). Al moverse de verdad, el 3V3 baja ~1 V.

Dos maneras de usarlo:
  - con el brazo: se guarda en la placa como garra.py, al lado del main.py
    (main_control.py), que lo importa, le pasa el 1 y el 0 que lleguen por el USB
    y llama a paso() en cada vuelta del lazo. Los manda puente_garra
    (traycho_garra.launch.py).
  - solo, para probar la garra: se ejecuta en Thonny (o se guarda como main.py)
    y se teclea 1 o 0.

Contesta (puente_garra las lee: no cambies su formato):
    GARRA cerrada 50   |  GARRA abierta 10       al mandarlo (de golpe) o al llegar
    GARRA cerrando 50  |  GARRA abriendo 10      al empezar, solo con rampa
    GARRA corriente si 430                       al soltar: el 3V3 bajo (mV en el ADC),
                                                 el servo tiro corriente
    GARRA corriente NO 5 (...)                   no le llega: el contacto del rojo/marron

El PWM se crea sin duty= y a una frecuencia que no es la de los motores (1 kHz):
con v1.29, crear en la placa del brazo un PWM a 1 kHz con duty= la cuelga (19/11).
Asi nace con duty 0 (sin pulso) y no da un tiron al encender.
"""
import time

from machine import Pin, PWM

try:
    from machine import mem32
except ImportError:      # sin la placa (las pruebas, placa_simulada)
    mem32 = None

PIN = 33
FRECUENCIA = 50   # pulsos/s, como el codigo probado. mover_servo() mantiene sus anchos de pulso
ABIERTA = 0      # grados: la mano abierta, como esta montada en HOME
CERRADA = 180      # grados: «chocolate». En el codigo probado el print dice 30°, pero manda 50
VELOCIDAD = None  # None = de golpe; grados/s = en rampa (con 20 la placa no se reiniciaba,
                  # pero el servo no se movia)
AGUANTAR_MS = 2000  # tras cada orden, el detector de caidas de tension apagado tanto tiempo
EMPUJON_MS = 1000   # tras llegar (de golpe: al mandarlo), la senal tanto tiempo; luego, suelto
VIGILA_PIN = 32     # libre: con pull-up y pull-down, al ADC, mide el 3V3 (x ~0.43)
CON_CORRIENTE_MV = 40  # lo que tiene que bajar para decir que el servo tiro (el WiFi: 75)

# RTC_CNTL_BROWN_OUT_REG de la ESP32: bit 30 = detector encendido. En esta placa ya
# esta al umbral mas bajo (2.43 V, leido el 19/11): bajarlo no se puede, solo apagarlo
DETECTOR = 0x3FF480D4
DETECTOR_ON = 1 << 30

servo = PWM(Pin(PIN), freq=FRECUENCIA)
estado = None     # 'abierta' | 'cerrada' | 'abriendo' | 'cerrando'
angulo = None     # lo ultimo mandado al servo, en grados
objetivo = None   # a donde va
_antes = 0        # ticks_ms del ultimo paso() de la rampa
_aguanta = None   # ticks_ms de la ultima orden, mientras el detector esta apagado
_suelta = None    # ticks_ms desde el que cuenta EMPUJON_MS, mientras hay senal
_base = 0         # el 3V3 (mV en el ADC) justo antes de empujar
_minimo = 0       # lo mas bajo mientras empuja

try:
    from machine import ADC
    vigia = ADC(Pin(VIGILA_PIN))
    vigia.atten(ADC.ATTN_11DB)
    Pin(VIGILA_PIN, Pin.IN, Pin.PULL_UP | Pin.PULL_DOWN)   # despues del ADC, que las quita
except Exception:          # sin la placa (las pruebas, placa_simulada)
    vigia = None


def tres_v3(n=1):
    """El 3V3 en el ADC (mV, ~0.43 del de verdad); sin vigia, 0."""
    if vigia is None:
        return 0
    return sum(vigia.read_uv() for _ in range(n)) // n // 1000


def mover_servo(angulo):
    # Convierte el angulo (0 a 180) al ciclo de trabajo (duty) para ESP32 (0 a 1023)
    # 0.5 ms (duty ~26) = 0° | 2.5 ms (duty ~128) = 180°, a 50 Hz. A FRECUENCIA, el
    # mismo pulso es FRECUENCIA / 50 veces mas duty
    duty = int(26 + (angulo / 180) * 102)
    servo.duty(duty * FRECUENCIA // 50)


def detector(encendido):
    """El detector de caidas de tension de la ESP32 (sin la placa, nada)."""
    if mem32 is None:
        return
    if encendido:
        mem32[DETECTOR] |= DETECTOR_ON
    else:
        mem32[DETECTOR] &= ~DETECTOR_ON


def _ir(a_donde, llegada, yendo):
    """Va hacia a_donde: de golpe, o empieza la rampa. Si ya esta ahi, vuelve a
    empujar (suelto, la mano se ha podido mover) y lo dice."""
    global objetivo, estado, angulo, _antes, _aguanta, _suelta, _base, _minimo
    objetivo = a_donde
    detector(False)
    _aguanta = _suelta = time.ticks_ms()
    _base = _minimo = tres_v3(16)       # antes de empujar
    if angulo == a_donde or VELOCIDAD is None:
        angulo = a_donde
        mover_servo(angulo)
        estado = llegada
    else:
        estado = yendo
        _antes = time.ticks_ms()
    print("GARRA %s %d" % (estado, a_donde))


def paso():
    """Llamarlo a menudo (main.py: en cada vuelta del lazo, cada 10 ms). Vuelve a
    encender el detector AGUANTAR_MS despues de la ultima orden y, con rampa, acerca
    la garra a donde va; al llegar dice «GARRA cerrada|abierta»."""
    global angulo, estado, _antes, _aguanta, _suelta, _minimo
    ahora = time.ticks_ms()
    if _suelta is not None:
        _minimo = min(_minimo, tres_v3())
        if angulo == objetivo and time.ticks_diff(ahora, _suelta) >= EMPUJON_MS:
            servo.duty(0)                  # suelto: sin senal no tira corriente
            _suelta = None
            if vigia is not None and _base:
                baja = _base - _minimo
                if baja >= CON_CORRIENTE_MV:
                    print("GARRA corriente si %d" % baja)
                else:
                    print("GARRA corriente NO %d (no le llega: mira el contacto del rojo "
                          "y el marron)" % baja)
    if _aguanta is not None and time.ticks_diff(ahora, _aguanta) >= AGUANTAR_MS \
            and angulo == objetivo:
        detector(True)
        _aguanta = None
    if angulo == objetivo:
        return
    d = VELOCIDAD * time.ticks_diff(ahora, _antes) / 1000.0
    _antes = ahora
    if abs(objetivo - angulo) <= d:
        angulo = objetivo
    else:
        angulo += d if objetivo > angulo else -d
    mover_servo(angulo)
    if angulo == objetivo:
        estado = 'cerrada' if objetivo == CERRADA else 'abierta'
        _aguanta = _suelta = ahora     # y el detector y la senal, un rato mas
        print("GARRA %s %d" % (estado, objetivo))


def abrir():
    _ir(ABIERTA, 'abierta', 'abriendo')


def cerrar():
    _ir(CERRADA, 'cerrada', 'cerrando')


def orden(texto):
    """'1' abre, '0' cierra. Devuelve False si no es ninguna de las dos."""
    texto = texto.strip()
    if texto == '1':
        abrir()
    elif texto == '0':
        cerrar()
    else:
        print("[x] garra: 1 = abrir, 0 = cerrar (ha llegado: " + texto + ")")
        return False
    return True


# Al arrancar, abierta (es como esta en HOME), y de golpe: no se sabe donde esta
mover_servo(ABIERTA)
angulo = objetivo = ABIERTA
estado = 'abierta'
_suelta = time.ticks_ms()          # tambien se suelta (sin decir nada de la corriente)
_base = _minimo = 0
print("GARRA abierta %d" % ABIERTA)

if __name__ == '__main__':
    import sys
    print("Garra lista en el GPIO %d: 1 = abrir (%d°), 0 = cerrar (%d°)"
          % (PIN, ABIERTA, CERRADA))
    try:
        while True:
            try:
                linea = sys.stdin.readline().strip()
            except UnicodeError:          # basura por el USB al abrir el puerto
                continue
            if linea and orden(linea):
                while angulo != objetivo or _aguanta is not None or _suelta is not None:
                    paso()
                    time.sleep_ms(10)
    except KeyboardInterrupt:
        # Detiene la señal PWM al interrumpir el programa
        detector(True)
        servo.deinit()
        print("Programa detenido.")
