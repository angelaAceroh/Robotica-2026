from machine import Pin, PWM, Timer
import machine
import time
import sys
import math

# ========================================
# CLASE PARA TRAYECTORIAS INTERNAS
# ========================================
class PerfilESP32:
    def __init__(self):
        self.activo = False
        
    def configurar(self, q0, qf, vmax, amax):
        self.dir = 1.0 if qf >= q0 else -1.0
        self.dq = abs(qf - q0)
        self.vmax = abs(vmax) if abs(vmax) > 0 else 0.1
        self.amax = abs(amax) if abs(amax) > 0 else 1.0
        
        ta_temp = self.vmax / self.amax
        qa_temp = 0.5 * self.amax * (ta_temp ** 2)
        
        if 2.0 * qa_temp <= self.dq:
            self.ta = ta_temp
            self.tc = (self.dq - 2.0 * qa_temp) / self.vmax
            self.v_peak = self.vmax
        else:
            self.ta = math.sqrt(self.dq / self.amax)
            self.tc = 0.0
            self.v_peak = self.amax * self.ta
            
        self.tf = 2.0 * self.ta + self.tc
        self.inicio_ms = time.ticks_ms()
        self.activo = True

    def evaluar(self):
        t = time.ticks_diff(time.ticks_ms(), self.inicio_ms) / 1000.0
        if t < 0: return 0.0
        elif t < self.ta: return self.dir * (self.amax * t)
        elif t < (self.ta + self.tc): return self.dir * self.v_peak
        elif t <= self.tf: return self.dir * (self.v_peak - self.amax * (t - (self.ta + self.tc)))
        else:
            self.activo = False
            return 0.0

perfil_M1 = PerfilESP32()
perfil_M2 = PerfilESP32()
perfil_M3 = PerfilESP32()

try:
    import select
except ImportError:
    import uselect as select

# ========================================
# PINES Y HARDWARE (Frecuencia corregida a 1000Hz)
# ========================================
pinLED = Pin(2, Pin.OUT)

# Motor 1
pinPWM = PWM(Pin(27), freq=1000, duty_u16=0)
IN1 = Pin(25, Pin.OUT)
IN2 = Pin(26, Pin.OUT)
ENCODER_A = Pin(18, Pin.IN, Pin.PULL_UP)
ENCODER_B = Pin(19, Pin.IN, Pin.PULL_UP)

# Motor 2
pinPWM2 = PWM(Pin(23), freq=1000, duty_u16=0)
IN3 = Pin(33, Pin.OUT)
IN4 = Pin(32, Pin.OUT)
ENCODER_A2 = Pin(21, Pin.IN, Pin.PULL_UP)
ENCODER_B2 = Pin(22, Pin.IN, Pin.PULL_UP)

# Motor 3
pinPWM3 = PWM(Pin(13), freq=1000, duty_u16=0)
IN5 = Pin(16, Pin.OUT)
IN6 = Pin(17, Pin.OUT)
ENCODER_A3 = Pin(4, Pin.IN, Pin.PULL_UP)
ENCODER_B3 = Pin(14, Pin.IN, Pin.PULL_UP)

# ========================================
# VARIABLES DE CONTROL Y ESTADO
# ========================================
PULSOS_POR_VUELTA = 990.0
T_MUESTREO = 0.01

# Coeficientes de la ecuación en diferencias
q0 = 0.2332
q1 = -0.0462

# PWM Mínimo para superar zona muerta y fricción del L298N (Ajustar si es necesario)
MIN_PWM = 45 

encoderCount = encoderCount2 = encoderCount3 = 0
lastCount = lastCount2 = lastCount3 = 0

ref = ref2 = ref3 = 0.0
refnew = refnew2 = refnew3 = 0.0

xk = ek = ekm1 = uk = ukm1 = 0.0
xk2 = ek2 = ekm12 = uk2 = ukm12 = 0.0
xk3 = ek3 = ekm13 = uk3 = ukm13 = 0.0

refImpr = xkImpr = 0.0
refImpr2 = xkImpr2 = 0.0
refImpr3 = xkImpr3 = 0.0

tiempoMovimiento = tiempoMovimiento2 = tiempoMovimiento3 = 0.0
tiempoInicio = tiempoInicio2 = tiempoInicio3 = 0
movimientoActivo = movimientoActivo2 = movimientoActivo3 = False

timer = Timer(0)
ejecutarControl = False
nuevoDato = False
contadorImpresion = 0

lectorConsola = select.poll()
lectorConsola.register(sys.stdin, select.POLLIN)

# ========================================
# INTERRUPCIONES Y TIMER
# ========================================
def leerEncoder(pin):
    global encoderCount
    encoderCount += 1 if ENCODER_B.value() == 1 else -1

def leerEncoder2(pin):
    global encoderCount2
    encoderCount2 += 1 if ENCODER_B2.value() == 1 else -1

def leerEncoder3(pin):
    global encoderCount3
    encoderCount3 += 1 if ENCODER_B3.value() == 1 else -1

def onTimer(timer):
    global ejecutarControl
    ejecutarControl = True

# ========================================
# APLICAR SALIDAS (PWM + DIRECCIÓN + ZONA MUERTA)
# ========================================
def aplicarControlMotor1(u_volts, dir_ref, activo):
    if not activo or abs(dir_ref) < 0.001 or u_volts <= 0.05:
        IN1.value(0); IN2.value(0)
        pinPWM.duty_u16(0)
        return
    
    if dir_ref > 0:
        IN1.value(1); IN2.value(0)
    else:
        IN1.value(0); IN2.value(1)
        
    pwm_calc = int((u_volts / 12.0) * (255 - MIN_PWM) + MIN_PWM)
    pwm_val = max(MIN_PWM, min(255, pwm_calc))
    pinPWM.duty_u16(int(pwm_val * 257))

def aplicarControlMotor2(u_volts, dir_ref, activo):
    if not activo or abs(dir_ref) < 0.001 or u_volts <= 0.05:
        IN3.value(0); IN4.value(0)
        pinPWM2.duty_u16(0)
        return
    
    if dir_ref > 0:
        IN3.value(1); IN4.value(0)
    else:
        IN3.value(0); IN4.value(1)
        
    pwm_calc = int((u_volts / 12.0) * (255 - MIN_PWM) + MIN_PWM)
    pwm_val = max(MIN_PWM, min(255, pwm_calc))
    pinPWM2.duty_u16(int(pwm_val * 257))

def aplicarControlMotor3(u_volts, dir_ref, activo):
    if not activo or abs(dir_ref) < 0.001 or u_volts <= 0.05:
        IN5.value(0); IN6.value(0)
        pinPWM3.duty_u16(0)
        return
    
    if dir_ref > 0:
        IN5.value(1); IN6.value(0)
    else:
        IN5.value(0); IN6.value(1)
        
    pwm_calc = int((u_volts / 12.0) * (255 - MIN_PWM) + MIN_PWM)
    pwm_val = max(MIN_PWM, min(255, pwm_calc))
    pinPWM3.duty_u16(int(pwm_val * 257))

def detenerMotor1():
    global ref, refnew, movimientoActivo, uk, ukm1, ek, ekm1
    ref = refnew = 0.0
    movimientoActivo = False
    uk = ukm1 = ek = ekm1 = 0.0
    aplicarControlMotor1(0, 0, False)

def detenerMotor2():
    global ref2, refnew2, movimientoActivo2, uk2, ukm12, ek2, ekm12
    ref2 = refnew2 = 0.0
    movimientoActivo2 = False
    uk2 = ukm12 = ek2 = ekm12 = 0.0
    aplicarControlMotor2(0, 0, False)

def detenerMotor3():
    global ref3, refnew3, movimientoActivo3, uk3, ukm13, ek3, ekm13
    ref3 = refnew3 = 0.0
    movimientoActivo3 = False
    uk3 = ukm13 = ek3 = ekm13 = 0.0
    aplicarControlMotor3(0, 0, False)

# ========================================
# BUCLE DE CONTROL (100 Hz - TIMER)
# ========================================
def controlMotor():
    global ref, refnew, ref2, refnew2, ref3, refnew3
    global lastCount, xk, ek, ekm1, uk, ukm1, refImpr, xkImpr
    global lastCount2, xk2, ek2, ekm12, uk2, ukm12, refImpr2, xkImpr2
    global lastCount3, xk3, ek3, ekm13, uk3, ukm13, refImpr3, xkImpr3
    global contadorImpresion, nuevoDato

    # Evaluar trayectoria local en la ESP32 si está activa
    if perfil_M1.activo:
        v_p = perfil_M1.evaluar()
        ref = abs(v_p); refnew = v_p
        if not perfil_M1.activo: detenerMotor1()

    if perfil_M2.activo:
        v_p2 = perfil_M2.evaluar()
        ref2 = abs(v_p2); refnew2 = v_p2
        if not perfil_M2.activo: detenerMotor2()

    if perfil_M3.activo:
        v_p3 = perfil_M3.evaluar()
        ref3 = abs(v_p3); refnew3 = v_p3
        if not perfil_M3.activo: detenerMotor3()

    pinLED.value(not pinLED.value())

    # Leer encoders de forma atómica
    irq_state = machine.disable_irq()
    c1, c2, c3 = encoderCount, encoderCount2, encoderCount3
    machine.enable_irq(irq_state)

    # Motor 1
    delta1 = c1 - lastCount; lastCount = c1
    xk = abs((delta1 / PULSOS_POR_VUELTA) * 2.0 * math.pi / T_MUESTREO)
    ek = ref - xk
    uk = max(0.0, min(12.0, (q0 * ek) + (q1 * ekm1) + ukm1))
    ekm1 = ek; ukm1 = uk
    aplicarControlMotor1(uk, refnew, movimientoActivo)
    refImpr, xkImpr = refnew, xk

    # Motor 2
    delta2 = c2 - lastCount2; lastCount2 = c2
    xk2 = abs((delta2 / PULSOS_POR_VUELTA) * 2.0 * math.pi / T_MUESTREO)
    ek2 = ref2 - xk2
    uk2 = max(0.0, min(12.0, (q0 * ek2) + (q1 * ekm12) + ukm12))
    ekm12 = ek2; ukm12 = uk2
    aplicarControlMotor2(uk2, refnew2, movimientoActivo2)
    refImpr2, xkImpr2 = refnew2, xk2

    # Motor 3
    delta3 = c3 - lastCount3; lastCount3 = c3
    xk3 = abs((delta3 / PULSOS_POR_VUELTA) * 2.0 * math.pi / T_MUESTREO)
    ek3 = ref3 - xk3
    uk3 = max(0.0, min(12.0, (q0 * ek3) + (q1 * ekm13) + ukm13))
    ekm13 = ek3; ukm13 = uk3
    aplicarControlMotor3(uk3, refnew3, movimientoActivo3)
    refImpr3, xkImpr3 = refnew3, xk3

    contadorImpresion += 1
    if contadorImpresion >= 5:
        contadorImpresion = 0
        nuevoDato = True

# ========================================
# SETUP Y BUCLE PRINCIPAL
# ========================================
def setup():
    detenerMotor1(); detenerMotor2(); detenerMotor3()
    ENCODER_A.irq(trigger=Pin.IRQ_RISING, handler=leerEncoder)
    ENCODER_A2.irq(trigger=Pin.IRQ_RISING, handler=leerEncoder2)
    ENCODER_A3.irq(trigger=Pin.IRQ_RISING, handler=leerEncoder3)
    
    timer.init(period=10, mode=Timer.PERIODIC, callback=onTimer)
    print("Control de tres motores iniciado (Firmware optimizado)")

def loop():
    global ref, refnew, tiempoMovimiento, tiempoInicio, movimientoActivo
    global ref2, refnew2, tiempoMovimiento2, tiempoInicio2, movimientoActivo2
    global ref3, refnew3, tiempoMovimiento3, tiempoInicio3, movimientoActivo3
    global ejecutarControl, nuevoDato

    # Lectura de comandos Serial
    if lectorConsola.poll(0):
        try:
            linea = sys.stdin.readline().strip()
        except Exception:
            return
            
        if linea:
            valores = linea.replace(",", " ").split()
            tipo_comando = valores[0]

            if tipo_comando == 'V' and len(valores) == 7:
                perfil_M1.activo = perfil_M2.activo = perfil_M3.activo = False
                
                v1, t1 = float(valores[1]), float(valores[2])
                v2, t2 = float(valores[3]), float(valores[4])
                v3, t3 = float(valores[5]), float(valores[6])

                if t1 > 0 and v1 != 0:
                    ref = abs(v1); refnew = v1; tiempoMovimiento = t1
                    tiempoInicio = time.ticks_ms(); movimientoActivo = True
                else: detenerMotor1()

                if t2 > 0 and v2 != 0:
                    ref2 = abs(v2); refnew2 = v2; tiempoMovimiento2 = t2
                    tiempoInicio2 = time.ticks_ms(); movimientoActivo2 = True
                else: detenerMotor2()

                if t3 > 0 and v3 != 0:
                    ref3 = abs(v3); refnew3 = v3; tiempoMovimiento3 = t3
                    tiempoInicio3 = time.ticks_ms(); movimientoActivo3 = True
                else: detenerMotor3()

            elif tipo_comando == 'P' and len(valores) == 10:
                q_obj1, vmax1, amax1 = float(valores[1]), float(valores[2]), float(valores[3])
                q_obj2, vmax2, amax2 = float(valores[4]), float(valores[5]), float(valores[6])
                q_obj3, vmax3, amax3 = float(valores[7]), float(valores[8]), float(valores[9])
                
                # Posiciones actuales desde encoders
                pos1 = (encoderCount / PULSOS_POR_VUELTA) * 2.0 * math.pi
                pos2 = (encoderCount2 / PULSOS_POR_VUELTA) * 2.0 * math.pi
                pos3 = (encoderCount3 / PULSOS_POR_VUELTA) * 2.0 * math.pi
                
                perfil_M1.configurar(pos1, q_obj1, vmax1, amax1)
                perfil_M2.configurar(pos2, q_obj2, vmax2, amax2)
                perfil_M3.configurar(pos3, q_obj3, vmax3, amax3)
                
                movimientoActivo = movimientoActivo2 = movimientoActivo3 = True

    # Comprobación de tiempo de movimiento
    now = time.ticks_ms()
    if movimientoActivo and not perfil_M1.activo:
        if time.ticks_diff(now, tiempoInicio) >= int(tiempoMovimiento * 1000):
            detenerMotor1()

    if movimientoActivo2 and not perfil_M2.activo:
        if time.ticks_diff(now, tiempoInicio2) >= int(tiempoMovimiento2 * 1000):
            detenerMotor2()

    if movimientoActivo3 and not perfil_M3.activo:
        if time.ticks_diff(now, tiempoInicio3) >= int(tiempoMovimiento3 * 1000):
            detenerMotor3()

    # Ejecutar control discreto
    if ejecutarControl:
        ejecutarControl = False
        controlMotor()

    # Telemetría de retorno
    if nuevoDato:
        nuevoDato = False
        print(f"{refImpr:.4f},{xkImpr:.4f},{uk:.2f},{refImpr2:.4f},{xkImpr2:.4f},{uk2:.2f},{refImpr3:.4f},{xkImpr3:.4f},{uk3:.2f}")

setup()

try:
    while True:
        loop()
        time.sleep_ms(1)
except KeyboardInterrupt:
    detenerMotor1(); detenerMotor2(); detenerMotor3()
    timer.deinit()
    pinPWM.deinit(); pinPWM2.deinit(); pinPWM3.deinit()
    print("Programa detenido")
