# ==============================================================================
# main.py — Firmware MicroPython de Alto Rendimiento para Robot SCARA RRP
# Motores: 3x JGA25-370 12V 60RPM + Encoders de Efecto Hall en Cuadratura
# Motor 4: Gripper N20 12V DC por tiempo y PWM configurable
# ==============================================================================
import sys
import time
import math
import machine
from machine import Pin, PWM, Timer

try:
    import select
except ImportError:
    import uselect as select

# ==============================================================================
# 1. HARDWARE: PINES Y PWM A 1000 Hz (Óptimo para puente bipolar L298N)
# ==============================================================================

pinLED = Pin(2, Pin.OUT)
pinLED.value(1)

PWM_FREQ = 1000

# Motor 1 - Hombro (J1) (L298N 1, Canal A)
PWM_M1 = PWM(Pin(27), freq=PWM_FREQ, duty_u16=0)
M1_IN1 = Pin(25, Pin.OUT)
M1_IN2 = Pin(26, Pin.OUT)
M1_ENC_A = Pin(18, Pin.IN, Pin.PULL_UP)
M1_ENC_B = Pin(19, Pin.IN, Pin.PULL_UP)

# Motor 2 - Codo (J2) (L298N 1, Canal B)
PWM_M2 = PWM(Pin(23), freq=PWM_FREQ, duty_u16=0)
M2_IN1 = Pin(33, Pin.OUT)
M2_IN2 = Pin(32, Pin.OUT)
M2_ENC_A = Pin(21, Pin.IN, Pin.PULL_UP)
M2_ENC_B = Pin(22, Pin.IN, Pin.PULL_UP)

# Motor 3 - Eje Z Prismático (J3) (L298N 2, Canal A)
PWM_M3 = PWM(Pin(13), freq=PWM_FREQ, duty_u16=0)
M3_IN1 = Pin(16, Pin.OUT)
M3_IN2 = Pin(17, Pin.OUT)
M3_ENC_A = Pin(34, Pin.IN)  # GPIO 34 (input-only)
M3_ENC_B = Pin(35, Pin.IN)  # GPIO 35 (input-only)

# Motor 4 - Gripper N20 (L298N 2, Canal B - Control PWM Bipolar Directo)
PWM_M4_OPEN = PWM(Pin(14), freq=5000, duty_u16=0)   # Apertura regulada directa en GPIO 14
PWM_M4_CLOSE = PWM(Pin(12), freq=5000, duty_u16=0)  # Cierre regulado directo en GPIO 12
Pin(4, Pin.OUT, value=0)                             # GPIO 4 asegurado en 0V
Pin(5, Pin.OUT, value=0)                             # GPIO 5 asegurado en 0V

# Gripper N20: parámetros seguros para motor de 3V (25% apertura, 18% cierre suave para no rodar soporte)
DEFAULT_PWM_GRIPPER_OPEN = 25.0
DEFAULT_PWM_GRIPPER_CLOSE = 18.0
DEFAULT_TIEMPO_M4_MS = 350
motor4_activo = False
motor4_fin_ms = 0

pwms = [PWM_M1, PWM_M2, PWM_M3]
pines_in1 = [M1_IN1, M2_IN1, M3_IN1]
pines_in2 = [M1_IN2, M2_IN2, M3_IN2]
pines_enc_b = [M1_ENC_B, M2_ENC_B, M3_ENC_B]

# ==============================================================================
# 2. CALIBRACIÓN DE ENCODERS Y CONTROLADORES
# ==============================================================================

PULSOS_MOTOR = 990.0
T_CONTROL = 0.01  # 10 ms (100 Hz)
MAX_RAD_S = 6.28  # 60 RPM máximo a 12V

# PWM mínimo para vencer fricción estática sin atascos:
# Para M3 (cremallera Z vertical) se requiere 110.0 para vencer peso de carro y gravedad
MIN_PWM = [85.0, 80.0, 110.0]
MAX_PWM = 255.0

KP_VEL = [12.0, 12.0, 15.0]
KI_VEL = [2.0, 2.0, 2.5]

encoder_count = [0, 0, 0]
last_count = [0, 0, 0]

ref_vel = [0.0, 0.0, 0.0]
vel_medida = [0.0, 0.0, 0.0]
integral_err = [0.0, 0.0, 0.0]

ultimo_comando_ms = time.ticks_ms()
TIMEOUT_WATCHDOG_MS = 400

flag_control = False
contador_telemetria = 0

# ==============================================================================
# 3. INTERRUPCIONES DE ENCODERS EN CUADRATURA
# ==============================================================================

def isr_enc1(pin):
    if M1_ENC_B.value():
        encoder_count[0] += 1
    else:
        encoder_count[0] -= 1

def isr_enc2(pin):
    if M2_ENC_B.value():
        encoder_count[1] += 1
    else:
        encoder_count[1] -= 1

def isr_enc3(pin):
    if M3_ENC_B.value():
        encoder_count[2] += 1
    else:
        encoder_count[2] -= 1

M1_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=isr_enc1)
M2_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=isr_enc2)
M3_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=isr_enc3)

# ==============================================================================
# 4. ACCIÓN DE MOTORES Y GRIPPER
# ==============================================================================

def aplicar_motor_pwm(idx, duty_val, direccion):
    if duty_val <= 0.0 or direccion == 0:
        pines_in1[idx].value(0)
        pines_in2[idx].value(0)
        pwms[idx].duty_u16(0)
        return

    duty_u16 = int(max(0.0, min(65535.0, duty_val * 257.0)))
    if direccion > 0:
        pines_in1[idx].value(1)
        pines_in2[idx].value(0)
    else:
        pines_in1[idx].value(0)
        pines_in2[idx].value(1)

    pwms[idx].duty_u16(duty_u16)

def parar_motores():
    for i in range(3):
        ref_vel[i] = 0.0
        integral_err[i] = 0.0
        pines_in1[i].value(0)
        pines_in2[i].value(0)
        pwms[i].duty_u16(0)

def controlar_gripper(accion, potencia_pct=None, duracion_ms=None):
    global motor4_activo, motor4_fin_ms
    def_p = DEFAULT_PWM_GRIPPER_CLOSE if accion == "CLOSE" else DEFAULT_PWM_GRIPPER_OPEN
    pct = potencia_pct if (potencia_pct is not None and potencia_pct > 5.0) else def_p
    # Límite estricto de seguridad: máximo 30% para no superar 3.0V en motor N20
    pct = max(0.0, min(30.0, pct))
    dur = duracion_ms if (duracion_ms is not None and duracion_ms >= 100) else DEFAULT_TIEMPO_M4_MS

    duty = int(65535 * (pct / 100.0))
    if accion == "OPEN":
        PWM_M4_CLOSE.duty_u16(0)
        PWM_M4_OPEN.duty_u16(duty)
        motor4_activo = True
        motor4_fin_ms = time.ticks_add(time.ticks_ms(), int(dur))
    elif accion == "CLOSE":
        PWM_M4_OPEN.duty_u16(0)
        PWM_M4_CLOSE.duty_u16(duty)
        motor4_activo = True
        motor4_fin_ms = time.ticks_add(time.ticks_ms(), int(dur))
    else:
        PWM_M4_OPEN.duty_u16(0)
        PWM_M4_CLOSE.duty_u16(0)
        motor4_activo = False

# ==============================================================================
# 5. BUCLE DE CONTROL PERIÓDICO (100 Hz / Timer)
# ==============================================================================

def on_timer(t):
    global flag_control
    flag_control = True

timer = Timer(0)
timer.init(period=10, mode=Timer.PERIODIC, callback=on_timer)

def ejecutar_paso_control():
    global contador_telemetria

    # Watchdog de comunicación
    if time.ticks_diff(time.ticks_ms(), ultimo_comando_ms) > TIMEOUT_WATCHDOG_MS:
        parar_motores()

    # Gestión de tiempo de Gripper N20
    global motor4_activo
    if motor4_activo and time.ticks_diff(time.ticks_ms(), motor4_fin_ms) >= 0:
        PWM_M4_OPEN.duty_u16(0)
        PWM_M4_CLOSE.duty_u16(0)
        motor4_activo = False

    # Control de velocidad por juntura con Feedforward instantáneo
    for i in range(3):
        delta = encoder_count[i] - last_count[i]
        last_count[i] = encoder_count[i]

        v_med = (delta / T_CONTROL) * (2.0 * math.pi / PULSOS_MOTOR)
        vel_medida[i] = v_med

        w_ref = ref_vel[i]
        if abs(w_ref) < 0.005:
            integral_err[i] = 0.0
            aplicar_motor_pwm(i, 0.0, 0)
        else:
            dir_ref = 1 if w_ref > 0 else -1
            w_mag = abs(w_ref)

            # 1. FEEDFORWARD DIRECTO
            min_p = MIN_PWM[i]
            duty_ff = min_p + (w_mag / MAX_RAD_S) * (MAX_PWM - min_p)

            # 2. FEEDBACK PI
            err = w_ref - v_med
            if dir_ref < 0:
                err = -err

            integral_err[i] += err * T_CONTROL
            integral_err[i] = max(-30.0, min(30.0, integral_err[i]))

            duty_fb = (KP_VEL[i] * err) + (KI_VEL[i] * integral_err[i])

            duty_total = max(min_p, min(MAX_PWM, duty_ff + duty_fb))
            aplicar_motor_pwm(i, duty_total, dir_ref)

    # Telemetría continua a 50 Hz hacia ROS 2
    contador_telemetria += 1
    if contador_telemetria >= 2:
        contador_telemetria = 0
        sys.stdout.write(f"P,{encoder_count[0]},{encoder_count[1]},{encoder_count[2]}\n")

# ==============================================================================
# 6. PARSER SERIE NO BLOQUEANTE
# ==============================================================================

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
buffer_rx = ""

def procesar_comando(cmd):
    global ultimo_comando_ms
    cmd = cmd.strip()
    if not cmd:
        return

    # Comando de Velocidad: V,w1,w2,w3
    if cmd.startswith("V,"):
        partes = cmd.split(",")
        if len(partes) >= 4:
            try:
                ref_vel[0] = float(partes[1])
                ref_vel[1] = float(partes[2])
                ref_vel[2] = float(partes[3])
                ultimo_comando_ms = time.ticks_ms()
            except ValueError:
                pass

    # Comando de Paro: S
    elif cmd == "S":
        parar_motores()
        ultimo_comando_ms = time.ticks_ms()

    # Comando de Calibrar Cero: Z
    elif cmd == "Z":
        encoder_count[0] = 0
        encoder_count[1] = 0
        encoder_count[2] = 0
        last_count[0] = 0
        last_count[1] = 0
        last_count[2] = 0
        integral_err[0] = 0.0
        integral_err[1] = 0.0
        integral_err[2] = 0.0
        sys.stdout.write("ACK,ZERO_OK\n")

    # Comando de Gripper: G,open[,pwm,dur_ms] o G,close[,pwm,dur_ms]
    elif cmd.startswith("G,"):
        partes = cmd.split(",")
        if len(partes) >= 2:
            accion = partes[1].strip().upper()
            pot = float(partes[2]) if len(partes) >= 3 else None
            dur = float(partes[3]) if len(partes) >= 4 else None
            controlar_gripper(accion, pot, dur)
            sys.stdout.write(f"ACK,GRIPPER_{accion}\n")

# ==============================================================================
# 7. BUCLE PRINCIPAL
# ==============================================================================

print("ESP32_SCARA_READY")

while True:
    if flag_control:
        flag_control = False
        ejecutar_paso_control()

    events = poller.poll(0)
    if events:
        try:
            while poller.poll(0):
                caracter = sys.stdin.read(1)
                if not caracter:
                    break
                if caracter == '\x03':
                    parar_motores()
                    raise KeyboardInterrupt
                elif caracter == '\n' or caracter == '\r':
                    if buffer_rx:
                        procesar_comando(buffer_rx)
                        buffer_rx = ""
                else:
                    buffer_rx += caracter
                    if len(buffer_rx) > 64:
                        buffer_rx = ""
        except KeyboardInterrupt:
            parar_motores()
            break
        except Exception:
            buffer_rx = ""
