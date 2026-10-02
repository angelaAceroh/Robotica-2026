from machine import Pin, PWM, Timer
import time
import sys
import select

# alvin [-20,0] NO DARLE A LA H
IN1 = Pin(4, Pin.OUT)
IN2 = Pin(16, Pin.OUT)
ENA = PWM(Pin(17), freq=1000)

ENC_A = Pin(26, Pin.IN)
ENC_B = Pin(25, Pin.IN)

PULSOS_REV = 22000  # Confirmado por tu prueba de posición

def guardar_posicion_flash(pulsos_actuales):
    try:
        with open("posicion_guardada.txt", "w") as f:
            f.write(str(pulsos_actuales))
    except Exception as e:
        print("[✗] Error al guardar en flash:", e)

def leer_posicion_flash():
    try:
        with open("posicion_guardada.txt", "r") as f:
            return int(f.read().strip())
    except:
        return 0  # Si el archivo no existe, arranca en 0 por defecto

# Cargar la última posición registrada en la Flash
pulsos = leer_posicion_flash()
direccion = 0
estado_anterior = 0

PASOS = [
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
]

def encoder_callback(pin):
    global pulsos, direccion, estado_anterior
    A = ENC_A.value()
    B = ENC_B.value()
    estado_actual = (A << 1) | B
    indice = (estado_anterior << 2) | estado_actual
    resultado = PASOS[indice]
    if resultado != 0:
        pulsos += resultado
        direccion = resultado
    estado_anterior = estado_actual

estado_anterior = (ENC_A.value() << 1) | ENC_B.value()
ENC_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_callback)
ENC_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_callback)

def motor(pwm):
    pwm_val = int(max(-1023, min(1023, pwm)))
    if pwm_val > 0:
        IN1.value(1)
        IN2.value(0)
        ENA.duty(pwm_val)
    elif pwm_val < 0:
        IN1.value(0)
        IN2.value(1)
        ENA.duty(abs(pwm_val))
    else:
        IN1.value(0)
        IN2.value(0)
        ENA.duty(0)

ref = 0.0               
velocidad_signo = 1.0   
posicion_deseada = ((pulsos / PULSOS_REV) * 360.0)  # Inicia en la posición actual

rpm_actual = 0.0
flag_muestreo = False
en_regimen = False
pulsos_anterior = pulsos

vectorE = [0.0, 0.0, 0.0]
vectorV = [0.0, 0.0, 0.0]

tiempo_inicio = time.ticks_ms()
tiempo_anterior = tiempo_inicio

NUM_DATOS_PROMEDIO = 8
buffer_rpm = [0.0] * NUM_DATOS_PROMEDIO
indice_buffer = 0
datos_buffer = 0

def limpiar_filtro():
    global indice_buffer, datos_buffer
    indice_buffer = 0
    datos_buffer = 0

def promedio_rpm(nueva_rpm):
    global buffer_rpm, indice_buffer, datos_buffer
    buffer_rpm[indice_buffer] = nueva_rpm
    indice_buffer += 1
    if indice_buffer >= NUM_DATOS_PROMEDIO:
        indice_buffer = 0
    if datos_buffer < NUM_DATOS_PROMEDIO:
        datos_buffer += 1
    suma = 0.0
    for i in range(datos_buffer):
        suma += buffer_rpm[i]
    return suma / datos_buffer

def tick(timer):
    global flag_muestreo
    flag_muestreo = True

timer = Timer(0)
timer.init(period=10, mode=Timer.PERIODIC, callback=tick)

def datosE(ultimo):
    vectorE[2] = vectorE[1]
    vectorE[1] = vectorE[0]
    vectorE[0] = ultimo

def datosV(ultimo):
    vectorV[2] = vectorV[1]
    vectorV[1] = vectorV[0]
    vectorV[0] = ultimo

def dis_dis():
    control_raw = (vectorV[0] + (0.6139 * vectorE[0]) - (0.5861 * vectorE[1]))
    if control_raw > 1023.0:
        control_raw = 1023.0
    elif control_raw < -1023.0:
        control_raw = -1023.0

    datosV(control_raw)
    
    if ref == 0:
        control_pwm = 0.0
        datosV(0.0)
    else:
        control_pwm = control_raw
        
    motor(control_pwm)
    return control_pwm

# =================================================
# LECTURA DE CONSOLA Y PARADA CONTINUA
# =================================================
def leer_consola():
    global pulsos, posicion_deseada # Agregamos pulsos y posicion_deseada como globales
    if select.select([sys.stdin], [], [], 0)[0]:
        linea = sys.stdin.readline().strip()
        if linea.lower() == 'q':
            return 'q'
        if linea.lower() == 'h': # <--- NUEVA LÍNEA: Si escribes 'h' en consola
            pulsos = 0
            posicion_deseada = 0.0
            guardar_posicion_flash(0)
            print("\n[🏠] ¡Home seteado! La posición actual ahora es 0°.\n")
            return None
        return linea 
    return None

def detener_y_guardar():
    global ref, vectorE, vectorV
    ref = 0.0
    motor(0)
    # Limpiamos los históricos de control para evitar sobretiros al arrancar de nuevo
    vectorE = [0.0, 0.0, 0.0]
    vectorV = [0.0, 0.0, 0.0]
    guardar_posicion_flash(pulsos)
    
    posicion_encoder_real = ((pulsos / PULSOS_REV) * 360.0)
    print("\n" + "=" * 50)
    print(" [!] OBJETIVO ALCANZADO. Guardando posición en flash...")
    print(f" [📍] Posición actual estable: {posicion_encoder_real:.2f}°")
    print("=" * 50)

# =================================================
# BUCLE PRINCIPAL
# =================================================
posicion_encoder_real = ((pulsos / PULSOS_REV) * 360.0) 
print("Sistema listo y calibrado.")
print(f" Recordando última posición en Flash: {posicion_encoder_real:.2f}°")
print("Formato de comando en consola: POSICION,VELOCIDAD (Usa velocidades de 10 a 60 RPM)")
print("Escribe 'q' para forzar una parada de emergencia y guardar.")
print("=" * 50)

while True:
    cmd = leer_consola()
    if cmd == 'q':
        detener_y_guardar()
    elif cmd is not None:
        try:
            partes = cmd.split(',')
            posicion_deseada = float(partes[0])
            ref_velocidad_crucero = abs(float(partes[1])) 
            
            posicion_encoder_real = ((pulsos / PULSOS_REV) * 360.0)
            
            # Validación: si ya está muy cerca del destino, no arranca
            if abs(posicion_deseada - posicion_encoder_real) < 0.5:
                print(f"[!] Ya te encuentras en la posición {posicion_deseada}° (Margen < 0.5°)")
                ref = 0.0
            # Movimiento hacia adelante
            elif posicion_deseada > posicion_encoder_real:
                velocidad_signo = 1.0
                ref = ref_velocidad_crucero
            # Movimiento hacia atrás (Cálculo relativo dinámico automático)
            else:
                velocidad_signo = -1.0
                ref = -ref_velocidad_crucero 

            en_regimen = False
            limpiar_filtro()
            print(f"\n[+] Nueva Meta: {posicion_deseada:.1f}° | Posición Actual: {posicion_encoder_real:.1f}° | Velocidad asignada: {ref:.1f} RPM\n")
        except:
            print("[✗] Error de formato. Usa: POSICION,VELOCIDAD")

    if flag_muestreo:
        flag_muestreo = False

        tiempo_actual = time.ticks_ms()
        dt = time.ticks_diff(tiempo_actual, tiempo_anterior)
        tiempo_anterior = tiempo_actual

        pulsos_actuales = pulsos
        delta_pulsos = pulsos_actuales - pulsos_anterior
        pulsos_anterior = pulsos_actuales

        posicion_encoder_real = ((pulsos / PULSOS_REV) * 360.0)

        if dt > 0:
            revoluciones = delta_pulsos / PULSOS_REV
            rpm_calculada = (revoluciones / dt) * 60000.0
        else:
            rpm_calculada = rpm_actual

        # Filtro de velocidad suavizado aplicando el promedio directo
        rpm_filtrada = promedio_rpm(rpm_calculada)
        
        if ref != 0:
            limite_inferior = ref * 0.95 if ref > 0 else ref * 1.05
            
            if not en_regimen:
                if (ref > 0 and rpm_filtrada >= limite_inferior) or (ref < 0 and rpm_filtrada <= limite_inferior):
                    en_regimen = True
                rpm_actual = rpm_filtrada
            else:
                rpm_actual = rpm_filtrada
        else:
            en_regimen = False
            rpm_actual = rpm_filtrada
            limpiar_filtro()

        # =================================================
        # CONTROLADOR DE VIAJE Y PARADA AUTOMÁTICA
        # =================================================
        if ref != 0.0:
            if (velocidad_signo == 1.0 and posicion_encoder_real >= posicion_deseada) or \
               (velocidad_signo == -1.0 and posicion_encoder_real <= posicion_deseada):
                detener_y_guardar()

        # Error de velocidad para el controlador discreto
        error_velocidad = ref - rpm_actual
        datosE(error_velocidad)
        
        control_aplicado = dis_dis()
        
        t_transcurrido = time.ticks_diff(tiempo_actual, tiempo_inicio) / 1000.0
        
        # Opcional: Imprime telemetría solo si el motor se está moviendo
        if ref != 0.0:
            print(f"T: {t_transcurrido:5.2f}s | Destino: {posicion_deseada:5.1f}° | Pos: {posicion_encoder_real:5.1f}° | RPM: {rpm_actual:5.1f} | PWM: {control_aplicado:5.1f}")

