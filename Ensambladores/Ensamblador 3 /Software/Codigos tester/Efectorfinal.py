from machine import Pin, PWM
import time

# Configura el pin GPIO 15 para la señal del servo (frecuencia estándar de 50 Hz)
servo = PWM(Pin(33), freq=50)

def mover_servo(angulo):
    # Convierte el ángulo (0 a 180) al ciclo de trabajo (duty) para ESP32 (0 a 1023)
    # 0.5 ms (duty ~26) = 0° | 2.5 ms (duty ~128) = 180°
    duty = int(26 + (angulo / 180) * 102)
    servo.duty(duty)

print("Iniciando control de servo...")

try:
    while True:
        print("chcolate (30°)...")
        mover_servo(50)
        time.sleep(2)  # Espera 2 segundos

        print("abriendo (10°)...")
        mover_servo(10)
        time.sleep(2)  # Espera 2 segundos

except KeyboardInterrupt:
    # Detiene la señal PWM al interrumpir el programa
    servo.deinit()
    print("Programa detenido.")