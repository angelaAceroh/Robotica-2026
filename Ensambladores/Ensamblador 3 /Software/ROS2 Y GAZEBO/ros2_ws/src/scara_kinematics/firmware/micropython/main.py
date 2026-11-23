"""main.py: Alvin, Simon y Teodoro a la vez, en la ESP32 (MicroPython).

Es el mismo control de Alvin.py, Simon.py y Teodoro.py (los de Sergio, 21/10):
lazo de velocidad con v[k] = v[k-1] + 0.6139*e[k] - 0.5861*e[k-1] cada 10 ms,
error en PORCENTAJE de la velocidad pedida, filtro de 8 muestras, parada al
pasar la posicion y posicion guardada en flash. Con sus pines y sus cuentas.
Lo unico que cambia es que los tres van en el MISMO bucle, porque en una ESP32
solo corre un programa a la vez: con Alvin.py cargado, Simon y Teodoro no existen.

LA CONSIGNA: una linea por el USB, la manda el nodo `vector` del PC (o tu
desde Thonny):

    [pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, vel_teodoro]

en las unidades de sus programas: Alvin y Simon en grados de su junta y RPM,
Teodoro en mm y mm/min (lo que Teodoro.py llama "RPM"). Velocidad 0 = ese motor
no se mueve en este tramo.

El PC la manda con un checksum detras, `[...] *A3` (suma de los bytes del
corchete, modulo 256, en hexadecimal): si no cuadra, el USB la ha estropeado y
no se mueve nada. Tecleada a mano en Thonny puede ir sin checksum.

Si algun nodo tendria que salirse de su recorrido (MIN y MAX en la tabla
MOTORES), no se mueve NINGUNO: es la ultima barrera, por si llega un vector que
no ha pasado por el PC.

OTRAS ORDENES
    q           parada de emergencia de los tres (y guarda las posiciones)
    h simon     home de uno: su posicion actual pasa a 0 (h teodoro). A Alvin
                no: «NO DARLE A LA H» (Alvin.py)
    fijar simon -35.2   le dice donde esta de verdad (grados o mm)
    ?           dice donde esta cada uno
    l           dice los limites que tiene cada nodo
    (h y fijar, solo con los tres parados)

LO QUE CONTESTA (el nodo `vector` lee estas lineas; no cambies su formato)
    VECTOR [-10, 14, -22.2, 30, 0, 60] checksum ok
                                    el vector que ha llegado, tal cual lo ha leido
    RECIBE Alvin pos 10.000 vel 14 esta 0.00
                                    lo que toma cada nodo del vector, y donde esta
    RECHAZADO 0.00 0.00 0.00        algun nodo se saldria de su recorrido: nada se mueve
    [!] Alvin: OBJETIVO ALCANZADO en 10.15° (meta 10.00°, error +0.15°)
    LIMITES Alvin -60 60 Simon -280 0 Teodoro -135 0     respuesta a 'l'
    T: 1.23s | Alvin 30.1° 10.0rpm pwm 312 meta 40.0° | Simon ... | Teodoro -5.0mm 60.0mm/min ...
                                    10 veces por segundo, mientras alguno se mueve:
                                    posicion, velocidad, PWM y destino de cada nodo
    LISTO 30.05 -44.98 -20.02       los tres han acabado el vector y estan quietos
    POS 30.05 -44.98 -20.02         respuesta a '?' (grados, grados, mm)
    ERROR Simon <motivo>            un vigilante ha parado ese motor
"""
from machine import Pin, PWM, Timer
import time
import sys
import select

# =================================================
# LOS TRES MOTORES
# =================================================
# Los pines y las cuentas de Alvin.py, Simon.py y Teodoro.py (21/10): los que
# funcionan. Los mismos que la tabla de main_control.py (un test lo comprueba).
#
# INDICE es el sitio de cada motor en el vector
#     [pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, vel_teodoro]
# Cada nodo recibe el vector ENTERO y saca de el solo su pareja:
# vector[2*INDICE] es su posicion y vector[2*INDICE + 1] su velocidad.
#
# PULSOS_REV  pulsos del encoder por vuelta del motor (22000/4, la prueba de
#             posicion de Sergio)
# POR_VUELTA  lo que avanza su junta en una vuelta del motor, en SUS unidades:
#             Alvin 360 grados; Simon 360*18/21 (Simon.py: 5500*21/18 pulsos por
#             vuelta de junta, la transmision); Teodoro los mm de Teodoro.py
# MIN y MAX   el recorrido de cada motor, en sus unidades: los que dio Sergio
#             con el brazo delante (21/10). Tienen que ser los mismos que
#             `limites_motor` en config/micropython.yaml (el PC): el nodo
#             `vector` los compara al conectar y avisa si no cuadran.
#
# Teodoro.py: AVANCE_POR_VUELTA_MM = (3.14159265*(0.5 * 20)) + 10 (pinon M0.5 de
# 20 dientes, 31.4 mm, mas 10)
TEODORO_POR_VUELTA = 3.14159265 * (0.5 * 20) + 10
SIMON_POR_VUELTA = 360.0 * 18 / 21
#
#            nombre  INDICE  IN1  IN2  ENA  ENC_A  ENC_B  PULSOS_REV  POR_VUELTA           MIN     MAX
MOTORES = [
    ('Alvin',      0,     4,  16,  17,   26,    25,   5500,  360.0,              -60.0,   60.0),
    ('Simon',      1,    18,  19,   5,   12,    13,   5500,  SIMON_POR_VUELTA,  -280.0,    0.0),
    ('Teodoro',    2,    21,  22,  23,   27,    14,   5500,  TEODORO_POR_VUELTA, -135.0,   0.0),
]
# Como en sus programas: la posicion en grados o mm, y la velocidad en RPM de la
# junta (vueltas de 360 grados por minuto) o en mm por minuto. El tercer numero
# es lo que avanza la junta, en sus unidades, por cada unidad de velocidad y minuto.
UNIDAD = {'Alvin': ('°', 'rpm', 360.0), 'Simon': ('°', 'rpm', 360.0),
          'Teodoro': ('mm', 'mm/min', 1.0)}
SIN_H = ('Alvin',)        # Alvin.py: "NO DARLE A LA H"

# El control, igual que en los tres archivos
GANANCIA_E0 = 0.6139
GANANCIA_E1 = -0.5861
PWM_MAX = 1023.0
NUM_DATOS_PROMEDIO = 8
PERIODO_MS = 10
MARGEN = 0.5              # mas cerca que esto del destino (grados o mm) = ya esta
VEL_MIN = 10.0            # RPM (mm/min Teodoro): lo que dicen sus programas,
VEL_MAX = 60.0            # «usa velocidades de 10 a 60»
RPM_QUIETO = 1.0          # LISTO solo cuando los tres van mas despacio que esto

# Vigilantes. No estaban en los archivos de un motor, y con tres a la vez hacen
# falta: con un encoder que no cuenta, o un motor que gira al reves de lo que
# cuenta su encoder, el motor nunca "llega" y sigue hasta el tope mecanico.
# Probado en simulacion: sin ellos, Alvin al reves llego a -278 grados.
# Los grados son del EJE DEL MOTOR, para que valgan igual con Teodoro en mm.
GRADOS_AL_REVES = 5.0     # se ha movido esto en sentido contrario al pedido
SEG_SIN_MOVER = 2.0       # tanto tiempo empujando...
GRADOS_SIN_MOVER = 1.0    # ... sin moverse ni esto: el encoder no cuenta
FACTOR_TIEMPO = 3.0       # un movimiento puede tardar 3 veces lo previsto...
MARGEN_TIEMPO_S = 3.0     # ... mas 3 s, y despues se para

PASOS = [
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
]


class Motor:
    """Un nodo de junta: un motor con su lazo, lo que antes era un archivo entero."""

    def __init__(self, nombre, indice, in1, in2, ena, enc_a, enc_b, pulsos_rev,
                 por_vuelta, lim_min, lim_max):
        self.nombre = nombre
        self.indice = indice
        self.unidad, self.unidad_vel, self.por_min = UNIDAD[nombre]
        self.por_vuelta = por_vuelta
        self.f = 360.0 / por_vuelta           # sus unidades -> grados del eje del motor
        self.lim_min = lim_min
        self.lim_max = lim_max
        self.meta_pos = 0.0
        self.meta_vel = 0.0
        self.IN1 = Pin(in1, Pin.OUT)
        self.IN2 = Pin(in2, Pin.OUT)
        self.ENA = PWM(Pin(ena), freq=1000)
        self.ENC_A = Pin(enc_a, Pin.IN)
        self.ENC_B = Pin(enc_b, Pin.IN)
        self.PULSOS_REV = pulsos_rev
        self.archivo = "posicion_" + nombre + ".txt"

        self.pulsos = self.leer_posicion_flash()
        self.estado_anterior = (self.ENC_A.value() << 1) | self.ENC_B.value()
        self._cb = self.encoder_callback      # una sola vez: el IRQ no crea objetos
        self.ENC_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)
        self.ENC_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)

        self.ref = 0.0
        self.velocidad_signo = 1.0
        self.posicion_deseada = self.posicion()
        self.rpm_actual = 0.0
        self.en_regimen = False
        self.pulsos_anterior = self.pulsos
        self.vectorE = [0.0, 0.0, 0.0]
        self.vectorV = [0.0, 0.0, 0.0]
        self.buffer_rpm = [0.0] * NUM_DATOS_PROMEDIO
        self.indice_buffer = 0
        self.datos_buffer = 0
        self.pwm = 0.0
        self.fallo = False
        self.inicio = self.posicion()
        self.ms_movimiento = 0
        self.ms_maximo = 0
        self.motor(0)

    # ---------------------------------------------------------------- hardware
    def encoder_callback(self, pin):
        estado_actual = (self.ENC_A.value() << 1) | self.ENC_B.value()
        resultado = PASOS[(self.estado_anterior << 2) | estado_actual]
        if resultado != 0:
            self.pulsos += resultado
        self.estado_anterior = estado_actual

    def motor(self, pwm):
        pwm_val = int(max(-1023, min(1023, pwm)))
        if pwm_val > 0:
            self.IN1.value(1)
            self.IN2.value(0)
            self.ENA.duty(pwm_val)
        elif pwm_val < 0:
            self.IN1.value(0)
            self.IN2.value(1)
            self.ENA.duty(abs(pwm_val))
        else:
            self.IN1.value(0)
            self.IN2.value(0)
            self.ENA.duty(0)

    def posicion(self):
        """En sus unidades: grados de junta o mm, como en sus programas."""
        return (self.pulsos / self.PULSOS_REV) * self.por_vuelta

    # ------------------------------------------------------------------- flash
    def guardar_posicion_flash(self):
        try:
            with open(self.archivo, "w") as f:
                f.write(str(self.pulsos))
        except Exception as e:
            print("[✗] " + self.nombre + ": error al guardar en flash:", e)

    def leer_posicion_flash(self):
        try:
            with open(self.archivo, "r") as f:
                return int(f.read().strip())
        except:
            return 0  # Si el archivo no existe, arranca en 0

    def fijar(self, valor):
        """Donde esta de verdad, en sus unidades. La h es fijar(0)."""
        self.detener(guardar=False, avisar=False)
        self.pulsos = int(round(valor / self.por_vuelta * self.PULSOS_REV))
        self.pulsos_anterior = self.pulsos
        self.posicion_deseada = self.posicion()
        self.guardar_posicion_flash()
        print(f"[🏠] {self.nombre}: la posición actual ahora es "
              f"{self.posicion():.2f}{self.unidad}.")

    # ----------------------------------------------------------------- control
    def limpiar_filtro(self):
        self.indice_buffer = 0
        self.datos_buffer = 0

    def promedio_rpm(self, nueva_rpm):
        self.buffer_rpm[self.indice_buffer] = nueva_rpm
        self.indice_buffer += 1
        if self.indice_buffer >= NUM_DATOS_PROMEDIO:
            self.indice_buffer = 0
        if self.datos_buffer < NUM_DATOS_PROMEDIO:
            self.datos_buffer += 1
        suma = 0.0
        for i in range(self.datos_buffer):
            suma += self.buffer_rpm[i]
        return suma / self.datos_buffer

    def datosE(self, ultimo):
        self.vectorE[2] = self.vectorE[1]
        self.vectorE[1] = self.vectorE[0]
        self.vectorE[0] = ultimo

    def datosV(self, ultimo):
        self.vectorV[2] = self.vectorV[1]
        self.vectorV[1] = self.vectorV[0]
        self.vectorV[0] = ultimo

    def dis_dis(self):
        control_raw = (self.vectorV[0] + (GANANCIA_E0 * self.vectorE[0])
                       + (GANANCIA_E1 * self.vectorE[1]))
        if control_raw > PWM_MAX:
            control_raw = PWM_MAX
        elif control_raw < -PWM_MAX:
            control_raw = -PWM_MAX

        self.datosV(control_raw)

        if self.ref == 0:
            control_pwm = 0.0
            self.datosV(0.0)
        else:
            control_pwm = control_raw

        self.motor(control_pwm)
        return control_pwm

    def detener(self, guardar=True, error=None, avisar=True):
        moviendose = self.ref != 0.0
        self.ref = 0.0
        self.motor(0)
        # Se limpian los historicos para evitar sobretiros al arrancar de nuevo
        self.vectorE = [0.0, 0.0, 0.0]
        self.vectorV = [0.0, 0.0, 0.0]
        self.pwm = 0.0
        if guardar:
            self.guardar_posicion_flash()
        if error:
            self.fallo = True
            print("ERROR " + self.nombre + " " + error)
        elif moviendose and avisar:
            pos = self.posicion()
            u = self.unidad
            print(f" [!] {self.nombre}: OBJETIVO ALCANZADO en {pos:.2f}{u} "
                  f"(meta {self.posicion_deseada:.2f}{u}, error {pos - self.posicion_deseada:+.2f}{u})")

    # ---------------------------------------------------------------- consigna
    def recibir(self, vector):
        """Le llega el vector entero; el nodo toma SOLO lo suyo (aun no se mueve)."""
        self.meta_pos = vector[2 * self.indice]
        self.meta_vel = vector[2 * self.indice + 1]
        # Lo que le ha llegado a este nodo (el nodo `vector` del PC lo comprueba)
        print(f"RECIBE {self.nombre} pos {self.meta_pos:.3f} vel {self.meta_vel:g} "
              f"esta {self.posicion():.2f}")

    def fuera_de_recorrido(self):
        """Motivo si su parte del vector le saca de [MIN, MAX], o None."""
        if self.meta_vel == 0.0:
            return None                          # no se va a mover
        if self.meta_pos < self.lim_min or self.meta_pos > self.lim_max:
            return (f"{self.nombre} pos {self.meta_pos:.2f} fuera de su recorrido "
                    f"[{self.lim_min:g}, {self.lim_max:g}]")
        return None

    def arrancar(self):
        self.nueva_meta(self.meta_pos, self.meta_vel)

    def nueva_meta(self, posicion_deseada, velocidad):
        """Lo que antes hacia la linea POSICION,VELOCIDAD de la consola."""
        ref_velocidad_crucero = abs(velocidad)
        actual = self.posicion()
        self.posicion_deseada = posicion_deseada

        if ref_velocidad_crucero == 0.0:
            self.ref = 0.0                       # velocidad 0: no se mueve
        elif abs(posicion_deseada - actual) < MARGEN:
            self.ref = 0.0                       # ya esta
        else:
            if ref_velocidad_crucero < VEL_MIN or ref_velocidad_crucero > VEL_MAX:
                recortada = min(max(ref_velocidad_crucero, VEL_MIN), VEL_MAX)
                print(f"[!] {self.nombre}: {ref_velocidad_crucero:.1f} {self.unidad_vel} "
                      f"fuera de [{VEL_MIN:.0f}, {VEL_MAX:.0f}], uso {recortada:.1f}")
                ref_velocidad_crucero = recortada
            if posicion_deseada > actual:
                self.velocidad_signo = 1.0
                self.ref = ref_velocidad_crucero
            else:
                self.velocidad_signo = -1.0
                self.ref = -ref_velocidad_crucero
            # cuanto deberia tardar, para el vigilante de tiempo
            previsto_s = (abs(posicion_deseada - actual) * 60.0
                          / (ref_velocidad_crucero * self.por_min))
            self.ms_maximo = int((FACTOR_TIEMPO * previsto_s + MARGEN_TIEMPO_S) * 1000)

        self.vectorE = [0.0, 0.0, 0.0]
        self.vectorV = [0.0, 0.0, 0.0]
        self.en_regimen = False
        self.inicio = actual
        self.ms_movimiento = 0
        self.limpiar_filtro()
        print(f"[+] {self.nombre} | Meta: {posicion_deseada:.1f}{self.unidad} | Actual: "
              f"{actual:.1f}{self.unidad} | Velocidad: {self.ref:.1f} {self.unidad_vel}")

    def paso(self, dt):
        """Lo que antes hacia cada 10 ms el bucle de un archivo."""
        pulsos_actuales = self.pulsos
        delta_pulsos = pulsos_actuales - self.pulsos_anterior
        self.pulsos_anterior = pulsos_actuales
        posicion_encoder_real = self.posicion()

        if dt > 0:
            # RPM de la junta (Alvin, Simon) o mm/min (Teodoro), como en sus programas
            rpm_calculada = (delta_pulsos / self.PULSOS_REV * self.por_vuelta
                             / self.por_min / dt) * 60000.0
        else:
            rpm_calculada = self.rpm_actual
        rpm_filtrada = self.promedio_rpm(rpm_calculada)

        if self.ref != 0:
            limite_inferior = self.ref * 0.95 if self.ref > 0 else self.ref * 1.05
            if not self.en_regimen:
                if (self.ref > 0 and rpm_filtrada >= limite_inferior) or \
                   (self.ref < 0 and rpm_filtrada <= limite_inferior):
                    self.en_regimen = True
            self.rpm_actual = rpm_filtrada
        else:
            self.en_regimen = False
            self.rpm_actual = rpm_filtrada
            self.limpiar_filtro()

        # Parada automatica al pasar el destino
        if self.ref != 0.0:
            if (self.velocidad_signo == 1.0 and posicion_encoder_real >= self.posicion_deseada) or \
               (self.velocidad_signo == -1.0 and posicion_encoder_real <= self.posicion_deseada):
                self.detener()

        # Error de velocidad para el controlador discreto: en PORCENTAJE de la
        # pedida, como en sus programas (100 al arrancar, sea cual sea la
        # velocidad). Sus programas hacen (|ref| - |rpm|)/|ref| * 100 * signo:
        # es lo mismo mientras el motor gira hacia donde se le pide, y si gira al
        # reves esta cuenta empuja mas fuerte en vez de creer que va bien.
        if self.ref != 0.0:
            self.datosE((self.ref - self.rpm_actual) / abs(self.ref) * 100.0)
        else:
            self.datosE(-self.rpm_actual)
        self.pwm = self.dis_dis()

        if self.ref != 0.0:
            self.vigilar(dt)

    def vigilar(self, dt):
        self.ms_movimiento += dt
        # en grados del eje del motor
        avance = (self.posicion() - self.inicio) * self.velocidad_signo * self.f
        u = self.unidad
        if avance <= -GRADOS_AL_REVES:
            self.detener(error=f"va al reves: se pidio {self.ref:+.1f} {self.unidad_vel} y se "
                               f"ha movido {-avance / self.f:.2f}{u} en sentido contrario. "
                               f"Motor al reves de su encoder: cambia IN1 por IN2 (o ENC_A "
                               f"por ENC_B)")
        elif self.ms_movimiento >= SEG_SIN_MOVER * 1000 and abs(avance) < GRADOS_SIN_MOVER:
            self.detener(error=f"{SEG_SIN_MOVER:.0f} s empujando y el motor no ha girado ni "
                               f"{GRADOS_SIN_MOVER:.0f}°: el encoder no cuenta, falta potencia "
                               f"o esta bloqueado")
        elif self.ms_movimiento >= self.ms_maximo:
            self.detener(error=f"lleva {self.ms_movimiento / 1000:.1f} s y no llega a "
                               f"{self.posicion_deseada:.1f}{u} (esta en {self.posicion():.1f}{u})")


# =================================================
# ARRANQUE
# =================================================
motores = [Motor(*m) for m in MOTORES]
POR_NOMBRE = {m.nombre.lower(): m for m in motores}

flag_muestreo = False


def tick(timer):
    global flag_muestreo
    flag_muestreo = True


timer = Timer(0)
timer.init(period=PERIODO_MS, mode=Timer.PERIODIC, callback=tick)


def posiciones():
    return " ".join(f"{m.posicion():.2f}" for m in motores)


def checksum(texto):
    """Suma de los bytes, modulo 256, en hexadecimal: la misma cuenta que el PC."""
    return "%02X" % (sum(texto.encode()) & 0xFF)


def leer_vector(linea):
    """'[pA, vA, pS, vS, pT, vT] *HH' -> (6 numeros, 'ok' o 'sin').

    Con checksum, si no cuadra levanta ValueError y no se mueve nada.
    """
    estado = 'sin'
    if '*' in linea:
        cuerpo, llegado = linea.rsplit('*', 1)
        cuerpo, llegado = cuerpo.strip(), llegado.strip().upper()
        if checksum(cuerpo) != llegado:
            raise ValueError("checksum: llego *" + llegado + " y el vector da *"
                             + checksum(cuerpo) + " (el USB lo ha estropeado)")
        linea, estado = cuerpo, 'ok'
    linea = linea.replace('[', '').replace(']', '').replace(';', ',').replace(' ', ',')
    partes = [p for p in linea.split(',') if p != '']
    if len(partes) != 2 * len(motores):
        raise ValueError(f"hacen falta {2 * len(motores)} valores y llegaron {len(partes)}")
    return [float(p) for p in partes], estado


def fijar(palabras):
    """'h simon' o 'fijar simon -35.2': donde esta el motor de verdad."""
    orden, args = palabras[0], palabras[1:]
    if any(m.ref != 0.0 for m in motores):
        print("[✗] h y fijar, solo con los tres parados: manda q antes")
        return
    uso = "h <motor>" if orden == 'h' else "fijar <motor> <donde esta, grados o mm>"
    if len(args) != (1 if orden == 'h' else 2) or args[0] not in POR_NOMBRE:
        print("[✗] uso: " + uso + " (motores: " + ", ".join(POR_NOMBRE) + ")")
        return
    m = POR_NOMBRE[args[0]]
    if orden == 'h' and m.nombre in SIN_H:
        print(f"[✗] a {m.nombre} no se le da la h (NO DARLE A LA H): su cero es el bueno. "
              f"Si de verdad hay que cambiarlo: fijar {args[0]} 0")
        return
    try:
        valor = 0.0 if orden == 'h' else float(args[1])
    except ValueError:
        print("[✗] " + args[1] + " no es un numero")
        return
    m.fijar(valor)


def leer_consola():
    if select.select([sys.stdin], [], [], 0)[0]:
        linea = sys.stdin.readline().strip()
        return linea or None   # un Enter suelto o el \r\n del PC: no es una orden
    return None


esperando = False     # se mando un vector y todavia no se ha dicho LISTO

print("Sistema listo y calibrado: " + ", ".join(m.nombre for m in motores) + ".")
for m in motores:
    print(f" {m.nombre}: última posición en flash {m.posicion():.2f}{m.unidad}")
print("Consigna: [pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, vel_teodoro]")
print(f" (grados y RPM, mm y mm/min; velocidades entre {VEL_MIN:.0f} y {VEL_MAX:.0f}, "
      f"0 = no se mueve)")
print("Órdenes: q = parada | h simon = home de uno | fijar simon -35 = donde está "
      "| ? = posiciones | l = límites")
for m in motores:
    print(f" {m.nombre}: recorrido [{m.lim_min:g}, {m.lim_max:g}]{m.unidad}")
print("=" * 50)

tiempo_inicio = time.ticks_ms()
tiempo_anterior = tiempo_inicio
tic = 0

while True:
    cmd = leer_consola()
    if cmd is not None:
        orden = cmd.lower()
        if orden == 'q':
            for m in motores:              # parada: no es "objetivo alcanzado"
                m.detener(avisar=False)
            esperando = False
            print("PARADO " + posiciones())
        elif orden == '?':
            print("POS " + posiciones())
        elif orden == 'l':
            print("LIMITES " + " ".join(f"{m.nombre} {m.lim_min:g} {m.lim_max:g}"
                                        for m in motores))
        elif orden.split()[0] in ('h', 'fijar'):
            fijar(orden.split())
        else:
            try:
                vector, estado = leer_vector(cmd)
            except Exception as e:
                vector = None
                print("ERROR vector " + str(e) + ". Usa: "
                      "[pos_alvin, vel_alvin, pos_simon, vel_simon, pos_teodoro, vel_teodoro]")
            if vector is not None:
                print("VECTOR [" + ", ".join(f"{x:g}" for x in vector) + "] checksum " + estado)
                for m in motores:          # el vector entero, a los tres nodos
                    m.recibir(vector)
                fuera = []
                for m in motores:
                    motivo = m.fuera_de_recorrido()
                    if motivo:
                        fuera.append(motivo)
                if fuera:                  # si uno no puede, no se mueve ninguno
                    for motivo in fuera:
                        print("ERROR " + motivo)
                    print("RECHAZADO " + posiciones())
                else:
                    for m in motores:
                        m.arrancar()
                    esperando = True

    if flag_muestreo:
        flag_muestreo = False
        tiempo_actual = time.ticks_ms()
        dt = time.ticks_diff(tiempo_actual, tiempo_anterior)
        tiempo_anterior = tiempo_actual

        for m in motores:
            m.paso(dt)

        # Si un vigilante ha parado a uno, se paran los tres: el movimiento es
        # conjunto, y los otros dos siguiendo solos pueden llevar el brazo a
        # donde no debe. Lo decide la placa, sin esperar al PC.
        fallo = False
        for m in motores:
            if m.fallo:
                fallo = True
                m.fallo = False
        if fallo:
            for m in motores:
                m.detener(avisar=False)
            esperando = False
            print("PARADO " + posiciones())

        moviendo = False
        for m in motores:
            if m.ref != 0.0:
                moviendo = True
        tic += 1
        # 10 veces por segundo, no 100: a 115200 baudios, tres lineas cada
        # 10 ms llenan el USB y el print bloquea el bucle de control.
        if moviendo and tic % 10 == 0:
            t = time.ticks_diff(tiempo_actual, tiempo_inicio) / 1000.0
            print(f"T: {t:.2f}s | " + " | ".join(
                f"{m.nombre} {m.posicion():.1f}{m.unidad} {m.rpm_actual:.1f}{m.unidad_vel} "
                f"pwm {m.pwm:.0f} meta {m.posicion_deseada:.1f}{m.unidad}" for m in motores))
        # LISTO cuando estan QUIETOS, no cuando se les corta el PWM: el motor
        # sigue girando un poco por inercia, y si el PC manda el siguiente
        # vector antes, calcula desde una posicion que ya no es.
        quietos = True
        for m in motores:
            if abs(m.rpm_actual) >= RPM_QUIETO:
                quietos = False
        if esperando and not moviendo and quietos:
            esperando = False
            print("LISTO " + posiciones())
