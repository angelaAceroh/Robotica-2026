"""
GRIPPER: mover las palas con limites por software y medir su carrera.

NO HAY TOPE FISICO.  Si se empuja hasta el final las palas se salen de la
guia, asi que aqui NADA se mueve "hasta que se pare":
  - todo movimiento esta acotado en cuentas Y en tiempo
  - LIMITE_MIN y LIMITE_MAX son topes de software, y arrancan desconocidos
  - el atasco ya NO significa "fin de carrera" sino AVERIA: se para y avisa

No toca control_vel.py.  Se sube aparte y se llama desde el REPL.

MONTAJE
    blanco  (M1)   -> OUT3 del L298N #2
    rojo    (M2)   -> OUT4 del L298N #2
    negro   (VCC)  -> 3.3 V de la ESP32      <- NO a 5 V
    azul    (GND)  -> GND comun
    verde   (C1)   -> GPIO 36  (rotulada VP)
    amarillo(C2)   -> GPIO 39  (rotulada VN)

    IN3 -> GPIO 22     IN4 -> GPIO 23     ENB -> GPIO 2   (jumper FUERA)

    El PWM va en ENB y el sentido en IN3/IN4.  El jumper de ENB se QUITA: con
    el enable fijo a 5 V, al reiniciar la placa las GPIO 22 y 23 quedaban
    flotando con el canal habilitado, el motor arrancaba solo y se quedaba
    BLOQUEADO contra el tope de entrada hasta que alguien apagaba la fuente.
    Eso es lo que, muy probablemente, cocio el colector del primer motor: el
    OL del multimetro aparecia justo con las palas recogidas.

    La GPIO 2 se elige porque lleva pull-down interno durante el reset, por
    hardware, asi que el canal arranca deshabilitado sin depender del software.
    El jumper de ENA (que es J3) sigue fuera, como estaba.

POR QUE EL TOPE DE DUTY
    La fuente da 12 V y el L298N se come ~2, asi que al motor le llegarian
    ~10 V.  El N20 es de 6 V nominales.  DUTY_MAX lo limita al equivalente de
    6 V.  Si mides con el multimetro la tension real en bornes a duty 100 %,
    ajusta DUTY_MAX = 6.0 / esa medida.
"""
import time

from machine import PWM, Encoder, Pin

PIN_ENB = 2                  # PWM.  La GPIO 2 lleva pull-down interno
                             # durante el reset, por hardware, asi que
                             # el canal arranca DESHABILITADO y el motor
                             # no puede empujar durante el arranque.
                             # Antes el PWM iba en IN3/IN4 con el enable
                             # fijo por jumper, y el motor se quedaba
                             # bloqueado contra el tope en cada reinicio.
PIN_IN3, PIN_IN4 = 22, 23    # sentido, salidas digitales sin PWM
PIN_C1, PIN_C2 = 36, 39      # VP y VN en el serigrafiado
UNIDAD_PCNT = 3              # 0, 1 y 2 las usan J1, J2 y J3

PWM_FREQ = 1000              # igual que el resto: el L298N es lento
FASES = 4                    # cuadratura x4, igual que las otras juntas
DUTY_MAX = 0.60              # ~6 V de los ~10 que llegan
DUTY_JOG = 0.25              # para moverse a tientas

SIGNO_ENC = 1                # -1 si el encoder cuenta al reves del sentido
SENTIDO_SALIDA = 1           # que sentido hace SALIR las palas; se confirma
                             # con el primer pulso y se corrige aqui

# ---------------------------------------------------------------- LIMITES
# Cuentas respecto del cero puesto con cero().  None = sin medir todavia:
# mientras valgan None, ir_a() se niega a moverse.  Se rellenan cuando
# Julian diga hasta donde pueden salir y entrar las palas.
# 2026-11-13, medidos sobre el robot con el motor nuevo:
#   0    = tope de entrada, referenciado con referenciar()
#   +522 = hasta donde llego el motor solo, en tres pasos
#   +580 = maximo seguro, sacando a mano desde +522; ahi la cremallera
#          todavia engrana con el pinon.  MAS ALLA NO HAY TOPE: las palas
#          se salen de la guia.
# La prueba a mano del mismo dia dio un recorrido mayor (~680), pero sacando
# con los dedos se puede pasar del punto donde uno dejaria ir al motor.  Se
# toma el numero pequeno a proposito.
LIMITE_MIN = 0               # palas dentro
LIMITE_MAX = 580             # palas fuera

MARGEN = 70                  # cuentas de respeto antes de cada limite.
                             # Medido: al cortar a 130 acabo en 171, o sea
                             # ~40 cuentas de inercia al 25 % de duty.

# --------------------------------------------------------------- seguridad
MS_PULSO = 80                # duracion del primer toque, en ms
CUENTAS_PULSO = 25           # aborta el pulso si se pasa de aqui.
                             # 2026-11-12: eran 400.  La carrera completa
                             # de las palas mide ~225 cuentas (medida a
                             # mano), asi que 400 era casi el doble del
                             # recorrido entero y 60 era un cuarto.
ATASCO_CUENTAS = 3           # menos de esto en una ventana = no se mueve
ATASCO_VENTANA = 0.25        # s
ATASCO_ARRANQUE = 0.6        # s de cortesia antes de vigilar el atasco
MAX_REVERSIONES = 2          # para que ir_a() no se quede zumbando
COAST = 55                   # lo que sigue rodando al 25 %% tras soltar:
                             # medido, se pasaba 50-63 cuentas
DUTY_MIN = 0.25              # por debajo de esto el motor no arranca
MS_TIRON = 20                # tiron corto de la fase de afinado
RETIRADA = 60                # cuentas que se separa del tope antes
                             # de volver a buscarlo
RECORRIDO_MIN = 25           # referenciar() no acepta un tope sin
                             # haberse movido al menos esto


class Gripper:
    def __init__(self):
        # el enable primero y a cero: hasta que no este sujeto, nada se mueve
        self.en = PWM(Pin(PIN_ENB), freq=PWM_FREQ, duty_u16=0)
        self.d1 = Pin(PIN_IN3, Pin.OUT, value=0)
        self.d2 = Pin(PIN_IN4, Pin.OUT, value=0)
        self.enc = Encoder(UNIDAD_PCNT, Pin(PIN_C1), Pin(PIN_C2), phases=FASES)
        self.ref = self.enc.value()

    # ---------------------------------------------------------------- motor
    def mover(self, duty, sentido):
        """duty 0..1 (se recorta a DUTY_MAX), sentido +1 o -1."""
        if duty > DUTY_MAX:
            duty = DUTY_MAX
        if duty < 0:
            duty = 0
        if sentido >= 0:
            self.d1.value(1)
            self.d2.value(0)
        else:
            self.d1.value(0)
            self.d2.value(1)
        self.en.duty_u16(int(duty * 65535))

    def parar(self):
        """Enable a cero: el motor queda desconectado y rueda libre."""
        self.en.duty_u16(0)

    def frenar(self, seg=0.15):
        """Las dos entradas a nivel alto con el enable activo: cortocircuita
        el motor y lo para en seco.  Soltar y dejar rodar cuesta unas 50
        cuentas."""
        self.d1.value(1)
        self.d2.value(1)
        self.en.duty_u16(65535)
        time.sleep(seg)
        self.parar()
        self.d1.value(0)
        self.d2.value(0)

    def soltar(self):
        """Deja el canal en estado seguro y SUJETO.

        No se hace deinit: un pin liberado vuelve a entrada y queda flotando,
        y con el enable flotando el motor puede arrancar solo.  Ese fue el
        fallo que dejaba el gripper bloqueado contra el tope.
        """
        self.en.duty_u16(0)
        self.d1.value(0)
        self.d2.value(0)

    # -------------------------------------------------------------- posicion
    def cuentas(self):
        """Posicion respecto del cero, con el signo del encoder ya corregido."""
        return SIGNO_ENC * (self.enc.value() - self.ref)

    def cero(self):
        """Declara la posicion actual como el cero. Hazlo con las palas a
        media carrera."""
        self.ref = self.enc.value()
        print('cero puesto aqui')
        return 0

    # ----------------------------------------------------------- movimientos
    def pulso(self, sentido=1, duty=DUTY_JOG, ms=MS_PULSO, tope=CUENTAS_PULSO,
              callado=False):
        """PRIMER CONTACTO: un toque corto, acotado en tiempo y en cuentas.

        Es lo unico seguro antes de saber cuanto se mueven las palas por
        cuenta, porque esa relacion depende de la reduccion, que no conocemos.
        """
        c0 = self.cuentas()
        t0 = time.ticks_ms()
        self.mover(duty, sentido)
        exceso = False
        while time.ticks_diff(time.ticks_ms(), t0) < ms:
            if abs(self.cuentas() - c0) > tope:
                exceso = True
                break
        dt = time.ticks_diff(time.ticks_ms(), t0)   # solo lo que empujo
        self.frenar()
        time.sleep(0.25)                      # que termine de rodar
        dc = self.cuentas() - c0
        print('pulso %+d  duty %.0f%%  %d ms  ->  %+d cuentas  (ahora en %+d)'
              % (sentido, duty * 100, dt, dc, self.cuentas()))
        if exceso:
            print('  [ABORTADO] se paso de %d cuentas en el pulso' % tope)
        if callado:
            return dc
        if abs(dc) < 3:
            print('  [AVISO] no conto: revisa verde/amarillo, el VCC y el jumper de ENB')
        elif (dc > 0) != (sentido > 0):
            print('  [nota] el encoder cuenta al reves del sentido pedido:')
            print('         pon SIGNO_ENC = -1 en este archivo')
        print('  mira hacia donde fueron las palas: si con sentido +1 ENTRAN,')
        print('  pon SENTIDO_SALIDA = -1')
        return dc

    def ir_a(self, destino, duty=DUTY_JOG, tol=15, limite_t=4.0):
        """Va a una posicion absoluta en cuentas, respetando los limites.

        Dos fases, porque a 770 cuentas/s no se puede parar en seco:
        primero se acerca rapido y se detiene COAST cuentas ANTES, y luego
        afina a tirones cortos con freno entre uno y otro.  La version de una
        sola fase se pasaba 60 cuentas y llegaba a cruzar LIMITE_MAX.
        """
        if LIMITE_MIN is None or LIMITE_MAX is None:
            print('[NO] LIMITE_MIN/LIMITE_MAX sin definir: no muevo nada.')
            return self.cuentas()
        lo, hi = LIMITE_MIN + MARGEN, LIMITE_MAX - MARGEN
        if destino < lo:
            print('[recorte] %d -> %d (limite inferior)' % (destino, lo))
            destino = lo
        if destino > hi:
            print('[recorte] %d -> %d (limite superior)' % (destino, hi))
            destino = hi

        motivo = 'llego'
        t0 = time.ticks_ms()
        ultima = self.cuentas()
        t_ventana = t0

        # ---- fase 1: acercamiento rapido, parando COAST cuentas antes
        while True:
            pos = self.cuentas()
            e = destino - pos
            if abs(e) <= COAST:
                break
            s = 1 if e > 0 else -1
            if (pos <= LIMITE_MIN and s < 0) or (pos >= LIMITE_MAX and s > 0):
                motivo = 'TOPE DE SOFTWARE'
                break
            ahora = time.ticks_ms()
            if time.ticks_diff(ahora, t0) > limite_t * 1000:
                motivo = 'TIEMPO AGOTADO'
                break
            if time.ticks_diff(ahora, t_ventana) >= ATASCO_VENTANA * 1000:
                if (abs(pos - ultima) < ATASCO_CUENTAS
                        and time.ticks_diff(ahora, t0) > ATASCO_ARRANQUE * 1000):
                    motivo = 'ATASCADO (averia, no fin de carrera)'
                    break
                ultima = pos
                t_ventana = ahora
            self.mover(duty, s)
            time.sleep_ms(5)
        self.frenar()
        time.sleep(0.25)

        # ---- fase 2: afinado a tirones
        if motivo == 'llego':
            for _ in range(10):
                pos = self.cuentas()
                e = destino - pos
                if abs(e) <= tol:
                    break
                s = 1 if e > 0 else -1
                if (pos <= LIMITE_MIN and s < 0) or (pos >= LIMITE_MAX and s > 0):
                    motivo = 'TOPE DE SOFTWARE'
                    break
                self.mover(DUTY_MIN, s)
                time.sleep_ms(MS_TIRON)
                self.frenar(0.12)
                time.sleep(0.12)

        print('ir_a %+d -> quedo en %+d  (error %+d, %s)'
              % (destino, self.cuentas(), self.cuentas() - destino, motivo))
        return self.cuentas()

    def abrir(self, duty=DUTY_JOG):
        return self.ir_a(LIMITE_MAX if SENTIDO_SALIDA > 0 else LIMITE_MIN, duty)

    def cerrar(self, duty=DUTY_JOG):
        return self.ir_a(LIMITE_MIN if SENTIDO_SALIDA > 0 else LIMITE_MAX, duty)


# ===================================================================== medir
def arranque(g, sentido=1, tope=15, ms=250):
    """Busca el duty MINIMO que mueve el motor, subiendo de poco en poco.

    Con tope bajo (60 cuentas) el movimiento es minusculo pase lo que pase,
    porque no sabemos la reduccion.  Para en el primer duty que cuenta.
    """
    for duty in (0.25, 0.30, 0.35, 0.45, 0.60):
        dc = g.pulso(sentido, duty, ms, tope)
        if abs(dc) >= 3:
            print('\n>>> ARRANCA al %.0f%% de duty' % (duty * 100))
            return duty, dc
        time.sleep(0.5)
    print('\n>>> NO SE MUEVE ni al %.0f%% (el tope de duty).' % (DUTY_MAX * 100))
    print('    el encoder ya se comprobo vivo, asi que el fallo esta en la')
    print('    potencia: jumper de ENB, OUT3/OUT4, los 12 V o el propio motor')
    return None, 0


def referenciar(g, duty=0.25, limite_t=3.0, ventana=0.12, quieto=3):
    """Busca el final de carrera del lado SEGURO y pone ahi el cero.

    Solo corre hacia -SENTIDO_SALIDA, que es el lado donde las palas entran y
    hay un tope firme.  Hacia el lado de salida NO se puede buscar nada
    apoyandose: las palas se salen de la guia.

    Si al primer intento no se mueve, puede ser que YA este en el tope: las
    dos cosas se ven igual desde el encoder.  Entonces se separa un poco y
    vuelve a buscar.  Separarse solo es seguro precisamente porque acabamos
    de comprobar que no puede ir mas hacia dentro.

    Devuelve True si encontro el tope.
    """
    sentido = -1 if SENTIDO_SALIDA > 0 else 1
    if sentido == SENTIDO_SALIDA:
        print('[NO] eso apunta al lado de salida; no busco topes por ahi')
        return False
    if _buscar_tope(g, sentido, duty, limite_t, ventana, quieto):
        return True
    print('  no se movio: puede que ya estuviera en el tope.  Me separo.')
    g.pulso(SENTIDO_SALIDA, duty, 300, tope=RETIRADA, callado=True)
    time.sleep(0.4)
    return _buscar_tope(g, sentido, duty, limite_t, ventana, quieto)


def _buscar_tope(g, sentido, duty, limite_t, ventana, quieto):
    print('buscando el tope de entrada (sentido %+d)' % sentido)
    c0 = g.cuentas()
    t0 = time.ticks_ms()
    g.mover(duty, sentido)
    ultima = g.cuentas()
    t_ventana = time.ticks_ms()
    tope = False
    while True:
        ahora = time.ticks_ms()
        if time.ticks_diff(ahora, t0) > limite_t * 1000:
            break
        if time.ticks_diff(ahora, t_ventana) >= ventana * 1000:
            c = g.cuentas()
            # 2026-11-13: antes esto aceptaba el tope si habia arrancado
            # O si habian pasado 500 ms.  Ese 'o' hacia que un motor que
            # NUNCA giraba se reportase como 'TOPE alcanzado', que es
            # exactamente el fallo que hay que detectar.  Ahora el
            # recorrido minimo es obligatorio.
            if abs(c - ultima) < quieto and abs(c - c0) > RECORRIDO_MIN:
                tope = True
                break
            ultima = c
            t_ventana = ahora
        time.sleep_ms(5)
    g.mover(0, sentido)
    g.parar()
    time.sleep(0.3)
    recorrido = g.cuentas() - c0
    if tope:
        motivo = 'TOPE'
    elif abs(recorrido) <= RECORRIDO_MIN:
        motivo = 'NO SE MOVIO: no hay potencia, o esta atascado'
    else:
        motivo = 'se acabo el tiempo sin encontrar tope'
    print('  se movio %+d cuentas en %.2f s  (%s)'
          % (recorrido, time.ticks_diff(time.ticks_ms(), t0) / 1000.0, motivo))
    if not tope:
        return False
    g.cero()
    print('  cero puesto en el tope de entrada')
    return True


def motor_a_ciegas(g, sentido=1, duty=0.45, ms=400):
    """Empuja sin mirar el encoder, para oir y medir con el multimetro.

    Solo tiene sentido si arranque() no detecto nada: sirve para separar
    'el motor no gira' de 'gira pero el encoder no lo ve'.  Acotado en tiempo
    y nada mas, asi que NO lo uses si el encoder funciona.
    """
    print('empujo %d ms al %.0f%%: escucha y mira las palas' % (ms, duty * 100))
    c0 = g.cuentas()
    g.mover(duty, sentido)
    time.sleep(ms / 1000.0)
    g.parar()
    time.sleep(0.3)
    print('cuentas: %+d' % (g.cuentas() - c0))

def tantear(g, sentido=1, veces=5, duty=DUTY_JOG, ms=MS_PULSO):
    """Da varios pulsos en el mismo sentido, de uno en uno, para ir viendo.

    Mira las palas entre pulso y pulso.  Si ves que se acercan al final de la
    guia, CORTA (Ctrl-C) y anota la ultima posicion: ese es el limite.
    """
    for i in range(veces):
        print('--- pulso %d de %d ---' % (i + 1, veces))
        g.pulso(sentido, duty, ms)
        time.sleep(0.6)
    print('\nposicion final: %+d cuentas' % g.cuentas())
    return g.cuentas()


def relacion(g, cuentas=None, duty=DUTY_JOG):
    """Que motor N20 es: cuentas por vuelta del eje de SALIDA.

    El encoder va en el eje del MOTOR, antes de la reductora, asi que los tres
    N20 dan las mismas cuentas por segundo: el encoder solo NO distingue.
    Hay que relacionar cuentas con el giro del eje de salida.

    Marca el eje de salida, llama a esto, y mide con el transportador cuantos
    grados giro.  Luego:  cuentas_por_vuelta = cuentas * 360 / grados.

        1:10  -> ~280 cuentas/vuelta   (1000 rpm)
        1:30  -> ~840                  (300 rpm)
        1:100 -> ~2800                 (100 rpm)

    (suponiendo 7 polos y cuadratura x4 = 28 cuentas por vuelta de motor)
    """
    if cuentas is None:
        if LIMITE_MIN is None:
            print('sin limites definidos: usa tantear() y vuelve')
            return None
        cuentas = (LIMITE_MAX - LIMITE_MIN) // 2
    p0 = g.cuentas()
    print('marca el eje de salida.  voy a moverlo %+d cuentas en 3 s' % cuentas)
    time.sleep(3)
    g.ir_a(p0 + cuentas, duty)
    dc = g.cuentas() - p0
    print('\nse movio %+d cuentas.' % dc)
    print('mide los grados que giro el eje de salida y calcula:')
    print('   %d * 360 / grados = cuentas por vuelta de salida' % abs(dc))
    return dc


def estado(g):
    print('posicion   : %+d cuentas' % g.cuentas())
    print('limites    : %s .. %s' % (LIMITE_MIN, LIMITE_MAX))
    print('signo enc  : %+d     sentido salida: %+d' % (SIGNO_ENC, SENTIDO_SALIDA))
    print('duty max   : %.0f%%   duty jog: %.0f%%' % (DUTY_MAX * 100, DUTY_JOG * 100))


def primer_contacto():
    """Lo primero que se lanza. Un pulso a cada lado y nada mas."""
    g = Gripper()
    try:
        g.cero()
        print('\n--- un toque en sentido +1 ---')
        g.pulso(+1)
        time.sleep(1.0)
        print('\n--- un toque en sentido -1 (deberia volver) ---')
        g.pulso(-1)
        print('\n--- estado ---')
        estado(g)
        print('\nsi los dos toques movieron las palas y las cuentas cuadran,')
        print('sigue con tantear(g, +1, veces=5) mirando la guia.')
    finally:
        g.soltar()
    return None
