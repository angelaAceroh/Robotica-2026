/* ============================================================================
 * scara_esp32.ino  -  firmware del SCARA "scara_urdf" (3 GDL, R-R-P)
 *
 * El PC resuelve la cinematica inversa (nodo scara_ik de ROS 2) y va soltando
 * consignas de junta a 50 Hz por el USB. Aqui no hay cinematica: esta placa
 * solo cierra un PID de posicion por junta contra los encoders de los tres
 * JGA25-370, que es lo que la simulacion se ahorra porque en gz los
 * JointPositionController ya hacen ese trabajo.
 *
 * La cuarta junta del URDF ("manita", la rotacion de la muneca) NO se implementa:
 * el brazo fisico no la lleva. En ROS eso se corresponde con has_wrist: false en
 * config/scara_urdf3.yaml, que deja el modelo en 3 GDL; el eslabon Ian se queda
 * dibujado en su cero y ya esta.
 *
 *     panel / RViz / Gazebo  ->  scara_ik  ->  esp32_bridge  --USB-->  ESP32
 *                                                    ^                    |
 *                                                    +---- posicion ------+
 *
 * MOTORES: JGA25-370 12 V 60 RPM con encoder en cuadratura.
 *
 *   - 60 RPM en el eje de salida son 6.28 rad/s: sobra para las rotativas,
 *     pero OJO con el husillo (ver CUENTAS_* mas abajo).
 *   - La reductora NO es freno. Con la etapa de potencia apagada el eje
 *     vertical se cae solo, asi que "parar" aqui significa frenar (IN1 = IN2
 *     con el puente habilitado), no dejar el motor libre.
 *
 * ETAPA DE POTENCIA: L298N. Dos pines de direccion + un pin PWM de "enable"
 * por motor, que es como esta cableado el brazo (ENA con el puente QUITADO).
 *
 *     u > 0  ->  IN1=1, IN2=0, ENA=duty
 *     u < 0  ->  IN1=0, IN2=1, ENA=duty
 *     freno  ->  IN1=IN2=0, ENA=maximo   (motor en corto por los transistores de abajo)
 *     libre  ->  ENA=0                    (no se usa: el husillo se caeria)
 *
 *   El PWM arranca a 1 kHz (PWM_FREQ_INICIAL; W hz lo cambia en caliente): el
 *   L298N es lento y a 20 kHz se calienta sin dar mas par. Si algun dia se cambia a un TB6612 o un BTS7960, ata su
 *   pin de enable (STBY / R_EN+L_EN) a 3.3 V fijos y deja el PWM en ENA igual;
 *   entonces si se puede subir a 20 kHz para salir del rango audible.
 *
 * ENCODERS: son 5 V en la mayoria de modulos y el ESP32 NO tolera 5 V en sus
 * entradas. Alimenta el encoder desde 3.3 V (los hall del JGA25-370 funcionan)
 * o mete un divisor 10k/20k en cada canal. Los seis pines de encoder de este
 * cableado (12/13, 14/27, 26/25) si tienen pull-up interno, asi que no hacen
 * falta resistencias externas aunque el modulo sea de colector abierto.
 *
 * PROTOCOLO (lineas de texto terminadas en \n, checksum XOR "*XX" opcional):
 *
 *   PC -> ESP32
 *     T q1 q2 d3          consigna de junta        (rad, rad, m)
 *     M j q               lleva SOLO la junta j a q (rad, o m si j = 2)
 *     A j u ms            prueba en lazo abierto: ciclo u (-1..1) durante ms
 *     I j +1|-1           invierte el motor por software (como cruzar sus cables)
 *     E 0|1               etapa de potencia        (0 = frena y abre el lazo)
 *     Z                   toma la pose actual como cero
 *     H                   vuelve a la pose de referencia, por rampa
 *     K j kp ki kd kv     ganancias de la junta j  (j = 0..2)
 *     C j cuentas         cuentas de encoder por rad (por metro si j = 2)
 *     D j sentido         +1 / -1, invierte el encoder de la junta j
 *     R j qmin qmax vmax  recorrido y velocidad maxima
 *     P j pwmmin pwmmax   PWM de arranque y tope, 0..1
 *     F hz                frecuencia de telemetria (0 = apagada)
 *     W hz                frecuencia del PWM de los motores (con E 0)
 *     ?                   vuelca toda la configuracion
 *
 *   ESP32 -> PC
 *     > ms q1 q2 q3 u1 u2 u3 flags          telemetria
 *     ! texto                                confirmacion de un comando
 *     # texto                                mensaje informativo
 *
 *   El checksum es opcional a proposito: asi se puede teclear "E 1" o "?" a
 *   mano en el monitor serie del IDE para depurar sin calcular nada.
 *
 *   T es un FLUJO: el puente la manda a 50 Hz y, si deja de llegar 500 ms, la
 *   referencia se congela. M y H son METAS: se teclean una vez y la junta
 *   llega aunque no llegue nada mas. La siguiente T vuelve a mandar.
 *
 * Placa: ESP32 DevKit (WROOM-32).  Core esp32 3.x (API ledcAttach por pin).
 * ========================================================================= */

#include <Arduino.h>

#define NJ 3                      // juntas: juntura1, juntura2, vertiacal1

// ============================================================================
// 1. PINES
//
//    Cableado REAL del brazo. Cada motor lleva el nombre del eslabon que lo
//    ALOJA, no del que mueve, asi que el orden no es el que parece:
//
//      j0  juntura1   hombro   motor "Alvin"    (va dentro de la base)
//      j1  juntura2   codo     motor "Simon"
//      j2  vertiacal1 husillo  motor "Teodoro"
//
//    Quedan libres 32, 33, 34, 35, 36 y 39 para finales de carrera o la pinza
//    (34..39 solo valen como entrada). Evitados: 1/3 (el USB) y 6..11 (flash).
//
//    ---- GPIO 12: STRAPPING, NEUTRALIZADO POR eFUSE ----
//    GPIO 12 (MTDI) fija en el arranque la tension de la flash: si esta en
//    ALTO al soltarse el reset, la ESP32 la pone a 1.8 V y una DevKit con
//    flash de 3.3 V no arranca. El canal A del encoder de Simon va ahi, y un
//    encoder hall deja su salida donde le pille el iman: la placa arrancaba o
//    no segun donde se hubiera parado Simon (la ROM decia "boot:0x33" en vez
//    de 0x13 e "invalid header: 0xffffffff", y esptool "Flash voltage set by
//    a strapping pin: 1.8V"). Como el montaje no deja mover ese cable, el
//    2026-09-10 se quemo el eFuse que fija la flash a 3.3 V (espefuse
//    set-flash-voltage 3.3V): desde entonces ESTA placa ignora el GPIO 12 al
//    arrancar. Con otra ESP32 sin ese eFuse, el canal A de Simon tiene que ir
//    a otro pin (el 33) o la placa volvera a no arrancar.
//
//    SIN_PIN en ENC_A deja una junta con un solo canal (el B): cada flanco es
//    un paso y el sentido lo pone el motor (isrUnCanal). Media resolucion y
//    algo de deriva en cada cambio de sentido: es el recurso si se pierde un
//    canal A.
// ============================================================================
#define SIN_PIN      255          // encoder sin canal A: se cuenta solo el B
//                              IN1   IN2   ENA   ENC_A  ENC_B
#define PINES_J1                16,    4,   17,    26,    25   // Alvin   -> hombro
#define PINES_J2                19,   18,    5, SIN_PIN,   12   // Simon   -> codo
#define PINES_J3                22,   21,   23,    14,    27   // Teodoro -> husillo
// Simon va con un solo canal: en las pruebas A del 2026-09-11, con 12 V, su
// canal del GPIO 12 cambiaba ~22 veces en 0.2 s y el del 13 ni una. El 12 pasa
// al hueco del B, que es el que cuenta isrUnCanal.

// GPIO 5 tambien es strapping, pero al reves: al arrancar lleva pull-up y sale
// alto, que es lo que la placa espera. Lo unico que provoca es un pulso en el
// ENA de Simon durante el arranque; con IN1 e IN2 aun en bajo el L298N no saca
// tension al motor, asi que no se mueve.
// Frecuencia del PWM en ENA. Arranca en 1 kHz, la del MicroPython de Sergio con
// el que Teodoro si se movia; en las pruebas A a 8 kHz los canales de Simon y de
// Teodoro no movieron nada (el de Alvin si). W hz la cambia en caliente.
#define PWM_FREQ_INICIAL 1000     // Hz
#define PWM_BITS     10           // 0..1023
#define PWM_MAX      ((1 << PWM_BITS) - 1)

// ============================================================================
// 2. CALIBRACION MECANICA
//
//    CUENTAS_VUELTA_EJE es cuantos flancos cuenta el decodificador x4 en una
//    vuelta completa del EJE DE SALIDA del motorreductor. Se mide, no se copia:
//    manda "E 0", gira el eje una vuelta exacta a mano y mira la telemetria con
//    C puesto a 1.0 (asi "> " saca cuentas en crudo).
//
//    El valor de partida sale de 11 pulsos/vuelta del encoder en el eje rapido
//    por la reductora del modelo de 60 RPM, contando los cuatro flancos.
//
//    REDUCCION_EXTRA es lo que haya ENTRE el eje del motor y la junta (poleas,
//    correa, engranajes). 1.0 = motor acoplado directamente a la junta.
//
//    PASO_HUSILLO es el avance del eje vertical en una vuelta del tornillo.
//    0.008 m es un T8 de 4 entradas, el mas comun; un T8 de una entrada son
//    0.002 m. Miralo antes de mover nada.
// ============================================================================
#define CUENTAS_VUELTA_EJE   5440.0f    // 1360 flancos en A x 4
#define PASO_HUSILLO         0.008f     // m por vuelta del tornillo

static const float REDUCCION_EXTRA[NJ] = { 1.0f, 1.0f, 1.0f };

#define DOS_PI 6.283185307179586f

// ============================================================================
// 3. LAZO
// ============================================================================
#define TS            0.005f      // s  -> PID a 200 Hz
#define TS_US         5000UL
#define N_FILTRO      8.0f        // filtro de la derivada, como en el PID analogico
#define TIMEOUT_MS    500UL       // sin consignas -> se congela la referencia

// Proteccion de bloqueo. Si una junta lleva MS_BLOQUEO saturada sin haberse
// movido, algo va mal: encoder desconectado, eje agarrotado o signo del lazo
// invertido. Sin esto, el PID empuja al 85 % indefinidamente contra un error
// que nunca se cierra, que es la forma de quemar un motor o partir una correa.
// El margen es generoso: una junta sana recorre metros en ese tiempo.
#define MS_BLOQUEO    1500UL
#define MOV_MINIMO    4.0f        // en unidades de banda muerta

// Sentido invertido. El bloqueo de arriba no lo ve hasta que el eje llega a su
// tope mecanico: con el signo del lazo cambiado el motor SI se mueve, pero
// alejandose de la consigna, y a toda potencia. Aqui se corta en cuanto una
// junta saturada avanza MOV_ESCAPE bandas EN CONTRA de lo que se le empuja
// (3 mm en el husillo, 2.3 grados en las rotativas). Holgado para que la
// inercia de un cambio de sentido no lo dispare.
#define MOV_ESCAPE    10.0f       // en unidades de banda muerta

// ============================================================================
// 4. ESTADO DE CADA JUNTA
//
//    Todo lo ajustable vive aqui y se puede cambiar en caliente por el puerto
//    serie, para no reprogramar la placa en cada intento de sintonia.
// ============================================================================
struct Junta {
  const char* nombre;
  uint8_t in1, in2, ena, encA, encB;   // L298N: direccion + PWM de enable

  float cuentasUd;       // cuentas de encoder por rad (o por metro, en j = 2)
  float sentido;         // +1 / -1 para que el signo case con el URDF

  float qMin, qMax;      // recorrido, del YAML de scara_kinematics
  float vMax;            // rad/s o m/s: lo que el motor puede seguir de verdad

  float kp, ki, kd, kv;  // PID + realimentacion anticipativa de velocidad
  float pwmMin, pwmMax;  // 0..1. pwmMin es lo que cuesta vencer la friccion
  float banda;           // banda muerta, en unidades de junta

  // --- lo que mueve el lazo ---
  volatile int32_t cuentas;
  volatile uint8_t estadoEnc;
  int32_t cero;
  float qMed, qMedAnt, qRef, qRefAnt, qCmd;
  float integral, derivFilt, u;
  bool saturada, tocaLimite;

  // deteccion de bloqueo: desde cuando esta saturada sin moverse, y hacia
  // donde empujaba entonces (0 = no esta saturada)
  uint32_t tBloqueo;
  float qBloqueo;
  int8_t signoBloqueo;

  // Encoder de un canal (encA = SIN_PIN): cada flanco va en el sentido del
  // ultimo empuje del motor. invMotor cruza IN1/IN2 por software (orden I).
  volatile int8_t dirMotor;
  bool invMotor;
};

// Recorridos y velocidades copiados de config/scara_urdf.yaml.
//
// OJO con vMax de la junta 2 (el husillo): el YAML pide 0.10 m/s, pero 60 RPM
// con un paso de 8 mm dan 0.008 m/s. Aqui va el limite fisico; si le pides mas,
// la consigna se adelanta al brazo y el PID acumula error de seguimiento.
static Junta J[NJ] = {
  // Alvin cuenta al reves de lo que empuja (pruebas A del 2026-09-11: u + da
  // cuentas -, al 35, 60 y 85 %). sentido -1 para que el lazo no se escape; si
  // luego gira al reves que en Gazebo, se deja +1 y se invierte el motor (I 0 -1).
  { "juntura1",   PINES_J1, 0, -1.0f, -2.0944f, 2.0944f, 1.50f,
    3.0f,  1.0f, 0.05f, 0.15f,  0.12f, 0.85f, 0.004f,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, 0, 0, 1, false },
  // Simon giraba en horario (visto desde arriba) con u + y el URDF lo quiere
  // antihorario (Sergio, 2026-09-11). Con un solo canal las cuentas siguen a u,
  // asi que basta con invertir el motor (invMotor = true, lo mismo que I 1 -1).
  { "juntura2",   PINES_J2, 0, +1.0f, -2.6180f, 2.6180f, 1.50f,
    3.0f,  1.0f, 0.05f, 0.15f,  0.12f, 0.85f, 0.004f,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, 0, 0, 1, true },
  { "vertiacal1", PINES_J3, 0, +1.0f,  0.0200f, 0.1700f, 0.008f,
    80.0f, 20.0f, 1.00f, 12.0f, 0.15f, 0.90f, 0.0003f,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, 0, 0, 1, false },
};

static bool     habilitado   = false;   // arranca SIEMPRE en frio: ver nota en Z
static int8_t   juntaFallo   = -1;      // junta que provoco el corte de potencia
static bool     falloSentido = false;   // y si fue por sentido invertido o por bloqueo
static bool     metaFija     = false;   // la consigna vino de M/H: no caduca sin enlace

// Prueba en lazo abierto (orden A): una junta, un ciclo de trabajo fijo, un rato.
static int8_t   pruebaJ      = -1;      // -1 = no hay prueba en marcha
static float    pruebaU      = 0.0f;
static uint32_t pruebaFin    = 0;
static int32_t  pruebaDesde  = 0;       // cuentas al empezar
static uint16_t pruebaCambiosA = 0, pruebaCambiosB = 0;   // cambios de nivel de cada
static uint8_t  pruebaUltA = 0, pruebaUltB = 0;           // canal, por sondeo

static uint32_t pwmFreq      = PWM_FREQ_INICIAL;           // la cambia W
static bool     enaOk[NJ]    = { false, false, false };    // ledcAttach salio bien
static bool     ceroHecho    = false;
static uint32_t ultimaOrden  = 0;
static bool     enlaceVivo   = false;
static float    telemetriaHz = 25.0f;
static uint32_t tTelemetria  = 0;
static uint32_t tLazo        = 0;
static uint32_t tArranque    = 0;

// ============================================================================
// 5. ENCODERS
//
//    Decodificacion en cuadratura x4 por tabla. El indice es
//    (estado_anterior << 2) | estado_nuevo, con estado = (A << 1) | B; los ceros
//    son las transiciones imposibles, que es como se filtran los rebotes.
// ============================================================================
// DRAM_ATTR es obligatorio: la lee una ISR en IRAM y una const normal acabaria
// en flash, inaccesible mientras la cache este ocupada.
static const DRAM_ATTR int8_t TABLA_CUAD[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

// digitalRead() se puede llamar desde la ISR: en el core de ESP32 esta marcada
// ARDUINO_ISR_ATTR, o sea que vive en IRAM y no depende de la cache de flash.
static void IRAM_ATTR isrEncoder(void* arg) {
  Junta* j = (Junta*) arg;
  uint8_t s = (digitalRead(j->encA) << 1) | digitalRead(j->encB);
  j->cuentas += TABLA_CUAD[(j->estadoEnc << 2) | s];
  j->estadoEnc = s;
}

// Un solo canal: B da los flancos y el motor, el sentido. Vale porque la
// reductora apenas deja rodar por inercia y, quieto, el motor queda frenado.
static void IRAM_ATTR isrUnCanal(void* arg) {
  Junta* j = (Junta*) arg;
  j->cuentas += j->dirMotor;
}

// ============================================================================
// 6. ETAPA DE POTENCIA
//
//    Unico sitio que sabe como es el driver: el resto del firmware solo pide
//    "aplica u" o "frena". Aqui es un L298N con el puente de ENA quitado, o
//    sea direccion por IN1/IN2 y magnitud por PWM en ENA.
// ============================================================================
static void escribirMotor(Junta& j, float u) {      // u en -1 .. +1
  if (u > 1.0f)  u =  1.0f;
  if (u < -1.0f) u = -1.0f;
  uint32_t duty = (uint32_t) (fabsf(u) * PWM_MAX + 0.5f);
  // El encoder de un canal no sabe hacia donde gira: se lo dice el motor. Con
  // u = 0 se queda el ultimo sentido, que es hacia donde aun pueda rodar.
  if (u > 0.0f)      j.dirMotor = 1;
  else if (u < 0.0f) j.dirMotor = -1;
  // invMotor cruza IN1/IN2 por software, para cuando el motor va al reves y
  // no se pueden cruzar sus cables.
  float v = j.invMotor ? -u : u;
  // Primero la direccion y luego el duty: al reves habria un instante con el
  // puente ya conduciendo y las entradas a medio cambiar.
  if (v > 0.0f)      { digitalWrite(j.in1, HIGH); digitalWrite(j.in2, LOW);  }
  else if (v < 0.0f) { digitalWrite(j.in1, LOW);  digitalWrite(j.in2, HIGH); }
  else               { digitalWrite(j.in1, LOW);  digitalWrite(j.in2, LOW);  }
  ledcWrite(j.ena, duty);
}

// Freno dinamico: con las dos entradas iguales y el puente habilitado, el
// L298N cortocircuita el motor contra si mismo. Es lo que hay que hacer al
// deshabilitar, porque dejarlo libre (ENA a 0) deja caer el eje vertical.
// Se frena POR ABAJO (IN1 = IN2 = 0: los dos bornes del motor a masa), no por
// arriba (IN1 = IN2 = 1: los dos a +12 V). En el banco de pruebas, con los IN de
// Simon o de Alvin a 1 Teodoro no se movia nada, y con ellos a 0 si. Los dos
// frenan igual; solo cambia a que tension quedan los bornes del motor parado.
static void frenarMotor(Junta& j) {
  digitalWrite(j.in1, LOW);
  digitalWrite(j.in2, LOW);
  ledcWrite(j.ena, PWM_MAX);
}

static void frenarTodo() {
  for (int i = 0; i < NJ; i++) { J[i].u = 0.0f; frenarMotor(J[i]); }
}

// ============================================================================
// 7. LECTURA DE POSICION
// ============================================================================
static float leerJunta(Junta& j) {
  // Lectura de 32 bits alineada: en el ESP32 es atomica, no hace falta seccion
  // critica para un solo campo.
  int32_t c = j.cuentas - j.cero;
  return j.sentido * (float) c / j.cuentasUd;
}

static void ponerCero() {
  for (int i = 0; i < NJ; i++) {
    // El cero del husillo no es 0 m sino el fondo de su recorrido: es lo mismo
    // que hace scara_ik al arrancar, que parte de cero y lo recorta a d3_min.
    float q0 = (J[i].qMin > 0.0f) ? J[i].qMin : 0.0f;
    J[i].cero = J[i].cuentas - (int32_t) (J[i].sentido * q0 * J[i].cuentasUd);
    J[i].qMed = J[i].qMedAnt = leerJunta(J[i]);
    J[i].qRef = J[i].qRefAnt = J[i].qCmd = J[i].qMed;
    J[i].integral = 0.0f;
    J[i].derivFilt = 0.0f;
  }
  ceroHecho = true;
}

// ============================================================================
// 8. LEY DE CONTROL
//
//    PID de posicion con las tres correcciones que hacen falta con un
//    motorreductor de continua:
//
//    - DERIVADA SOBRE LA MEDIDA y filtrada. Con la derivada sobre el error, cada
//      escalon de consigna (y llegan 50 por segundo) daria un pico de par.
//
//    - ANTI-WINDUP CONDICIONAL. Sin el, el integrador se carga mientras el
//      motor esta saturado y el brazo se pasa de largo al desaturar.
//
//    - PWM MINIMO. Por debajo de ~12% de duty el JGA25-370 no arranca: la
//      friccion se lo come. Sin este termino el brazo se planta a unos grados
//      del objetivo y se queda ahi con el integrador subiendo despacio.
//
//    kv es realimentacion anticipativa: como la consigna viene rampeada y se
//    sabe a que velocidad va, se le da al motor de entrada la tension que esa
//    velocidad necesita en vez de esperar a que el error la genere.
// ============================================================================
static float control(Junta& j) {
  float e = j.qRef - j.qMed;

  // --- derivada filtrada, sobre la medida ---
  float Tf = (j.kd > 0.0f && j.kp > 0.0f) ? (j.kd / (j.kp * N_FILTRO)) : 0.0f;
  float dq = (j.qMed - j.qMedAnt) / TS;
  if (Tf > 0.0f) j.derivFilt += (TS / (Tf + TS)) * (dq - j.derivFilt);
  else           j.derivFilt  = dq;
  j.qMedAnt = j.qMed;

  if (fabsf(e) < j.banda) {
    // Dentro de la banda muerta no se para el lazo del todo: se deja el
    // integral y el freno para que el brazo no ceda, pero sin excitar el motor.
    j.saturada = false;
    return 0.0f;
  }

  float vRef = (j.qRef - j.qRefAnt) / TS;
  float uPd  = j.kp * e - j.kd * j.derivFilt + j.kv * vRef;

  float uTent = uPd + j.integral;
  bool sat        = (uTent > j.pwmMax) || (uTent < -j.pwmMax);
  bool mismoSigno = (uTent > 0.0f && e > 0.0f) || (uTent < 0.0f && e < 0.0f);
  if (!sat || !mismoSigno) {
    j.integral += j.ki * e * TS;
    j.integral = constrain(j.integral, -j.pwmMax, j.pwmMax);
  }

  float u = uPd + j.integral;
  j.saturada = (fabsf(u) > j.pwmMax);
  u = constrain(u, -j.pwmMax, j.pwmMax);

  // Suelo de arranque: si el lazo pide algo, que pida al menos lo que hace
  // falta para que el eje se mueva.
  if (fabsf(u) < j.pwmMin) u = (u >= 0.0f) ? j.pwmMin : -j.pwmMin;
  return u;
}

// ============================================================================
// 9. LAZO DE CONTROL COMPLETO
// ============================================================================
static void tick() {
  uint32_t ahora = millis();
  enlaceVivo = (ahora - ultimaOrden) < TIMEOUT_MS;

  for (int i = 0; i < NJ; i++) {
    Junta& j = J[i];
    j.qMed = leerJunta(j);

    // Sin enlace se congela la referencia donde este. Se sigue sosteniendo el
    // brazo: soltar los motores por perder el USB seria la peor reaccion
    // posible, porque el eje vertical se viene abajo. Una meta de M o H no
    // caduca: se tecleo una vez y no va a llegar ninguna mas.
    float destino = (enlaceVivo || metaFija) ? j.qCmd : j.qRef;
    destino = constrain(destino, j.qMin, j.qMax);

    // Rampa de referencia limitada en velocidad. Absorbe los saltos de
    // consigna: da igual que llegue un objetivo al otro lado del espacio de
    // trabajo, el brazo sale hacia el a la velocidad que puede seguir.
    j.qRefAnt = j.qRef;
    float paso = j.vMax * TS;
    float d = destino - j.qRef;
    j.qRef += constrain(d, -paso, paso);

    if (!habilitado || !ceroHecho) {
      j.integral = 0.0f;
      j.qRef = j.qRefAnt = j.qCmd = j.qMed;
      j.u = 0.0f;
      continue;
    }

    j.u = control(j);

    // Tope por software: si el eje ya esta fuera de recorrido, solo se le deja
    // empujar hacia dentro. Es la unica proteccion que hay sin finales de
    // carrera, asi que conviene que qMin/qMax sean de verdad alcanzables.
    // Estar EN el tope no es una anomalia: la pose de referencia deja el
    // husillo justo en d3_min. Lo que hay que avisar es que el lazo siga
    // empujando hacia fuera, que es cuando el tope esta haciendo de freno.
    j.tocaLimite = (j.qMed <= j.qMin && j.u < 0.0f) || (j.qMed >= j.qMax && j.u > 0.0f);
    if (j.tocaLimite) j.u = 0.0f;

    // --- proteccion de bloqueo y de sentido invertido ---
    if (fabsf(j.u) >= j.pwmMax - 1e-3f) {
      int8_t s = (j.u > 0.0f) ? 1 : -1;
      if (s != j.signoBloqueo) {          // acaba de saturar, o de cambiar de lado
        j.signoBloqueo = s;
        j.qBloqueo = j.qMed;
        j.tBloqueo = ahora;
      }
      float avance = s * (j.qMed - j.qBloqueo);   // > 0: va hacia donde se le empuja
      if (avance > MOV_MINIMO * j.banda) {
        j.qBloqueo = j.qMed;              // se mueve bien: el reloj vuelve a cero
        j.tBloqueo = ahora;
      } else if (avance < -MOV_ESCAPE * j.banda) {
        juntaFallo = (int8_t) i;          // se mueve, pero al reves
        falloSentido = true;
      } else if (ahora - j.tBloqueo > MS_BLOQUEO) {
        juntaFallo = (int8_t) i;          // no se mueve
        falloSentido = false;
      }
    } else {
      j.signoBloqueo = 0;
      j.qBloqueo = j.qMed;
      j.tBloqueo = ahora;
    }

    // u = 0 significa "quieto", y quieto con un eje vertical es frenado: dejar
    // el motor suelto haria que la herramienta se descolgase sola.
    if (j.u == 0.0f) frenarMotor(j);
    else             escribirMotor(j, j.u);
  }

  if (juntaFallo >= 0 && habilitado) {
    habilitado = false;
    if (falloSentido)
      Serial.printf("! SENTIDO INVERTIDO en j%d (%s): empujando a tope se aleja de "
                    "la consigna, potencia cortada. Muevela a mano en su sentido +: "
                    "si q baja, manda D %d -1; si sube, I %d -1 (o cruza sus cables).\n",
                    juntaFallo, J[juntaFallo].nombre, juntaFallo, juntaFallo);
    else
      Serial.printf("! BLOQUEO en j%d (%s): saturada %lu ms sin moverse, potencia "
                    "cortada. Revisa el encoder y el eje.\n",
                    juntaFallo, J[juntaFallo].nombre, MS_BLOQUEO);
  }
  // Sin potencia, todo frenado... salvo la junta de una prueba A en marcha.
  if (!habilitado || !ceroHecho)
    for (int i = 0; i < NJ; i++)
      if (i != pruebaJ) { J[i].u = 0.0f; frenarMotor(J[i]); }

  // --- prueba en lazo abierto (orden A) ---
  if (pruebaJ >= 0) {
    Junta& j = J[pruebaJ];
    if ((int32_t) (ahora - pruebaFin) < 0) {
      j.u = pruebaU;
      escribirMotor(j, pruebaU);
      // Sondeo de cada canal, aparte de la ISR: dice cual de los dos se mueve
      // cuando las cuentas no cuadran (con un canal muerto salen +-1 y nada mas).
      uint8_t a = (j.encA == SIN_PIN) ? 0 : digitalRead(j.encA);
      uint8_t b = digitalRead(j.encB);
      if (a != pruebaUltA) { pruebaCambiosA++; pruebaUltA = a; }
      if (b != pruebaUltB) { pruebaCambiosB++; pruebaUltB = b; }
    } else {
      j.u = 0.0f;
      frenarMotor(j);
      int32_t mov = j.cuentas - pruebaDesde;
      float movUd = j.sentido * (float) mov / j.cuentasUd;
      // Lo que importa antes de cerrar el lazo: que haya cuentas, y que vayan
      // hacia donde empuja u. Al reves, el PID se escaparia.
      const char* nota = (mov == 0) ? "ninguna: el motor no giro o el encoder no llega"
                       : (j.encA == SIN_PIN)
                           ? "encoder de un canal: el sentido lo pone u, mira a ojo que "
                             "gire hacia + (si no, I j -1)"
                       : ((movUd > 0.0f) != (pruebaU > 0.0f))
                           ? "AL REVES de u: con el lazo cerrado se escaparia "
                             "(D j -1, o I j -1 si no se pueden cruzar los cables)"
                           : "sentido correcto";
      Serial.printf("! prueba j%d (%s): %+ld cuentas = %+.4f %s, %s | cambios A/B por "
                    "sondeo %u/%u\n", pruebaJ, j.nombre, (long) mov, movUd,
                    (pruebaJ == 2) ? "m" : "rad", nota, pruebaCambiosA, pruebaCambiosB);
      pruebaJ = -1;
    }
  }
}

// ============================================================================
// 10. PROTOCOLO SERIE
// ============================================================================
static uint8_t checksum(const char* s, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= (uint8_t) s[i];
  return c;
}

static uint8_t banderas() {
  uint8_t f = 0;
  if (habilitado) f |= 0x01;
  if (enlaceVivo) f |= 0x02;
  if (ceroHecho)  f |= 0x04;
  for (int i = 0; i < NJ; i++) {
    if (J[i].tocaLimite) f |= 0x08;
    if (J[i].saturada)   f |= 0x10;
  }
  if (juntaFallo >= 0) f |= 0x20;
  return f;
}

static void enviarTelemetria() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
                   "> %lu %.5f %.5f %.5f %.3f %.3f %.3f %u",
                   (unsigned long) (millis() - tArranque),
                   J[0].qMed, J[1].qMed, J[2].qMed,
                   J[0].u, J[1].u, J[2].u, banderas());
  if (n < 0 || n >= (int) sizeof(buf)) n = strlen(buf);
  Serial.printf("%s*%02X\n", buf, checksum(buf, n));
}

static void volcarConfig() {
  Serial.printf("# scara_esp32  Ts=%.0f ms  pwm=%d Hz/%d bits  habilitado=%d cero=%d\n",
                TS * 1000.0f, (int) pwmFreq, PWM_BITS, habilitado, ceroHecho);
  for (int i = 0; i < NJ; i++) {
    Junta& j = J[i];
    char canalA[4];
    if (j.encA == SIN_PIN) strcpy(canalA, "-");
    else snprintf(canalA, sizeof(canalA), "%d", j.encA);
    Serial.printf("# j%d %-11s in1/in2=%d/%d ena=%d%s enc=%s/%d%s  cuentas/ud=%.2f "
                  "sentido=%+.0f%s\n",
                  i, j.nombre, j.in1, j.in2, j.ena, enaOk[i] ? "" : " (SIN PWM)",
                  canalA, j.encB,
                  (j.encA == SIN_PIN) ? " (un canal)" : "", j.cuentasUd, j.sentido,
                  j.invMotor ? "  motor invertido" : "");
    Serial.printf("#    q=[%+.4f, %+.4f] vmax=%.4f  kp=%.3f ki=%.3f kd=%.4f kv=%.3f\n",
                  j.qMin, j.qMax, j.vMax, j.kp, j.ki, j.kd, j.kv);
    Serial.printf("#    pwm=[%.2f, %.2f] banda=%.4f  medido=%+.4f cuentas=%ld\n",
                  j.pwmMin, j.pwmMax, j.banda, j.qMed, (long) (j.cuentas - j.cero));
  }
}

// Trocea la linea en campos separados por espacios o comas.
static int trocear(char* s, char* campos[], int maxCampos) {
  int n = 0;
  char* p = strtok(s, " ,\t");
  while (p && n < maxCampos) { campos[n++] = p; p = strtok(NULL, " ,\t"); }
  return n;
}

static void procesarLinea(char* linea) {
  // Checksum opcional: si la linea trae "*XX" se comprueba, si no, se acepta
  // igual para poder teclear ordenes a mano en el monitor serie.
  char* ast = strrchr(linea, '*');
  if (ast) {
    *ast = '\0';
    uint8_t esperado = (uint8_t) strtol(ast + 1, NULL, 16);
    if (checksum(linea, strlen(linea)) != esperado) {
      Serial.println("# checksum incorrecto, linea descartada");
      return;
    }
  }

  char* c[10];
  int n = trocear(linea, c, 10);
  if (n == 0) return;
  char cmd = toupper(c[0][0]);

  // Un indice de junta valido es requisito de casi todo lo demas.
  int j = -1;
  if (n >= 2 && strchr("KCDRPMAI", cmd)) {
    j = atoi(c[1]);
    if (j < 0 || j >= NJ) { Serial.println("# junta fuera de rango"); return; }
  }

  switch (cmd) {
    case 'T':
      if (n < 1 + NJ) { Serial.println("# T necesita 3 valores"); return; }
      for (int i = 0; i < NJ; i++)
        J[i].qCmd = constrain(atof(c[1 + i]), J[i].qMin, J[i].qMax);
      ultimaOrden = millis();
      metaFija = false;                 // manda el flujo: una meta de M/H deja de valer
      break;

    case 'E':
      if (n < 2) return;
      if (atoi(c[1])) {
        if (!ceroHecho) { Serial.println("! sin cero: manda Z antes de habilitar"); return; }
        // Al habilitar, la consigna arranca donde esta el brazo: si no, el
        // primer ciclo veria un error enorme y saldria disparado.
        for (int i = 0; i < NJ; i++) {
          J[i].qMed = J[i].qMedAnt = leerJunta(J[i]);
          J[i].qRef = J[i].qRefAnt = J[i].qCmd = J[i].qMed;
          J[i].integral = 0.0f;
          J[i].derivFilt = 0.0f;
          J[i].qBloqueo = J[i].qMed;
          J[i].tBloqueo = millis();
          J[i].signoBloqueo = 0;
        }
        juntaFallo = -1;
        pruebaJ = -1;                   // una prueba A en marcha se acaba aqui
        habilitado = true;
        ultimaOrden = millis();
        Serial.println("! potencia habilitada");
      } else {
        habilitado = false;
        pruebaJ = -1;                   // E 0 tambien para una prueba A
        frenarTodo();
        Serial.println("! potencia deshabilitada (motores frenados)");
      }
      break;

    case 'Z':
      // Cero manual. Con encoders incrementales no hay posicion absoluta: al
      // encender, la placa no sabe donde esta el brazo. Se coloca a mano en la
      // pose de referencia del URDF y esto la fija como origen.
      ponerCero();
      Serial.println("! cero establecido en la pose actual");
      break;

    case 'H':
      if (!habilitado) { Serial.println("! potencia deshabilitada: manda E 1 antes"); return; }
      for (int i = 0; i < NJ; i++)
        J[i].qCmd = constrain((J[i].qMin > 0.0f) ? J[i].qMin : 0.0f,
                              J[i].qMin, J[i].qMax);
      metaFija = true;
      // Solo util tecleandolo a mano: con el puente conectado, las lineas T
      // que llegan a 50 Hz pisan esta consigna en el ciclo siguiente. Desde
      // ROS el home se pide a scara_ik (servicio /scara/home).
      Serial.println("! volviendo a la pose de referencia");
      break;

    case 'M': {
      // Una junta sola, las demas quietas: es la orden para probar un motor
      // suelto desde el monitor serie o desde "teodoro", sin ROS. Con el puente
      // conectado no sirve, por lo mismo que H.
      if (n < 3) { Serial.println("# M j q"); return; }
      if (!habilitado) { Serial.println("! potencia deshabilitada: manda E 1 antes"); return; }
      // Si lo ultimo fue el flujo de T, lo que quede en qCmd es una muestra
      // vieja: las otras juntas se quedan en su referencia en vez de ir a ella.
      if (!metaFija)
        for (int i = 0; i < NJ; i++) J[i].qCmd = J[i].qRef;
      float q = atof(c[2]);
      J[j].qCmd = constrain(q, J[j].qMin, J[j].qMax);
      metaFija = true;
      Serial.printf("! j%d %s -> %.5f%s\n", j, J[j].nombre, J[j].qCmd,
                    (J[j].qCmd != q) ? "  (recortada al recorrido)" : "");
      break;
    }

    case 'A': {
      // Prueba en lazo abierto: el ciclo de trabajo u (-1..1) en la junta j
      // durante ms milisegundos, y cuantas cuentas se ha movido. Sin PID ni
      // cero: ensena justo lo que el lazo cerrado no deja ver, si el motor gira,
      // hacia donde y si el encoder cuenta. Solo con la potencia cortada, para
      // no pelearse con el lazo, y como mucho 1 s.
      if (n < 4) { Serial.println("# A j u ms"); return; }
      if (habilitado) { Serial.println("! la prueba A va con la potencia cortada (E 0)"); return; }
      float u = constrain((float) atof(c[2]), -J[j].pwmMax, J[j].pwmMax);
      uint32_t ms = (uint32_t) constrain(atol(c[3]), 0L, 1000L);
      pruebaDesde = J[j].cuentas;
      pruebaCambiosA = pruebaCambiosB = 0;
      pruebaUltA = (J[j].encA == SIN_PIN) ? 0 : digitalRead(J[j].encA);
      pruebaUltB = digitalRead(J[j].encB);
      pruebaU = u;
      pruebaFin = millis() + ms;
      pruebaJ = (int8_t) j;
      Serial.printf("! prueba j%d (%s): u=%+.2f durante %lu ms\n", j, J[j].nombre, u,
                    (unsigned long) ms);
      break;
    }

    case 'K':
      if (n < 6) { Serial.println("# K j kp ki kd kv"); return; }
      J[j].kp = atof(c[2]); J[j].ki = atof(c[3]);
      J[j].kd = atof(c[4]); J[j].kv = atof(c[5]);
      J[j].integral = 0.0f;
      Serial.printf("! j%d kp=%.3f ki=%.3f kd=%.4f kv=%.3f\n",
                    j, J[j].kp, J[j].ki, J[j].kd, J[j].kv);
      break;

    case 'C':
      if (n < 3) { Serial.println("# C j cuentas_por_unidad"); return; }
      J[j].cuentasUd = atof(c[2]);
      if (J[j].cuentasUd < 1e-6f) J[j].cuentasUd = 1.0f;
      Serial.printf("! j%d cuentas/ud=%.3f\n", j, J[j].cuentasUd);
      break;

    case 'D':
      if (n < 3) { Serial.println("# D j sentido"); return; }
      J[j].sentido = (atof(c[2]) < 0.0f) ? -1.0f : +1.0f;
      Serial.printf("! j%d sentido=%+.0f\n", j, J[j].sentido);
      break;

    case 'I':
      // Invierte el motor, no el encoder: es cruzar sus cables sin tocarlos.
      // En una junta de un canal es la unica forma de arreglar el sentido,
      // porque ahi las cuentas siguen siempre al motor y D invertiria el lazo.
      if (n < 3) { Serial.println("# I j +1|-1"); return; }
      J[j].invMotor = (atof(c[2]) < 0.0f);
      Serial.printf("! j%d motor %s\n", j,
                    J[j].invMotor ? "invertido (IN1/IN2 cruzados por software)" : "normal");
      break;

    case 'R':
      if (n < 5) { Serial.println("# R j qmin qmax vmax"); return; }
      J[j].qMin = atof(c[2]); J[j].qMax = atof(c[3]); J[j].vMax = atof(c[4]);
      Serial.printf("! j%d q=[%+.4f, %+.4f] vmax=%.4f\n",
                    j, J[j].qMin, J[j].qMax, J[j].vMax);
      break;

    case 'P':
      if (n < 4) { Serial.println("# P j pwmmin pwmmax"); return; }
      J[j].pwmMin = constrain((float) atof(c[2]), 0.0f, 1.0f);
      J[j].pwmMax = constrain((float) atof(c[3]), 0.0f, 1.0f);
      Serial.printf("! j%d pwm=[%.2f, %.2f]\n", j, J[j].pwmMin, J[j].pwmMax);
      break;

    case 'F':
      if (n < 2) return;
      telemetriaHz = constrain((float) atof(c[1]), 0.0f, 200.0f);
      Serial.printf("! telemetria %.1f Hz\n", telemetriaHz);
      break;

    case 'W': {
      // Frecuencia del PWM de los tres ENA, en caliente, para dar con una que el
      // driver siga. Solo con la potencia cortada; los motores quedan frenados.
      if (n < 2) { Serial.println("# W hz"); return; }
      if (habilitado) { Serial.println("! W va con la potencia cortada (E 0)"); return; }
      uint32_t hz = (uint32_t) constrain(atol(c[1]), 100L, 40000L);
      bool ok = true;
      for (int i = 0; i < NJ; i++) {
        ok = ok && (ledcChangeFrequency(J[i].ena, hz, PWM_BITS) != 0);
        frenarMotor(J[i]);
      }
      if (ok) pwmFreq = hz;
      Serial.printf("! pwm a %lu Hz%s\n", (unsigned long) hz,
                    ok ? "" : " (FALLO al cambiarla)");
      break;
    }

    case '?':
      volcarConfig();
      break;

    default:
      Serial.printf("# orden desconocida '%c'\n", cmd);
  }
}

static void leerSerie() {
  static char buf[128];
  static int  n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[n] = '\0'; if (n) procesarLinea(buf); n = 0; continue; }
    if (n < (int) sizeof(buf) - 1) buf[n++] = ch;
    else n = 0;                       // linea absurda: se tira y se resincroniza
  }
}

// ============================================================================
// 11. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NJ; i++) {
    Junta& j = J[i];

    // Las direcciones a LOW antes de habilitar el puente: si se hiciera al
    // reves, el motor recibiria tension durante el instante en que IN1/IN2
    // todavia estan flotando.
    pinMode(j.in1, OUTPUT); digitalWrite(j.in1, LOW);
    pinMode(j.in2, OUTPUT); digitalWrite(j.in2, LOW);
    enaOk[i] = ledcAttach(j.ena, pwmFreq, PWM_BITS);   // '?' avisa si fallo
    frenarMotor(j);                   // frenado antes que nada, nunca suelto

    pinMode(j.encB, INPUT_PULLUP);    // todos los pines de encoder tienen pull-up
    j.cuentas = 0;
    j.cero = 0;
    j.dirMotor = 1;

    float porVuelta = CUENTAS_VUELTA_EJE * REDUCCION_EXTRA[i];
    if (j.encA == SIN_PIN) {
      // Un solo canal: cada flanco de B es un paso, hacia donde diga el motor
      // (isrUnCanal). Son la mitad de flancos que con los dos canales.
      attachInterruptArg(digitalPinToInterrupt(j.encB), isrUnCanal, &j, CHANGE);
      porVuelta *= 0.5f;
    } else {
      pinMode(j.encA, INPUT_PULLUP);
      j.estadoEnc = (digitalRead(j.encA) << 1) | digitalRead(j.encB);
      attachInterruptArg(digitalPinToInterrupt(j.encA), isrEncoder, &j, CHANGE);
      attachInterruptArg(digitalPinToInterrupt(j.encB), isrEncoder, &j, CHANGE);
    }

    // cuentas por unidad de junta a partir de la mecanica
    j.cuentasUd = (i == 2) ? (porVuelta / PASO_HUSILLO) : (porVuelta / DOS_PI);
  }

  delay(300);
  Serial.println("# =====================================================");
  Serial.println("# scara_esp32 - SCARA scara_urdf, 3 GDL (sin muneca), JGA25-370");
  Serial.println("# Arranca DESHABILITADO y SIN CERO, a proposito.");
  Serial.println("#   1) coloca el brazo en la pose de referencia del URDF");
  Serial.println("#   2) manda  Z   para fijar el cero");
  Serial.println("#   3) manda  E 1 para dar potencia");
  Serial.println("# 'M j q' mueve una junta sola: M 2 0.05 sube el husillo a 50 mm");
  Serial.println("# '?' vuelca la configuracion completa");
  Serial.println("# =====================================================");

  tArranque = millis();
  tLazo = micros();
}

// ============================================================================
// 12. BUCLE PRINCIPAL
// ============================================================================
void loop() {
  leerSerie();

  uint32_t ahora = micros();
  if ((uint32_t) (ahora - tLazo) >= TS_US) {
    tLazo += TS_US;                   // periodo fijo, sin deriva acumulada
    // Si el lazo se ha retrasado mas de un periodo entero (una impresion larga,
    // por ejemplo) se resincroniza en vez de intentar recuperar el tiempo
    // perdido a base de ciclos seguidos.
    if ((uint32_t) (micros() - tLazo) >= TS_US) tLazo = micros();
    tick();
  }

  if (telemetriaHz > 0.0f) {
    uint32_t periodo = (uint32_t) (1000.0f / telemetriaHz);
    if (millis() - tTelemetria >= periodo) {
      tTelemetria = millis();
      enviarTelemetria();
    }
  }
}
