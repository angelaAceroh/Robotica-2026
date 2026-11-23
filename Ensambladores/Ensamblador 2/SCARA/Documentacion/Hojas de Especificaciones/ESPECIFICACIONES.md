# Hojas de Especificaciones Técnicas — Componentes del Robot SCARA (Ensamblador 2)
**Universidad Militar Nueva Granada — Laboratorio de Robótica**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

---

## 1. Microcontrolador: ESP32 NodeMCU-32S (Espressif Systems)

| Parámetro | Especificación Técnica |
| :--- | :--- |
| **Arquitectura** | Dual Core Xtensa® 32-bit LX6 |
| **Frecuencia de Reloj** | 240 MHz |
| **Memoria SRAM** | 520 KB |
| **Memoria Flash Externa** | 4 MB (SPI) |
| **Tensión de Operación Lógica** | 3.3V DC (Niveles lógicos compatibles con 5V pull-up) |
| **Conectividad Periférica** | UART (hasta 5 Mbps), SPI, I2C, PWM por hardware (LEDC), ADC, DAC |
| **Entorno de Ejecución** | MicroPython v1.23+ optimizado para robótica de tiempo real |
| **Temporizadores de Hardware** | 4x Timers de 64 bits (Timer 0 configurado para interrupciones periódicas a 100 Hz / 10 ms) |
| **Gestión de Entradas/Salidas** | Soporte de interrupciones externas en todos los GPIO digitales (flanco de subida/bajada) |

---

## 2. Motores Principales: JGA25-370 con Reductor Planetario y Encoder

| Parámetro | Especificación Técnica |
| :--- | :--- |
| **Tipo de Motor** | DC con escobillas de carbón y bobinado de cobre |
| **Tensión Nominal** | 12V DC |
| **Velocidad sin Carga** | 60 RPM (en el eje de salida del reductor) |
| **Corriente sin Carga** | $\le 100\text{ mA}$ |
| **Corriente a Rotor Bloqueado** | $1.2\text{ A}$ |
| **Torque Nominal** | $2.5\text{ kg}\cdot\text{cm}$ ($0.245\text{ N}\cdot\text{m}$) |
| **Torque a Rotor Bloqueado** | $8.0\text{ kg}\cdot\text{cm}$ ($0.784\text{ N}\cdot\text{m}$) |
| **Sensor de Posición** | Encoder magnético de efecto Hall en cuadratura (Canales A y B) |
| **Resolución del Encoder** | 11 pulsos/rev en rotor $\times$ relación reductora $90:1$ = **990 pulsos/rev** en eje de salida |
| **Tensión de Alimentación del Sensor** | 3.3V a 5.0V DC |

---

## 3. Controladores de Potencia: Driver Dual Puente H L298N

| Parámetro | Especificación Técnica |
| :--- | :--- |
| **Circuito Integrado** | L298N (STMicroelectronics) |
| **Topología** | Puente H dual de transistores bipolares con salida Darlington |
| **Tensión de Alimentación de Potencia ($V_s$)** | 12.0V DC (Rango admitido: 5V a 35V DC) |
| **Tensión de Control Lógico ($V_{ss}$)** | 5.0V DC (generada por regulador onboard 78M05) |
| **Corriente Pico de Salida por Canal** | 2.0 A continuos / 3.0 A no repetitivos |
| **Frecuencia Máxima de Conmutación** | 40 kHz (Operado a 1000 Hz para optimizar rendimiento térmico) |
| **Caída de Tensión Interna ($V_{\text{drop}}$)** | $1.8\text{V} - 2.0\text{V}$ típica a 1 A |
| **Módulos Instalados en el Robot** | 2 unidades independientes (L298N #1: Motores J1 y J2; L298N #2: Motor J3 y Gripper N20) |

---

## 4. Micromotor de Efector Final: N20 con Husillo Roscado

| Parámetro | Especificación Técnica |
| :--- | :--- |
| **Tensión Nominal** | 3.0V DC |
| **Rango Seguro de Operación** | 1.8V a 3.0V DC (Alcanzado mediante PWM al 18% - 25% desde bus de 12V) |
| **Mecanismo de Salida** | Husillo roscado M3 de acero inoxidable (paso 0.5 mm) |
| **Accionamiento de Mordaza** | Traslación paralela simétrica para agarre de piezas ligeras |
| **Corriente en Carga Ligera** | $\approx 80\text{ mA}$ |
| **Fuerza de Sujeción Efectiva** | $\approx 2.5\text{ N}$ (Suficiente para piezas de chocolate de $25\text{ g}$ sin aplastamiento) |

---

## 5. Elementos de Transmisión Mecánica

| Elemento | Especificación |
| :--- | :--- |
| **Correas Dentadas** | Correa síncrona de caucho reforzado con fibra de vidrio, perfil GT2, paso 2 mm, ancho 6 mm |
| **Poleas Motrices** | Polea de aluminio GT2 de 20 dientes (diámetro primitivo $12.73\text{ mm}$, eje de 4 mm) |
| **Polea Conducida J1 (Hombro)** | Polea GT2 de 80 dientes en PETG alta densidad (Reducción 4:1) |
| **Polea Conducida J2 (Codo)** | Polea GT2 de 40 dientes en PETG alta densidad (Reducción 2:1) |
| **Cremallera Eje Z** | Módulo 2.0, longitud 150 mm, tallado recto de precisión |
| **Piñón Eje Z** | Módulo 2.0, 20 dientes ($D_p = 40\text{ mm}$), avance lineal de $125.66\text{ mm/rev}$ |
| **Guías Lineales Z** | Dos varillas de acero rectificado cromo duro de 8 mm con rodamientos lineales LM8UU |
