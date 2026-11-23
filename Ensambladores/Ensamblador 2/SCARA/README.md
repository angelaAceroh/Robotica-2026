# Manipulador SCARA RRP — Ensamblador 2
**Universidad Militar Nueva Granada — Laboratorio de Robótica 2026**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este directorio contiene todos los recursos de diseño, código fuente embebido y de supervisión, modelos de simulación y documentación técnica del robot manipulador SCARA asignado al puesto de **Ensamblador 2**.

---

## 1. Ficha Técnica General del Robot

| Parámetro | Valor de Diseño | Valor Experimental Físico |
| :--- | :--- | :--- |
| **Arquitectura Cinemática** | SCARA RRP (2 rotacionales + 1 prismático vertical) | Idéntico |
| **Grados de Libertad (GDL)** | 3 GDL de posicionamiento + 1 GDL en pinza de agarre | 4 actuadores independientes |
| **Longitud Eslabón 1 ($L_1$)** | $205.0\text{ mm}$ ($0.205\text{ m}$) | $205.0\text{ mm}$ |
| **Longitud Eslabón 2 ($L_2$)** | $130.0\text{ mm}$ ($0.130\text{ m}$) | $130.0\text{ mm}$ |
| **Radio de Alcance Máximo** | $335.0\text{ mm}$ ($0.335\text{ m}$) | $335.0\text{ mm}$ |
| **Radio de Alcance Mínimo** | $75.0\text{ mm}$ ($0.075\text{ m}$) | $75.0\text{ mm}$ |
| **Rango Angular Hombro ($J_1$)** | $[-135^\circ, +135^\circ]$ ($\approx \pm 2.35\text{ rad}$) | Acotado por software a $\pm 120^\circ$ |
| **Rango Angular Codo ($J_2$)** | $[-150^\circ, +150^\circ]$ ($\approx \pm 2.61\text{ rad}$) | Operado en codo izquierdo ($\sigma = -1.0$) |
| **Carrera Prismática Efectiva ($J_3$)** | $90.0\text{ mm}$ ($0.090\text{ m}$) | Rango real: $[8.0\text{ cm}, 17.0\text{ cm}]$ |
| **Carrera Mecánica Cremallera** | $150.0\text{ mm}$ ($0.150\text{ m}$) | Verificada en malla CAD STL |
| **Carga Útil Nominal (Payload)** | $100\text{ g}$ | Probada con bombones de $25\text{ g}$ |
| **Repetibilidad Cartesiana** | $\pm 2.0\text{ mm}$ | $< 2.3\text{ mm}$ medido en ciclos alternantes |
| **Efector Final** | Pinza paralela accionada por motor N20 con husillo | Regulación PWM a $1.8\text{V} - 2.5\text{V}$ |

---

## 2. Subsistemas del Robot

### 2.1. Subsistema Mecánico
- **Estructura Portante:** Diseñada en SolidWorks y fabricada mediante manufactura aditiva FDM en filamentos técnicos (PETG y PLA alta densidad) para máxima rigidez torsional.
- **Transmisión de Ejes $J_1$ y $J_2$:** Poleas síncronas de aluminio y correas de caucho con alma de fibra de vidrio GT2 (reducciones 4:1 en hombro y 2:1 en codo), garantizando cero deslizamiento y alta suavidad.
- **Transmisión de Eje $J_3$:** Mecanismo de piñón y cremallera módulo 2 ($D_p = 40\text{ mm}$) montado sobre columnas dobles de varilla rectificada de $8\text{ mm}$ y bujes lineales LM8UU.

### 2.2. Subsistema Electrónico y de Potencia
- **Unidad de Procesamiento Embebido:** Placa ESP32 NodeMCU-32S (Xtensa dual-core a 240 MHz).
- **Etapa de Potencia:** Dos drivers puente H dobles L298N operados a 1000 Hz.
- **Actuadores Principales:** 3 motores DC con reductor metálico planetario JGA25-370 a 12V y 60 RPM.
- **Sensores de Posición:** Encoders magnéticos de efecto Hall en cuadratura (990 pulsos/rev en eje de salida).
- **Protección de Efector:** Tensión limitada a máximo 3.0V en micromotor N20 para evitar daño térmico.

### 2.3. Subsistema de Software y Control
- **Capa Embebida (MicroPython):** Lazo de control PI a 100 Hz con Feedforward, watchdog serie y lectura no bloqueante.
- **Capa ROS 2 (Jazzy):** Nodos modulares para comunicación serial, cinemática inversa analítica, control cartesiano en línea recta por curva S, GUI en Tkinter y servicio de planta.
- **Gemelo Digital:** Entorno Gazebo Sim y RViz con modelos URDF exactos sincronizados en tiempo real.

---

## 3. Guía de Subdirectorios

1. **[`CODIGOS/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/CODIGOS/README.md):** Contiene el código fuente completo del firmware ESP32, los paquetes de ROS 2 (`scara_control` y `ENSAMBLE`) y los scripts de automatización de pruebas y puesta en marcha.
2. **[`Codigo_Control_Fisico/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Codigo_Control_Fisico/README.md):** Modelos de control y simulación en MATLAB/Simulink que sirvieron de base para la identificación y diseño de los lazos de control implementados físicamente.
3. **[`Documentacion/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/README.md):** Repositorio exhaustivo de ingeniería con memorias de cálculo analíticas, diagramas de arquitectura/secuencia/flujo, planos mecánicos/eléctricos, datasheets de componentes y manual de usuario.
