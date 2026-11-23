# Laboratorio de Robótica — Ensamblador 2 (Robot SCARA RRP)
**Universidad Militar Nueva Granada — Facultad de Ingeniería**  
**Programa de Ingeniería Mecatrónica — Laboratorio de Robótica 2026**  
**Grupo de Trabajo: Grupo #3**

Bienvenido a la documentación y repositorio oficial del **Ensamblador 2**, correspondiente al robot manipulador tipo SCARA implementado para la celda de manufactura y empaque automatizado de chocolates.

---

## 1. Contexto de la Solución Implementada

En el marco de la línea integrada de empaque de la planta de producción, el **Ensamblador 2** cumple la función crítica de recolectar unidades de producto (chocolates) desde la estación de alimentación/transporte intermedio y colocarlas de forma ordenada, secuencial y milimétrica en el interior de cajas de empaque en movimiento o indexadas.

Para lograr esta tarea sin comprometer la integridad estructural ni la apariencia de los chocolates, se diseñó e implementó un robot de cinemática **SCARA RRP (Rotacional - Rotacional - Prismático)** con las siguientes características:
- **Alta velocidad y rigidez en el plano horizontal ($XY$):** Dos eslabones rotacionales impulsados por poleas síncronas GT2 para movimientos rápidos y estables.
- **Precisión en la inserción vertical ($Z$):** Mecanismo de piñón y cremallera módulo 2 guiado por varillas rectificadas y rodamientos LM8UU con carrera útil de $90\text{ mm}$ (carrera mecánica de $150\text{ mm}$).
- **Manipulación no destructiva:** Pinza paralela accionada por micromotor N20 con control PWM bipolar regulado para garantizar agarres firmes y delicados ($F \approx 2.5\text{ N}$).
- **Coordinación con el Maestro de Celda:** Total integración al protocolo de comunicaciones de la planta mediante ROS 2 Jazzy, respondiendo a órdenes en tiempo real, emitiendo heartbeat a 2 Hz y acatando paradas de emergencia en menos de 10 ms.

---

## 2. Arquitectura General de la Solución

El sistema se estructura en tres niveles jerárquicos de automatización:
1. **Nivel de Coordinación de Celda (Planta / Nodo Maestro):** Comunicación desacoplada sobre DDS (`ROS_DOMAIN_ID=42`), gestionando sincronización por estados (`listo`, `ocupado`, `error`).
2. **Nivel de Planificación y Supervisión (ROS 2 Jazzy / Linux Ubuntu 24.04):**
   - Planificador de trayectorias con curvas suaves S-curve (Minimum Jerk) y cinemática inversa con selección de configuración de codo.
   - Driver serie bidireccional a 100 Hz con control de flujo y flushing inmediato.
   - Interfaces gráficas interactivas para operación local y de planta con telemetría en tiempo real.
   - Gemelo digital en Gazebo Sim y RViz sincronizado en espejo 1:1 a 20 Hz.
3. **Nivel Embebido y Ejecución de Potencia (ESP32 / MicroPython / L298N):**
   - Interrupciones de hardware en cuadratura para encoders de efecto Hall (990 pulsos/rev).
   - Lazo cerrado de velocidad articular a 100 Hz con Feedforward directo y Feedback PI por temporizador periódico de hardware.
   - Control bipolar PWM con protección térmica y sobretensión para actuadores.

```mermaid
graph LR
    Maestro["🏭 Maestro de Planta\n(/ensamblador2/orden, /estado)"] <-->|ROS 2 Domain 42| PC["💻 Nodo ROS 2 Jazzy\n(Cinemática, Serial, GUI, Gazebo)"]
    PC <-->|UART USB 115200 baud| ESP["⚡ ESP32 MicroPython\n(Control PI 100Hz, Encoders)"]
    ESP --> Driver["🔌 Drivers Puente H L298N"]
    Driver --> Robot["🦾 Robot Físico SCARA RRP + Gripper N20"]
    Robot -->|Pulsos Encoders| ESP
```

---

## 3. Equipo de Trabajo y Responsabilidades (Grupo #3)

| Integrante | Rol en el Proyecto | Responsabilidades Principales | Contacto Institucional |
| :--- | :--- | :--- | :--- |
| **Samuel David Azuero Forero** | Líder Mecánico y Cinemática | Diseño CAD en SolidWorks, cálculo cinemático analítico, generación de URDF, calibración de transmisiones y pruebas físicas de repetibilidad. | `est.samuel.azuero@unimilitar.edu.co` |
| **Dilhann Adrián Yañez Nieves** | Líder Eléctrico y Potencia | Diseño de esquemas de cableado, conexionado de puentes H L298N, alimentación de potencia 12V/5V, protección del gripper N20 y montaje en celda. | `est.dilhann.yanez@unimilitar.edu.co` |
| **Ana María Iglesias Roa** | Líder de Control y Firmware | Modelado y diseño de controladores PI de velocidad articular, firmware MicroPython en ESP32, sintonización de ganancias y análisis de estabilidad. | `est.ana.iglesias@unimilitar.edu.co` |
| **Julian Camilo Paez Colmenares** | Líder de Software y Simulación | Paquetes ROS 2 Jazzy, gemelo digital en Gazebo/RViz, integración con nodo maestro de planta, interfaz gráfica Tkinter y scripts de lanzamiento. | `est.julian.paez@unimilitar.edu.co` |

---

## 4. Estructura de Navegación del Repositorio

A continuación se detalla la organización de los directorios de este módulo:

```text
Ensambladores/Ensamblador 2/
├── .gitignore                                 # Exclusión de binarios y ficheros temporales
├── README.md                                  # Este documento principal
└── SCARA/                                     # Módulo integral del robot manipulador
    ├── README.md                              # Ficha técnica y visión de subsistemas
    ├── CODIGOS/                               # Códigos fuente implementados y comentados
    │   ├── README.md                          # Guía de compilación y ejecución de código
    │   ├── ESP32/                             # Firmware para microcontrolador
    │   │   ├── README.md                      # Explicación de arquitectura embebida
    │   │   └── main.py                        # Firmware MicroPython a 100 Hz
    │   └── ROS2/                              # Paquetes y scripts para ROS 2 Jazzy
    │       ├── README.md                      # Documentación de nodos, tópicos y servicios
    │       ├── 1_lanzar_gazebo.sh             # Script: Lanzar simulación Gazebo
    │       ├── 2_lanzar_control_esp32.sh      # Script: Lanzar driver serial + cinemática
    │       ├── 3_lanzar_gui.sh                # Script: Lanzar GUI de control local
    │       ├── 4_lanzar_planta_ensamblador2.sh # Script: Lanzar integración de planta
    │       ├── simulador_maestro.py           # Herramienta de test de contrato de planta
    │       ├── scara_control/                 # Paquete principal de nodos de control y GUI
    │       └── ENSAMBLE/                      # Paquete de URDF, mallas STL y simulación
    ├── Codigo_Control_Fisico/                 # Modelos de control y scripts de validación
    │   ├── README.md                          # Puente entre simulación analítica y hardware
    │   ├── control_de_posicon_motor.mlx       # Identificación de sistemas y lazos de control
    │   ├── control_de_posicion_simu.slx       # Simulación de control en Simulink
    │   ├── Simfinal.slx                       # Modelo dinámico completo
    │   └── Prueba.mlx                         # Rutinas de prueba cinemática
    └── Documentacion/                         # Memorias, diagramas, planos y manuales
        ├── README.md                          # Índice general de documentación de soporte
        ├── Memorias de Calculo/               # Deducciones cinemáticas, dinámicas y eléctricas
        ├── DIAGRAMAS/                         # Diagramas de arquitectura, flujo y secuencia
        ├── Hojas de Especificaciones/         # Fichas técnicas de componentes utilizados
        ├── Manual de Usuario/                 # Guía paso a paso de encendido y operación
        ├── Planos Mecánicos y eléctricos/     # Planos de ensamble, despiece y conexiones
        ├── Plano de planta/                   # Distribución en la celda de manufactura
        ├── Diagrama de nodos/                 # Topología de nodos ROS 2 y mensajería
        └── Actas de trabajo/                  # Registro de reuniones y seguimiento de equipo
```

---

## 5. Resumen de Desempeño y Validación Experimental

En las pruebas experimentales sobre el robot físico real montado en celda se obtuvieron los siguientes resultados:
- **Repetibilidad Cartesiana:** Error de posición menor a **$2.3\text{ mm}$** entre ciclos continuos alternantes.
- **Acumulación de Error:** **$0.0\text{ mm}$** de deriva residual gracias a la calibración de signos de encoder y vaciado de búfer serie inmediato.
- **Tiempo de Ciclo Pick & Place:** **$8.2\text{ s}$** por ciclo completo de chocolate (12 pasos con velocidades seguras).
- **Consumo y Rendimiento Térmico:** Calentamiento de drivers L298N $< 42^\circ\text{C}$ continuo a 1000 Hz.
