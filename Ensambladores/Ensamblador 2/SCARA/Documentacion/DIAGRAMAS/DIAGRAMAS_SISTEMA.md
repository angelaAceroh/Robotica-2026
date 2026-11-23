# Diagramas del Sistema — Robot Manipulador SCARA (Ensamblador 2)
**Universidad Militar Nueva Granada — Laboratorio de Robótica**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este documento consolida los diagramas de arquitectura, flujo de control, secuencia de procesos y cableado eléctrico del sistema robótico.

---

## 1. Arquitectura General del Sistema

El sistema implementa una arquitectura distribuida de tres capas:
1. **Capa de Supervisión y Coordinación (Planta / Maestro):** Gestión de estados de la celda de empaque mediante tópicos estándar de ROS 2.
2. **Capa de Control Cinemático y Visualización (PC de Control / ROS 2):** Planificación cartesiana, cinemática inversa, telemetría y GUI.
3. **Capa de Ejecución de Bajo Nivel y Potencia (ESP32 / Drivers / Actuadores):** Lazo de control PI a 100 Hz, interrupciones de encoders y modulación PWM.

```mermaid
graph TD
    subgraph "NODO MAESTRO / PLANTA (ROS_DOMAIN_ID=42)"
        M_ORDEN["/ensamblador2/orden (String)"]
        M_PARADA["/parada (Bool)"]
        M_ESTADO["/ensamblador2/estado (String - Heartbeat 2Hz)"]
        M_RESET["/ensamblador2/reset_estado (Trigger)"]
    end

    subgraph "ESTACIÓN DE CONTROL LOCAL (ROS 2 JAZZY)"
        GUI["scara_gui_planta_node / scara_gui_node\n(Interfaz Gráfica Tkinter)"]
        KIN["scara_kinematics_planta_node / scara_kinematics_node\n(Planificador S-Curve e IK)"]
        SER["scara_serial_node\n(Driver Comunicación Serial 100 Hz)"]
        GZ["Gazebo Sim / RViz\n(Gemelo Digital Espejo 1:1 a 20 Hz)"]
    end

    subgraph "MICROCONTROLADOR (ESP32 - MicroPython)"
        UART_RX["Parser Serie No Bloqueante (uselect.poll)"]
        CTRL_PI["Lazo de Control de Velocidad a 100 Hz (Timer 0)"]
        ISR_ENC["Interrupciones Hardware de Encoders (GPIO)"]
        PWM_GEN["Generador PWM Bipolar Hardware (LEDC)"]
    end

    subgraph "HARDWARE Y POTENCIA"
        L298_1["Driver L298N #1 (Hombro J1 y Codo J2)"]
        L298_2["Driver L298N #2 (Prismático J3 y Gripper N20)"]
        M1["Motor JGA25-370 J1 (Hombro) + Reductor GT2 4:1"]
        M2["Motor JGA25-370 J2 (Codo) + Reductor GT2 2:1"]
        M3["Motor JGA25-370 J3 (Prismático) + Cremallera Módulo 2"]
        M4["Micromotor N20 con Husillo (Gripper Pinza)"]
        ENC1["Encoder Hall J1"]
        ENC2["Encoder Hall J2"]
        ENC3["Encoder Hall J3"]
    end

    %% Conexiones Maestro - Cinemática
    M_ORDEN --> KIN
    M_PARADA --> KIN
    KIN --> M_ESTADO
    M_RESET --> KIN

    %% Conexiones Internas ROS 2
    GUI <-->|/cmd_cartesian_target, /cmd_gripper, /joint_states| KIN
    KIN -->|/cmd_joint_vel| SER
    KIN -->|/position_controller/commands| GZ
    KIN -->|/cmd_gripper| SER
    SER -->|/joint_states| KIN
    SER -->|/joint_states| GUI

    %% Conexiones ROS 2 - ESP32
    SER <==|UART USB (/dev/ttyUSB0 - 115200 baud)|==> UART_RX
    UART_RX --> CTRL_PI
    CTRL_PI --> PWM_GEN
    ISR_ENC --> CTRL_PI
    CTRL_PI -->|Telemetría P,c1,c2,c3 a 50 Hz| UART_RX

    %% Conexiones ESP32 - Potencia
    PWM_GEN --> L298_1
    PWM_GEN --> L298_2
    L298_1 --> M1
    L298_1 --> M2
    L298_2 --> M3
    L298_2 --> M4

    %% Conexiones Sensores - ESP32
    ENC1 --> ISR_ENC
    ENC2 --> ISR_ENC
    ENC3 --> ISR_ENC
```

---

## 2. Diagrama de Secuencia del Proceso Pick & Place y Sincronización con la Planta

```mermaid
sequenceDiagram
    autonumber
    participant Maestro as 🏭 Maestro de Planta
    participant Kin as 🤖 Cinemática Ensamblador 2
    participant Serial as 🔌 Driver Serial ROS 2
    participant ESP as ⚡ ESP32 Firmware
    participant Robot as 🦾 Manipulador Físico SCARA

    Note over Maestro, Kin: Estado Inicial: "listo" (Heartbeat 2 Hz)
    Maestro->>Kin: Publica /ensamblador2/orden ("ensamblar")
    Kin->>Maestro: Transición atómica a estado "ocupado"
    
    %% Paso 1
    Kin->>Kin: Paso 1: Elevar carro a cota segura (Z = 8.0 cm)
    Kin->>Serial: /cmd_joint_vel (consignas S-curve)
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Conducción de motor J3 hacia tope superior
    
    %% Paso 2
    Kin->>Serial: Paso 2: /cmd_gripper ("open,25,400")
    Serial->>ESP: G,OPEN,25,400
    ESP->>Robot: PWM 25% a GPIO 14 (Apertura completa de pinza)

    %% Paso 3
    Kin->>Kin: Paso 3: Traslación horizontal hacia coordenada de Pick
    Kin->>Serial: /cmd_joint_vel (Curva S en J1 y J2)
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Motores J1 y J2 mueven efector sobre chocolate

    %% Paso 4
    Kin->>Kin: Paso 4: Descenso vertical hacia chocolate (Z = 14.0 cm)
    Kin->>Serial: /cmd_joint_vel (Avance lineal J3)
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Descenso suave de cremallera Z

    %% Paso 5
    Kin->>Serial: Paso 5: /cmd_gripper ("close,18,350")
    Serial->>ESP: G,CLOSE,18,350
    ESP->>Robot: PWM 18% a GPIO 12 (Sujeción delicada sin aplastamiento)

    %% Paso 6
    Kin->>Kin: Paso 6: Ascenso vertical a cota segura (Z = 8.0 cm)
    Kin->>Serial: /cmd_joint_vel
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Ascenso vertical con producto asegurado

    %% Paso 7
    Kin->>Kin: Paso 7: Traslación horizontal hacia caja de empaque (Place)
    Kin->>Serial: /cmd_joint_vel
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Giro coordinado de hombro y codo hacia caja

    %% Paso 8
    Kin->>Kin: Paso 8: Descenso a cota de entrega en caja
    Kin->>Serial: /cmd_joint_vel
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Inserción suave de pieza en celda de caja

    %% Paso 9
    Kin->>Serial: Paso 9: /cmd_gripper ("open,25,400")
    Serial->>ESP: G,OPEN,25,400
    ESP->>Robot: Liberación limpia de pieza

    %% Paso 10
    Kin->>Kin: Paso 10: Retracción vertical a cota segura
    Kin->>Serial: /cmd_joint_vel
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Carro sube libre de interferencias

    %% Paso 11
    Kin->>Kin: Paso 11: Retorno a posición de reposo final (Home)
    Kin->>Serial: /cmd_joint_vel
    Serial->>ESP: V,w1,w2,w3
    ESP->>Robot: Posicionamiento en coordenadas de espera

    %% Paso 12
    Kin->>Serial: Paso 12: /cmd_gripper ("close,18,350")
    Serial->>ESP: G,CLOSE,18,350
    ESP->>Robot: Cierre de seguridad de pinza en reposo

    Note over Kin, Maestro: Ciclo completado exitosamente
    Kin->>Maestro: Publica /ensamblador2/estado ("listo")
```

---

## 3. Diagrama de Flujo de la Lógica de Control en ESP32

```mermaid
flowchart TD
    Start([Inicio / Boot ESP32]) --> InitHW[Inicializar Pines, PWM a 1000 Hz e Interrupciones de Encoders]
    InitHW --> InitTimer[Configurar Timer 0 a 100 Hz / 10 ms]
    InitHW --> InitUART[Configurar uselect.poll en sys.stdin]
    InitUART --> LoopPrincipal{¿Interrupción Timer 0?}

    LoopPrincipal -- Sí --> PasoControl[Ejecutar Paso de Control 100 Hz]
    LoopPrincipal -- No --> CheckUART[Consultar Búfer Serie]

    subgraph "Paso de Control Periódico (100 Hz)"
        PasoControl --> WDT{¿Tiempo desde último comando > 400 ms?}
        WDT -- Sí --> StopMotors[Parar Motores por Watchdog de Seguridad]
        WDT -- No --> CheckGrip{¿Motor Gripper activo y tiempo cumplido?}
        CheckGrip -- Sí --> StopGrip[Apagar PWM Gripper en GPIO 14 y 12]
        CheckGrip -- No --> CalcPID[Calcular Error de Velocidad y Feedback PI]
        CalPID --> Feedforward[Sumar Feedforward de Velocidad y Compensar Fricción Mínima]
        Feedforward --> OutputPWM[Escribir Pines IN1/IN2 y duty_u16 a PWM de Motores]
        OutputPWM --> Telemetria[Enviar Telemetría 'P,c1,c2,c3\n' cada 20 ms hacia ROS 2]
    end

    subgraph "Gestor de Comandos Serie"
        CheckUART --> HasCmd{¿Línea completa recibida?}
        HasCmd -- Sí --> CmdType{Prefijo de Comando}
        CmdType -- 'V,w1,w2,w3' --> SetVel[Actualizar ref_vel y refrescar Watchdog]
        CmdType -- 'G,accion,pwm,dur' --> SetGrip[Configurar sentido, PWM regulado y temporizador de parada]
        CmdType -- 'Z' --> SetZero[Restablecer encoder_count a cero absoluto y responder ACK]
        CmdType -- 'S' --> EmergStop[Detener inmediatamente todos los actuadores]
        HasCmd -- No --> LoopPrincipal
    end

    Telemetria --> LoopPrincipal
    SetVel --> LoopPrincipal
    SetGrip --> LoopPrincipal
    SetZero --> LoopPrincipal
    EmergStop --> LoopPrincipal
    StopMotors --> LoopPrincipal
    StopGrip --> LoopPrincipal
```

---

## 4. Diagrama Esquemático de Conexiones Eléctricas

| Componente | Pin / Señal | Conexión en ESP32 / Sistema | Función / Nivel Lógico |
| :--- | :--- | :--- | :--- |
| **L298N #1** | ENA (PWM) | **GPIO 27** | Velocidad Hombro ($J_1$) - PWM 1000 Hz |
| | IN1 | **GPIO 25** | Sentido Giro Hombro ($J_1$) |
| | IN2 | **GPIO 26** | Sentido Giro Hombro ($J_1$) |
| | ENB (PWM) | **GPIO 23** | Velocidad Codo ($J_2$) - PWM 1000 Hz |
| | IN3 | **GPIO 33** | Sentido Giro Codo ($J_2$) |
| | IN4 | **GPIO 32** | Sentido Giro Codo ($J_2$) |
| **L298N #2** | ENA (PWM) | **GPIO 13** | Velocidad Prismático ($J_3$) - PWM 1000 Hz |
| | IN1 | **GPIO 16** | Sentido Giro Prismático ($J_3$) |
| | IN2 | **GPIO 17** | Sentido Giro Prismático ($J_3$) |
| | ENB (Puente) | Puente a 5V | Habilitación permanente canal B |
| | IN3 | **GPIO 14** (PWM) | Apertura Gripper N20 (PWM regulado $\le 25\%$) |
| | IN4 | **GPIO 12** (PWM) | Cierre Gripper N20 (PWM regulado $\le 18\%$) |
| **Encoders** | M1 ENC A | **GPIO 18** (Pull-Up) | Canal A Hombro (Interrupción de hardware) |
| | M1 ENC B | **GPIO 19** (Pull-Up) | Canal B Hombro (Dirección de cuadratura) |
| | M2 ENC A | **GPIO 21** (Pull-Up) | Canal A Codo (Interrupción de hardware) |
| | M2 ENC B | **GPIO 22** (Pull-Up) | Canal B Codo (Dirección de cuadratura) |
| | M3 ENC A | **GPIO 34** (Input-Only)| Canal A Prismático (Interrupción de hardware) |
| | M3 ENC B | **GPIO 35** (Input-Only)| Canal B Prismático (Dirección de cuadratura) |
| **Alimentación** | GND | **GND común** | Masa unificada (Fuente 12V + ESP32 + L298Ns) |
| | +12V DC | Bornes $V_s$ L298N #1 y #2 | Potencia de motores |
| | +5V DC | Vin ESP32 y $V_{ss}$ L298N | Alimentación lógica y sensores Hall |
