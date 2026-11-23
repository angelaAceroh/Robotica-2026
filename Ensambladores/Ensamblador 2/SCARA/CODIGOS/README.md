# Códigos Fuente — Robot SCARA (Ensamblador 2)
**Universidad Militar Nueva Granada — Laboratorio de Robótica 2026**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este directorio contiene todo el código fuente ejecutable del proyecto, dividido entre el firmware embebido para el microcontrolador y los paquetes del middleware ROS 2.

---

## 1. Organización del Código

```text
CODIGOS/
├── README.md                          # Este archivo descriptivo
├── ESP32/                             # Código fuente para el microcontrolador
│   ├── README.md                      # Documentación del firmware embebido
│   └── main.py                        # Script MicroPython (Lazo PI a 100 Hz, Encoders, PWM)
└── ROS2/                              # Nodos, paquetes y scripts para ROS 2 Jazzy
    ├── README.md                      # Documentación de paquetes, tópicos y servicios
    ├── 1_lanzar_gazebo.sh             # Script de lanzamiento de simulación en Gazebo
    ├── 2_lanzar_control_esp32.sh      # Script de lanzamiento de nodo serial y cinemática
    ├── 3_lanzar_gui.sh                # Script de lanzamiento de interfaz gráfica local
    ├── 4_lanzar_planta_ensamblador2.sh # Script de lanzamiento integrado para planta
    ├── simulador_maestro.py           # Herramienta de test interactivo de contrato de planta
    ├── scara_control/                 # Paquete ROS 2: Nodos de cinemática, serial y GUI
    │   ├── package.xml
    │   ├── setup.py
    │   ├── launch/                    # scara.launch.py y scara_planta.launch.py
    │   └── scara_control/             # scara_serial_node, scara_kinematics_node, etc.
    └── ENSAMBLE/                      # Paquete ROS 2: Simulación URDF y mallas STL
        ├── package.xml
        ├── CMakeLists.txt
        ├── urdf/                      # Modelo URDF Ensamble.urdf
        ├── meshes/                    # Mallas STL de eslabones y base
        └── launch/                    # display.launch.py y gazebo.launch.py
```

---

## 2. Requisitos de Compilación y Ejecución

### 2.1. Entorno de Ejecución
- Ubuntu 24.04 LTS con ROS 2 Jazzy Jalisco instalado.
- Herramienta de compilación `colcon` (`sudo apt install python3-colcon-common-extensions`).
- Biblioteca serie de Python (`pip install pyserial`).

### 2.2. Proceso de Compilación en Workspace de ROS 2
Para compilar los paquetes en un espacio de trabajo limpio:
```bash
# 1. Crear workspace y carpeta src si no existen
mkdir -p ~/urdf_ws/src
cd ~/urdf_ws/src

# 2. Copiar o enlazar los paquetes scara_control y ENSAMBLE
# 3. Compilar los paquetes seleccionados
cd ~/urdf_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select scara_control Ensamble

# 4. Cargar el entorno compilado
source ~/urdf_ws/install/setup.bash
```

---

## 3. Instrucciones Rápidas de Ejecución

### Modo Standalone (Pruebas Locales del Robot Físico):
```bash
# Terminal 1: Lanzar control y driver serie hacia la ESP32
cd ~/urdf_ws
./2_lanzar_control_esp32.sh

# Terminal 2: Lanzar interfaz gráfica interactiva
cd ~/urdf_ws
./3_lanzar_gui.sh
```

### Modo Integrado con Planta (Línea de Empacado - Ensamblador 2):
```bash
cd ~/urdf_ws
./4_lanzar_planta_ensamblador2.sh
```

### Modo Simulación Gemelo Digital (Gazebo):
```bash
cd ~/urdf_ws
./1_lanzar_gazebo.sh
```
