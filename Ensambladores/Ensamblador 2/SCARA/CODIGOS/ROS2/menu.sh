#!/usr/bin/env bash

WORKSPACE="/home/samuel-azuero/urdf_ws"
source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42

show_menu() {
    clear
    echo "=========================================================="
    echo "       PANEL DE CONTROL ROBOT SCARA (ROS 2 JAZZY)         "
    echo "=========================================================="
    echo " 1) Iniciar Simulación en Gazebo Sim"
    echo " 2) Iniciar Control ESP32 + Cinemática + GUI (Todo en Uno)"
    echo " 3) Iniciar Sólo GUI Gráfica (Tkinter + Visualizador XY)"
    echo " 4) Consola de Control Directo de Posición (Ingresar X Y Z)"
    echo " 5) Probar Gripper Seguro N20 (Apertura 25% / Cierre 18%)"
    echo " 6) Ejecutar Secuencia Pick & Place (Objeto)"
    echo " 7) Probar Micro-pulsos Eje Z (Subir / Bajar seguro)"
    echo " 8) Probar conexión serie directa con la ESP32"
    echo " 9) Recompilar espacio de trabajo (colcon build)"
    echo " 10) Salir"
    echo "=========================================================="
    read -p "Seleccione una opción [1-10]: " opcion
}

while true; do
    show_menu
    case $opcion in
        1)
            echo "Lanzando Gazebo Sim..."
            bash "$WORKSPACE/1_lanzar_gazebo.sh"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        2)
            echo "Lanzando Control ESP32 + Cinemática + GUI..."
            bash "$WORKSPACE/2_lanzar_control_esp32.sh"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        3)
            echo "Lanzando Interfaz Gráfica..."
            bash "$WORKSPACE/3_lanzar_gui.sh"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        4)
            echo "Abriendo Consola de Control Directo..."
            python3 "$WORKSPACE/control_directo.py"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        5)
            echo "Ejecutando prueba segura de Gripper..."
            python3 "$WORKSPACE/prueba_gripper_segura.py"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        6)
            echo "Ejecutando rutina Pick & Place..."
            python3 "$WORKSPACE/prueba_pick_and_place.py"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        7)
            echo "Ejecutando prueba de micro-pulsos Z..."
            python3 "$WORKSPACE/prueba_pulsos_z.py"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        8)
            echo "Probando puerto /dev/ttyUSB0..."
            python3 -c "
import serial, time
puerto = '/dev/ttyUSB0'
baud = 115200
try:
    with serial.Serial(puerto, baud, timeout=1.0) as s:
        s.dtr = False
        s.rts = False
        print(f'✅ Conexión exitosa a {puerto} a {baud} baudios.')
        print('Escuchando datos por 3 segundos...')
        t_end = time.time() + 3.0
        recibido = False
        while time.time() < t_end:
            if s.in_waiting:
                line = s.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f'  [ESP32]: {line}')
                    recibido = True
            time.sleep(0.05)
        if not recibido:
            print('ℹ️ No se recibieron tramas en 3s (la ESP32 puede estar en reposo).')
except Exception as e:
    print(f'❌ Error abriendo {puerto}: {e}')
"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        9)
            echo "Recompilando workspace..."
            cd "$WORKSPACE" && colcon build --symlink-install
            source "$WORKSPACE/install/setup.bash"
            read -p "Presione [Enter] para volver al menú..."
            ;;
        10)
            echo "Saliendo..."
            exit 0
            ;;
        *)
            echo "Opción inválida."
            sleep 1
            ;;
    esac
done
