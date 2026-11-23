#!/usr/bin/env bash
# Script para iniciar el SCARA en modo Planta de Empacado (Rol: Ensamblador 2)
echo "=========================================================="
echo "   Iniciando SCARA RRP - PLANTA DE EMPACADO (ENSAMBLADOR 2)"
echo "   Contrato: /ensamblador2/orden | /ensamblador2/estado | /parada"
echo "=========================================================="

source /opt/ros/jazzy/setup.bash
source /home/samuel-azuero/urdf_ws/install/setup.bash

# Configuración de Red para comunicación Multicomputadora con el Maestro
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "📡 Dominio ROS 2: $ROS_DOMAIN_ID"
echo "🌐 Rango de Descubrimiento: $ROS_AUTOMATIC_DISCOVERY_RANGE"

# Verificar puerto serie de la ESP32
PUERTO="${1:-/dev/ttyUSB0}"
if [ ! -e "$PUERTO" ]; then
    echo "⚠️ ADVERTENCIA: No se detecta $PUERTO."
    echo "Verifique con 'ls /dev/ttyUSB* /dev/ttyACM*' o pase el puerto como argumento: ./4_lanzar_planta_ensamblador2.sh /dev/ttyUSB1"
fi

ros2 launch scara_control scara_planta.launch.py port:=$PUERTO baudrate:=115200
