#!/usr/bin/env bash
# Script para iniciar la conexión serie con la ESP32 y el nodo cinemático SCARA RRP
echo "=============================================="
echo "    Iniciando Control ESP32 + Cinemática      "
echo "=============================================="

source /opt/ros/jazzy/setup.bash
source /home/samuel-azuero/urdf_ws/install/setup.bash

# Aislamiento de red para evitar interferencias de otros robots en la misma WiFi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42

# Verificar existencia del puerto
PUERTO="/dev/ttyUSB0"
if [ ! -e "$PUERTO" ]; then
    echo "⚠️ ADVERTENCIA: No se detecta $PUERTO."
    echo "Verifique que la ESP32 esté conectada o verifique con 'ls /dev/ttyUSB* /dev/ttyACM*'"
fi

ros2 launch scara_control scara.launch.py port:=$PUERTO baudrate:=115200 "$@"
