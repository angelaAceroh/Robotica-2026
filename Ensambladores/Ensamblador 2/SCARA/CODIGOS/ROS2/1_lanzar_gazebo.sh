#!/usr/bin/env bash
# Script para iniciar la simulación en Gazebo Sim y los controladores de posición
echo "=============================================="
echo "    Iniciando Simulación SCARA en Gazebo      "
echo "=============================================="

source /opt/ros/jazzy/setup.bash
source /home/samuel-azuero/urdf_ws/install/setup.bash

# Aislamiento de red para evitar interferencias de otros robots en la misma WiFi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42

# Opcional: backend gráfico por compatibilidad
export QT_QPA_PLATFORM=xcb
export GZ_RENDERING_ENGINE_BACKEND=ogre

ros2 launch Ensamble gazebo.launch.py
