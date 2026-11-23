#!/usr/bin/env bash
# Script para iniciar la interfaz gráfica Tkinter con visualizador cartesiano Matplotlib
echo "=============================================="
echo "    Iniciando Interfaz Gráfica SCARA          "
echo "=============================================="

source /opt/ros/jazzy/setup.bash
source /home/samuel-azuero/urdf_ws/install/setup.bash

# Aislamiento de red para evitar interferencias de otros robots en la misma WiFi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42

ros2 run scara_control scara_gui_node
