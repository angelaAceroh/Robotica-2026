#!/usr/bin/env python3
import math
import time
import threading
import json
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger

# PARÁMETROS FÍSICOS ROBOT SCARA
L1 = 0.205  # m
L2 = 0.13   # m
MOTOR_GEAR_J1 = 9.0    # Reducción 1:9 (Hombro)
MOTOR_GEAR_J2 = 3.0    # Reducción 1:3 (Codo)
MOTOR_GEAR_J3 = 50.0   # Piñón 20 dientes y cremallera módulo 2 (Dp=40mm -> 50.0 rad/m)

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class ScaraGuiPlantaNode(Node):
    def __init__(self):
        super().__init__('scara_gui_planta_node')

        # Variables de estado
        self.q1_real = 0.0
        self.q2_real = 0.0
        self.q3_real = 0.080  # 8 cm inicial con Gripper
        self.target_x = 0.100
        self.target_y = 0.250
        self.target_z = 0.080

        self.ui_text_q1 = "Q1 Hombro: 0.000 rad (0.0°)"
        self.ui_text_q2 = "Q2 Codo: 0.000 rad (0.0°)"
        self.ui_text_q3 = "Q3 Z Prismático: 8.00 cm (0.0800 m)"
        self.ui_text_xy = "Posición XYZ Real: (0.000, 0.335, 0.080) m"
        self.ui_text_error = "Error al Objetivo: (---, ---, ---) mm"

        self.x_desired_hist = []
        self.y_desired_hist = []
        self.x_real_hist = []
        self.y_real_hist = []

        self.is_running_joint_ctrl = False

        # ROS 2 Interfaces
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub_cartesian_xyz = self.create_publisher(Float64MultiArray, '/cmd_cartesian_target', 10)
        self.pub_cartesian_pose2d = self.create_publisher(Pose2D, '/cmd_cartesian_target_pose2d', 10)
        self.pub_joint_vel = self.create_publisher(Float64MultiArray, '/cmd_joint_vel', 10)
        self.pub_gripper = self.create_publisher(String, '/cmd_gripper', 10)
        self.pub_elbow = self.create_publisher(String, '/cmd_elbow_config', 10)
        self.pub_joint_signs = self.create_publisher(Float64MultiArray, '/cmd_joint_signs', 10)
        self.pub_joint_direction = self.create_publisher(Float64MultiArray, '/cmd_joint_direction', 10)
        self.pub_pick_place_custom = self.create_publisher(String, '/cmd_pick_and_place_custom', 10)

        # Interfaces del Contrato de Planta (Ensamblador 2)
        self.create_subscription(String, '/ensamblador2/estado', self.planta_estado_callback, 10)
        self.create_subscription(String, '/ensamblador2/orden', self.planta_orden_callback, 10)
        self.create_subscription(Bool, '/parada', self.planta_parada_callback, 10)

        self.pub_planta_orden = self.create_publisher(String, '/ensamblador2/orden', 10)
        self.pub_planta_parada = self.create_publisher(Bool, '/parada', 10)
        self.cli_reset_estado = self.create_client(Trigger, '/ensamblador2/reset_estado')

        self.cli_zero = self.create_client(Trigger, '/calibrate_zero')
        self.cli_routine = self.create_client(Trigger, '/run_routine')
        self.cli_pick_place = self.create_client(Trigger, '/run_pick_and_place')
        self.cli_stop = self.create_client(Trigger, '/emergency_stop')

        # Construir Interfaz Tkinter
        self.root = tk.Tk()
        self.root.title("SCARA RRP - Ensamblador 2 (Línea de Empacado) & Monitoreo")
        self.root.geometry("1420x940")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self._build_ui()

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 2:
            self.q1_real = msg.position[0]
            self.q2_real = msg.position[1]

            x_r, y_r = self.fk(self.q1_real, self.q2_real)
            self.x_real_hist.append(x_r)
            self.y_real_hist.append(y_r)

            deg1 = math.degrees(self.q1_real)
            deg2 = math.degrees(self.q2_real)
            self.ui_text_q1 = f"Q1 Hombro: {self.q1_real:.3f} rad ({deg1:.1f}°)"
            self.ui_text_q2 = f"Q2 Codo: {self.q2_real:.3f} rad ({deg2:.1f}°)"

        if len(msg.position) >= 3:
            self.q3_real = msg.position[2]
            self.ui_text_q3 = f"Q3 Z Prismático: {self.q3_real*100.0:.2f} cm ({self.q3_real*1000.0:.1f} mm / {self.q3_real:.4f} m)"

        x_r, y_r = self.fk(self.q1_real, self.q2_real)
        self.ui_text_xy = f"Posición XYZ Real: ({x_r:.3f}, {y_r:.3f}, {self.q3_real:.3f}) m [Z={self.q3_real*100.0:.1f} cm]"

        # Cálculo de error residual respecto al último objetivo
        err_x = (self.target_x - x_r) * 1000.0
        err_y = (self.target_y - y_r) * 1000.0
        err_z = (self.target_z - self.q3_real) * 1000.0
        err_total = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        self.ui_text_error = f"Error al Objetivo: ex={err_x:+.1f}, ey={err_y:+.1f}, ez={err_z:+.1f} mm | Total={err_total:.1f} mm"

    def fk(self, q1, q2):
        theta1 = q1 + (math.pi / 2.0)
        theta2 = theta1 + q2
        x = L1 * math.cos(theta1) + L2 * math.cos(theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta2)
        return x, y

    def _build_ui(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Panel Izquierdo con Scroll
        canvas_ctrl = tk.Canvas(main_frame, width=540)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas_ctrl.yview)
        scrollable_frame = ttk.Frame(canvas_ctrl)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas_ctrl.configure(scrollregion=canvas_ctrl.bbox("all"))
        )
        canvas_ctrl.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas_ctrl.configure(yscrollcommand=scrollbar.set)

        canvas_ctrl.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=5, pady=5)
        scrollbar.pack(side=tk.LEFT, fill=tk.Y)

        ctrl = ttk.LabelFrame(scrollable_frame, text=" Panel de Control SCARA - ROS 2 ")
        ctrl.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # =====================================================================
        # 0. PANEL DE COMUNICACIÓN CON LA PLANTA (CONTRATO ENSAMBLADOR 2)
        # =====================================================================
        f_planta = ttk.LabelFrame(ctrl, text=" 🏭 LÍNEA DE EMPACADO (ENSAMBLADOR 2) - MAESTRO ")
        f_planta.pack(fill=tk.X, pady=4, padx=2)

        f_st = ttk.Frame(f_planta)
        f_st.pack(fill=tk.X, padx=4, pady=3)
        ttk.Label(f_st, text="Estado Robot:", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        self.lbl_planta_estado = tk.Label(
            f_st, text="🟢 LISTO", font=('Helvetica', 10, 'bold'),
            bg="#2e7d32", fg="white", padx=10, pady=2, relief=tk.RAISED
        )
        self.lbl_planta_estado.pack(side=tk.LEFT, padx=10)

        self.lbl_planta_parada = tk.Label(
            f_st, text="Línea: OK", font=('Helvetica', 8, 'bold'),
            bg="#37474f", fg="white", padx=6, pady=2
        )
        self.lbl_planta_parada.pack(side=tk.RIGHT, padx=4)

        self.lbl_planta_orden = ttk.Label(
            f_planta, text="Última orden: (Esperando órdenes del maestro...)",
            font=('Helvetica', 8, 'italic')
        )
        self.lbl_planta_orden.pack(anchor=tk.W, padx=6, pady=2)

        f_planta_btns = ttk.Frame(f_planta)
        f_planta_btns.pack(fill=tk.X, padx=4, pady=4)

        ttk.Button(f_planta_btns, text="🔄 Reset a LISTO", command=self.btn_reset_estado_clicked).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f_planta_btns, text="🧪 Simular 'ensamblar'", command=self.btn_simular_orden_clicked).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f_planta_btns, text="🚨 Parada Línea", command=self.btn_simular_parada_clicked).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 1. Calibración Cero
        f_cal = ttk.Frame(ctrl); f_cal.pack(fill=tk.X, pady=4)
        ttk.Button(f_cal, text="🎯 Calibrar Cero (Brazo Recto a +Y)", command=self.btn_zero_clicked).pack(fill=tk.X, padx=4)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 2. Configuración de Codo (Izquierda / Derecha)
        f_elbow = ttk.LabelFrame(ctrl, text=" 🦾 CONFIGURACIÓN DE CODO SCARA ")
        f_elbow.pack(fill=tk.X, pady=4)
        self.var_elbow = tk.StringVar(value="left")
        ttk.Radiobutton(f_elbow, text="👈 Codo Izquierda (Físico Real, -1.0)", variable=self.var_elbow, value="left", command=self.on_elbow_changed).pack(anchor=tk.W, padx=6, pady=2)
        ttk.Radiobutton(f_elbow, text="👉 Codo Derecha (+1.0)", variable=self.var_elbow, value="right", command=self.on_elbow_changed).pack(anchor=tk.W, padx=6, pady=2)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 2b. Convención y Dirección de Motores
        f_inv = ttk.LabelFrame(ctrl, text=" 🔄 CONVENCIÓN Y DIRECCIÓN DE ARTICULACIONES ")
        f_inv.pack(fill=tk.X, pady=4)
        f_inv_sub = ttk.Frame(f_inv)
        f_inv_sub.pack(fill=tk.X, padx=4, pady=2)
        self.var_inv_j1 = tk.BooleanVar(value=True)  # True = -1.0 (Invertido Físico)
        self.var_inv_j2 = tk.BooleanVar(value=True)  # True = -1.0 (Invertido Físico)
        self.var_inv_j3 = tk.BooleanVar(value=True)  # True = -1.0 (Invertido Físico)
        ttk.Checkbutton(f_inv_sub, text="Invertir J1 (-1)", variable=self.var_inv_j1, command=self.on_signs_changed).pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(f_inv_sub, text="Invertir J2 (-1)", variable=self.var_inv_j2, command=self.on_signs_changed).pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(f_inv_sub, text="Invertir J3 (-1)", variable=self.var_inv_j3, command=self.on_signs_changed).pack(side=tk.LEFT, padx=3)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 3. Trayectoria Cartesiana con Tiempo Automático
        c_frame = ttk.LabelFrame(ctrl, text=" 📍 TRAYECTORIA CARTESIANA XYZ (TIEMPO AUTOMÁTICO) ")
        c_frame.pack(fill=tk.X, pady=4)

        f_xy = ttk.Frame(c_frame); f_xy.pack(fill=tk.X, pady=2)
        ttk.Label(f_xy, text="X Destino (m):").grid(row=0, column=0, sticky=tk.W)
        self.ent_x = ttk.Entry(f_xy, width=7); self.ent_x.insert(0, "0.100"); self.ent_x.grid(row=0, column=1, padx=2)
        ttk.Label(f_xy, text="Y Destino (m):").grid(row=0, column=2, sticky=tk.W)
        self.ent_y = ttk.Entry(f_xy, width=7); self.ent_y.insert(0, "0.250"); self.ent_y.grid(row=0, column=3, padx=2)

        f_z = ttk.Frame(c_frame); f_z.pack(fill=tk.X, pady=2)
        ttk.Label(f_z, text="Z Destino [8 a 17 cm]:").grid(row=0, column=0, sticky=tk.W)
        self.ent_z = ttk.Entry(f_z, width=7); self.ent_z.insert(0, "8.0"); self.ent_z.grid(row=0, column=1, padx=2)
        ttk.Label(f_z, text="Tiempo T (0=Auto):").grid(row=0, column=2, sticky=tk.W)
        self.ent_t_cart = ttk.Entry(f_z, width=7); self.ent_t_cart.insert(0, "0.0"); self.ent_t_cart.grid(row=0, column=3, padx=2)

        ttk.Label(c_frame, text="💡 Rango real con Gripper: 8.0 cm (retraído) a 17.0 cm (estirado). Ingrese en cm (ej. 12) o m (ej. 0.12).", font=('Helvetica', 8, 'italic')).pack(anchor=tk.W, padx=4, pady=1)
        ttk.Label(c_frame, text="💡 Deje Tiempo en 0.0 para cálculo óptimo por distancia", font=('Helvetica', 8, 'italic')).pack(anchor=tk.W, padx=4, pady=1)
        ttk.Button(c_frame, text="🚀 Ejecutar Trayectoria Recta XYZ", command=self.btn_cartesian_clicked).pack(fill=tk.X, pady=6, padx=4)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 4. Control por Junturas Independientes
        ij_frame = ttk.LabelFrame(ctrl, text=" 🎛️ CONTROL INDEPENDIENTE POR JUNTURA ")
        ij_frame.pack(fill=tk.X, pady=4)

        ttk.Label(ij_frame, text="J1 Hombro (rad/s, s):", font=('Helvetica', 8, 'bold')).pack(anchor=tk.W)
        f_j1 = ttk.Frame(ij_frame); f_j1.pack(fill=tk.X, pady=1)
        self.ent_w1 = ttk.Entry(f_j1, width=7); self.ent_w1.insert(0, "0.25"); self.ent_w1.grid(row=0, column=0, padx=2)
        self.ent_t1 = ttk.Entry(f_j1, width=7); self.ent_t1.insert(0, "2.0"); self.ent_t1.grid(row=0, column=1, padx=2)

        ttk.Label(ij_frame, text="J2 Codo (rad/s, s):", font=('Helvetica', 8, 'bold')).pack(anchor=tk.W)
        f_j2 = ttk.Frame(ij_frame); f_j2.pack(fill=tk.X, pady=1)
        self.ent_w2 = ttk.Entry(f_j2, width=7); self.ent_w2.insert(0, "0.25"); self.ent_w2.grid(row=0, column=0, padx=2)
        self.ent_t2 = ttk.Entry(f_j2, width=7); self.ent_t2.insert(0, "2.0"); self.ent_t2.grid(row=0, column=1, padx=2)

        ttk.Label(ij_frame, text="J3 Prismático (m/s, s):", font=('Helvetica', 8, 'bold')).pack(anchor=tk.W)
        f_j3 = ttk.Frame(ij_frame); f_j3.pack(fill=tk.X, pady=1)
        self.ent_w3 = ttk.Entry(f_j3, width=7); self.ent_w3.insert(0, "0.02"); self.ent_w3.grid(row=0, column=0, padx=2)
        self.ent_t3 = ttk.Entry(f_j3, width=7); self.ent_t3.insert(0, "2.0"); self.ent_t3.grid(row=0, column=1, padx=2)

        ttk.Button(ij_frame, text="⚙️ Ejecutar Control por Juntura", command=self.btn_joint_control_clicked).pack(fill=tk.X, pady=4, padx=4)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 5. Control de Gripper (Motor 4 N20)
        grip_frame = ttk.LabelFrame(ctrl, text=" 🖐️ CONTROL DE GRIPPER (MOTOR 4 N20) ")
        grip_frame.pack(fill=tk.X, pady=4)

        # Fila Apertura (25% probado óptimo)
        f_open = ttk.Frame(grip_frame); f_open.pack(fill=tk.X, pady=2)
        ttk.Label(f_open, text="Abrir PWM (%):", width=14).grid(row=0, column=0, sticky=tk.W, padx=2)
        self.ent_grip_pwm_open = ttk.Entry(f_open, width=5); self.ent_grip_pwm_open.insert(0, "25"); self.ent_grip_pwm_open.grid(row=0, column=1, padx=2)
        ttk.Label(f_open, text="T (s):").grid(row=0, column=2, sticky=tk.W, padx=2)
        self.ent_grip_time_open = ttk.Entry(f_open, width=5); self.ent_grip_time_open.insert(0, "0.4"); self.ent_grip_time_open.grid(row=0, column=3, padx=2)
        ttk.Button(f_open, text="🖐️ Abrir", command=lambda: self.enviar_gripper_custom("open")).grid(row=0, column=4, padx=4)

        # Fila Cierre Suave (18% para no rodar soporte ni sobreforzar)
        f_close = ttk.Frame(grip_frame); f_close.pack(fill=tk.X, pady=2)
        ttk.Label(f_close, text="Cerrar PWM (%):", width=14).grid(row=0, column=0, sticky=tk.W, padx=2)
        self.ent_grip_pwm_close = ttk.Entry(f_close, width=5); self.ent_grip_pwm_close.insert(0, "18"); self.ent_grip_pwm_close.grid(row=0, column=1, padx=2)
        ttk.Label(f_close, text="T (s):").grid(row=0, column=2, sticky=tk.W, padx=2)
        self.ent_grip_time_close = ttk.Entry(f_close, width=5); self.ent_grip_time_close.insert(0, "0.35"); self.ent_grip_time_close.grid(row=0, column=3, padx=2)
        ttk.Button(f_close, text="✊ Cerrar (Suave)", command=lambda: self.enviar_gripper_custom("close")).grid(row=0, column=4, padx=4)

        self.var_inv_gripper = tk.BooleanVar(value=False)
        ttk.Checkbutton(grip_frame, text="🔄 Invertir Sentido Abrir/Cerrar (si cables quedaron invertidos)", variable=self.var_inv_gripper).pack(anchor=tk.W, padx=4, pady=2)
        ttk.Label(grip_frame, text="💡 Seguro N20: Abrir a 25% (0.4s) | Cerrar suave a 18% (0.35s) para no rodar el motor", font=('Helvetica', 8, 'italic')).pack(anchor=tk.W, padx=4, pady=1)
        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 6. Configurador de Secuencia Pick & Place
        pnp_frame = ttk.LabelFrame(ctrl, text=" 📦 CONFIGURADOR DE SECUENCIA PICK & PLACE ")
        pnp_frame.pack(fill=tk.X, pady=4)

        # 6a. Punto de Recogida (Pick)
        f_pick_lbl = ttk.Frame(pnp_frame); f_pick_lbl.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_pick_lbl, text="1️⃣ Recogida (PICK)", font=('Helvetica', 8, 'bold'), foreground='#0055aa').pack(side=tk.LEFT)
        ttk.Label(f_pick_lbl, text="[Z: 8 a 17 cm]", font=('Helvetica', 7, 'italic')).pack(side=tk.RIGHT)

        f_pick = ttk.Frame(pnp_frame); f_pick.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_pick, text="X (m):").grid(row=0, column=0, sticky=tk.W)
        self.ent_pnp_x_pick = ttk.Entry(f_pick, width=6); self.ent_pnp_x_pick.insert(0, "0.100"); self.ent_pnp_x_pick.grid(row=0, column=1, padx=2)
        ttk.Label(f_pick, text="Y (m):").grid(row=0, column=2, sticky=tk.W)
        self.ent_pnp_y_pick = ttk.Entry(f_pick, width=6); self.ent_pnp_y_pick.insert(0, "0.200"); self.ent_pnp_y_pick.grid(row=0, column=3, padx=2)
        ttk.Label(f_pick, text="Z (cm):").grid(row=0, column=4, sticky=tk.W)
        self.ent_pnp_z_pick = ttk.Entry(f_pick, width=6); self.ent_pnp_z_pick.insert(0, "14.0"); self.ent_pnp_z_pick.grid(row=0, column=5, padx=2)

        # 6b. Punto de Entrega (Place)
        f_place_lbl = ttk.Frame(pnp_frame); f_place_lbl.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_place_lbl, text="2️⃣ Entrega (PLACE)", font=('Helvetica', 8, 'bold'), foreground='#aa5500').pack(side=tk.LEFT)
        ttk.Label(f_place_lbl, text="[Z: 8 a 17 cm]", font=('Helvetica', 7, 'italic')).pack(side=tk.RIGHT)

        f_place = ttk.Frame(pnp_frame); f_place.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_place, text="X (m):").grid(row=0, column=0, sticky=tk.W)
        self.ent_pnp_x_place = ttk.Entry(f_place, width=6); self.ent_pnp_x_place.insert(0, "-0.100"); self.ent_pnp_x_place.grid(row=0, column=1, padx=2)
        ttk.Label(f_place, text="Y (m):").grid(row=0, column=2, sticky=tk.W)
        self.ent_pnp_y_place = ttk.Entry(f_place, width=6); self.ent_pnp_y_place.insert(0, "0.200"); self.ent_pnp_y_place.grid(row=0, column=3, padx=2)
        ttk.Label(f_place, text="Z (cm):").grid(row=0, column=4, sticky=tk.W)
        self.ent_pnp_z_place = ttk.Entry(f_place, width=6); self.ent_pnp_z_place.insert(0, "14.0"); self.ent_pnp_z_place.grid(row=0, column=5, padx=2)

        # 6c. Posición Final de Reposo y Tránsito Seguro
        ttk.Label(pnp_frame, text="3️⃣ Reposo Final y Tránsito Seguro", font=('Helvetica', 8, 'bold'), foreground='#007700').pack(anchor=tk.W, padx=4, pady=1)
        f_final = ttk.Frame(pnp_frame); f_final.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_final, text="X Fin:").grid(row=0, column=0, sticky=tk.W)
        self.ent_pnp_x_final = ttk.Entry(f_final, width=6); self.ent_pnp_x_final.insert(0, "0.000"); self.ent_pnp_x_final.grid(row=0, column=1, padx=2)
        ttk.Label(f_final, text="Y Fin:").grid(row=0, column=2, sticky=tk.W)
        self.ent_pnp_y_final = ttk.Entry(f_final, width=6); self.ent_pnp_y_final.insert(0, "0.250"); self.ent_pnp_y_final.grid(row=0, column=3, padx=2)
        ttk.Label(f_final, text="Z Trán:").grid(row=0, column=4, sticky=tk.W)
        self.ent_pnp_z_safe = ttk.Entry(f_final, width=6); self.ent_pnp_z_safe.insert(0, "8.0"); self.ent_pnp_z_safe.grid(row=0, column=5, padx=2)

        # 6d. Parámetros de Gripper N20
        ttk.Label(pnp_frame, text="4️⃣ Ajuste de Gripper N20 (Apertura y Cierre)", font=('Helvetica', 8, 'bold')).pack(anchor=tk.W, padx=4, pady=1)
        f_grip_pnp = ttk.Frame(pnp_frame); f_grip_pnp.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_grip_pnp, text="Abrir PWM%:").grid(row=0, column=0, sticky=tk.W)
        self.ent_pnp_pwm_open = ttk.Entry(f_grip_pnp, width=5); self.ent_pnp_pwm_open.insert(0, "25"); self.ent_pnp_pwm_open.grid(row=0, column=1, padx=2)
        ttk.Label(f_grip_pnp, text="T.Abrir(s):").grid(row=0, column=2, sticky=tk.W)
        self.ent_pnp_time_open = ttk.Entry(f_grip_pnp, width=5); self.ent_pnp_time_open.insert(0, "0.40"); self.ent_pnp_time_open.grid(row=0, column=3, padx=2)

        f_grip_pnp2 = ttk.Frame(pnp_frame); f_grip_pnp2.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(f_grip_pnp2, text="Cerrar PWM%:").grid(row=0, column=0, sticky=tk.W)
        self.ent_pnp_pwm_close = ttk.Entry(f_grip_pnp2, width=5); self.ent_pnp_pwm_close.insert(0, "18"); self.ent_pnp_pwm_close.grid(row=0, column=1, padx=2)
        ttk.Label(f_grip_pnp2, text="T.Cerrar(s):").grid(row=0, column=2, sticky=tk.W)
        self.ent_pnp_time_close = ttk.Entry(f_grip_pnp2, width=5); self.ent_pnp_time_close.insert(0, "0.35"); self.ent_pnp_time_close.grid(row=0, column=3, padx=2)

        # Nota de recomendación técnica
        ttk.Label(pnp_frame, text="💡 Recomendado: Abrir 25% (0.40s) | Cerrar 18% (0.35s). Tiempos >0.5s empujan objetos.", font=('Helvetica', 7, 'italic'), foreground='#775500').pack(anchor=tk.W, padx=4, pady=1)

        # Botones de Acción P&P
        f_pnp_btn = ttk.Frame(pnp_frame); f_pnp_btn.pack(fill=tk.X, pady=4, padx=4)
        ttk.Button(f_pnp_btn, text="💾 Guardar Config Planta", command=self.btn_guardar_pnp_planta).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(f_pnp_btn, text="🚀 Probar P&P Ahora", command=self.btn_ejecutar_pick_place_custom).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(f_pnp_btn, text="🔄 Defaults", command=self.btn_restaurar_defaults_pnp).pack(side=tk.RIGHT, padx=2)

        ttk.Separator(ctrl, orient='horizontal').pack(fill='x', pady=6)

        # 7. Rutinas Secuenciales y Paro de Emergencia
        r_frame = ttk.LabelFrame(ctrl, text=" 📋 OTRAS RUTINAS Y SEGURIDAD ")
        r_frame.pack(fill=tk.X, pady=4)
        ttk.Button(r_frame, text="📦 Ejecutar Pick & Place Estándar (1-Click)", command=self.btn_pick_place_clicked).pack(fill=tk.X, pady=2, padx=4)
        ttk.Button(r_frame, text="▶️ Rutina de Puntos Automáticos", command=self.btn_routine_clicked).pack(fill=tk.X, pady=2, padx=4)
        ttk.Button(r_frame, text="🛑 PARO DE EMERGENCIA", command=self.btn_stop_clicked).pack(fill=tk.X, pady=4, padx=4)

        # 6. Monitor de Estado Real y Precisión
        enc_frame = ttk.LabelFrame(ctrl, text=" Estado Actual y Monitoreo de Precisión ")
        enc_frame.pack(fill=tk.X, pady=4)
        self.lbl_q1 = ttk.Label(enc_frame, text=self.ui_text_q1); self.lbl_q1.pack(anchor=tk.W, padx=5)
        self.lbl_q2 = ttk.Label(enc_frame, text=self.ui_text_q2); self.lbl_q2.pack(anchor=tk.W, padx=5)
        self.lbl_q3 = ttk.Label(enc_frame, text=self.ui_text_q3); self.lbl_q3.pack(anchor=tk.W, padx=5)
        self.lbl_xy = ttk.Label(enc_frame, text=self.ui_text_xy, font=('Helvetica', 8, 'bold')); self.lbl_xy.pack(anchor=tk.W, padx=5)
        self.lbl_err = ttk.Label(enc_frame, text=self.ui_text_error, foreground='blue'); self.lbl_err.pack(anchor=tk.W, padx=5)

        # Gráfica Matplotlib
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.fig, self.ax = plt.subplots(figsize=(7.0, 7.0))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_plot()

    def on_elbow_changed(self):
        val = self.var_elbow.get()
        msg = String()
        msg.data = val
        self.pub_elbow.publish(msg)
        self.get_logger().info(f"Configuración de codo seleccionada en GUI: {val}")

    def on_signs_changed(self):
        s1 = -1.0 if self.var_inv_j1.get() else 1.0
        s2 = -1.0 if self.var_inv_j2.get() else 1.0
        s3 = -1.0 if self.var_inv_j3.get() else 1.0
        msg = Float64MultiArray()
        msg.data = [s1, s2, s3]
        self.pub_joint_signs.publish(msg)
        self.pub_joint_direction.publish(msg)
        self.get_logger().info(f"Convención de articulaciones actualizada: J1={s1}, J2={s2}, J3={s3}")

    def enviar_gripper(self, accion):
        msg = String()
        msg.data = accion
        self.pub_gripper.publish(msg)
        self.get_logger().info(f"Gripper comandado: {accion}")

    def enviar_gripper_custom(self, accion):
        if self.var_inv_gripper.get():
            accion_efectiva = "close" if accion.lower() == "open" else "open"
        else:
            accion_efectiva = accion.lower()

        if accion.lower() == "open":
            try:
                pwm = int(float(self.ent_grip_pwm_open.get()))
                tiempo_ms = int(float(self.ent_grip_time_open.get()) * 1000)
            except ValueError:
                pwm = 25
                tiempo_ms = 400
        else:
            try:
                pwm = int(float(self.ent_grip_pwm_close.get()))
                tiempo_ms = int(float(self.ent_grip_time_close.get()) * 1000)
            except ValueError:
                pwm = 20
                tiempo_ms = 400
        pwm = max(0, min(30, pwm))
        tiempo_ms = max(100, tiempo_ms)
        cmd_str = f"{accion_efectiva},{pwm},{tiempo_ms}"
        msg = String()
        msg.data = cmd_str
        self.pub_gripper.publish(msg)
        self.get_logger().info(f"Gripper comandado: {cmd_str} (Botón: {accion}, Invertido: {self.var_inv_gripper.get()})")

    def btn_zero_clicked(self):
        if self.cli_zero.wait_for_service(timeout_sec=1.0):
            self.cli_zero.call_async(Trigger.Request())
            self.x_real_hist.clear(); self.y_real_hist.clear()
            self.x_desired_hist.clear(); self.y_desired_hist.clear()
            messagebox.showinfo("Cero Calibrado", "Comando de cero enviado a ROS 2 y a la ESP32.")
        else:
            messagebox.showwarning("Aviso", "El servicio /calibrate_zero no respondió (compruebe scara_serial_node).")

    def btn_cartesian_clicked(self):
        try:
            x_target = float(self.ent_x.get())
            y_target = float(self.ent_y.get())
            z_raw = float(self.ent_z.get())
            # Si el usuario ingresa en centímetros (ej. 5.0, 10.0, 15.0), convertir a metros
            if abs(z_raw) > 0.30:
                z_target = z_raw / 100.0
            else:
                z_target = z_raw
            # Limitar estrictamente al rango real con Gripper: 8 a 17 cm (0.080 a 0.170 m)
            z_target = max(0.080, min(0.170, z_target))
            T_val = float(self.ent_t_cart.get())
        except ValueError:
            messagebox.showerror("Error", "Ingrese valores numéricos válidos en X, Y, Z y T.")
            return

        self.target_x = x_target
        self.target_y = y_target
        self.target_z = z_target

        x0, y0 = self.fk(self.q1_real, self.q2_real)
        self.x_desired_hist = [x0, x_target]
        self.y_desired_hist = [y0, y_target]

        # Publicar en formato [x, y, z, T] (si T es 0, cinemática lo calcula automático)
        msg_xyz = Float64MultiArray()
        msg_xyz.data = [x_target, y_target, z_target, T_val]
        self.pub_cartesian_xyz.publish(msg_xyz)
        self.get_logger().info(f"Trayectoria Cartesiana solicitada: X={x_target:.3f}, Y={y_target:.3f}, Z={z_target:.3f} m ({z_target*100.0:.1f} cm), T={T_val:.1f}s")

    def btn_joint_control_clicked(self):
        try:
            w1 = float(self.ent_w1.get()); t1 = float(self.ent_t1.get())
            w2 = float(self.ent_w2.get()); t2 = float(self.ent_t2.get())
            w3 = float(self.ent_w3.get()); t3 = float(self.ent_t3.get())
        except ValueError:
            messagebox.showerror("Error", "Valores numéricos inválidos en junturas.")
            return

        threading.Thread(target=self.execute_independent_joints, args=(w1, t1, w2, t2, w3, t3), daemon=True).start()

    def execute_independent_joints(self, w1, t1, w2, t2, w3, t3):
        self.is_running_joint_ctrl = True
        s1 = -1.0 if self.var_inv_j1.get() else 1.0
        s2 = -1.0 if self.var_inv_j2.get() else 1.0
        s3 = -1.0 if self.var_inv_j3.get() else 1.0

        w1_motor = s1 * w1 * MOTOR_GEAR_J1
        w2_motor = s2 * w2 * MOTOR_GEAR_J2
        # Si w3 está en m/s (ej: 0.02 m/s), escalar con MOTOR_GEAR_J3; si se ingresó directo en rad/s (>0.2), usar directo
        w3_motor = s3 * (w3 * MOTOR_GEAR_J3 if (abs(w3) < 0.2 and w3 != 0.0) else w3)
        t_max = max(t1, t2, t3)

        start_time = time.time()
        dt = 0.04

        while self.is_running_joint_ctrl:
            elapsed = time.time() - start_time
            if elapsed >= t_max:
                break

            cmd_w1 = w1_motor if elapsed <= t1 else 0.0
            cmd_w2 = w2_motor if elapsed <= t2 else 0.0
            cmd_w3 = w3_motor if elapsed <= t3 else 0.0

            msg = Float64MultiArray()
            msg.data = [cmd_w1, cmd_w2, cmd_w3]
            self.pub_joint_vel.publish(msg)
            time.sleep(dt)

        msg_stop = Float64MultiArray()
        msg_stop.data = [0.0, 0.0, 0.0]
        self.pub_joint_vel.publish(msg_stop)
        self.is_running_joint_ctrl = False

    def btn_routine_clicked(self):
        if self.cli_routine.wait_for_service(timeout_sec=1.0):
            self.cli_routine.call_async(Trigger.Request())
        else:
            messagebox.showwarning("Aviso", "El servicio /run_routine no está disponible.")

    # =========================================================================
    # MÉTODOS DE LA PLANTA (ENSAMBLADOR 2)
    # =========================================================================

    def planta_estado_callback(self, msg: String):
        estado = msg.data.strip().lower()
        if estado == "listo":
            self.lbl_planta_estado.config(text="🟢 LISTO", bg="#2e7d32", fg="white")
        elif estado == "ocupado":
            self.lbl_planta_estado.config(text="🟡 OCUPADO", bg="#f57f17", fg="black")
        elif estado == "error":
            self.lbl_planta_estado.config(text="🔴 ERROR / PARADA", bg="#c62828", fg="white")
        else:
            self.lbl_planta_estado.config(text=f"⚪ {estado.upper()}", bg="#546e7a", fg="white")

    def planta_orden_callback(self, msg: String):
        hora = time.strftime("%H:%M:%S")
        self.lbl_planta_orden.config(text=f"Última orden recibida: '{msg.data}' [{hora}]")

    def planta_parada_callback(self, msg: Bool):
        if msg.data:
            self.lbl_planta_parada.config(text="🔴 PARADA ACTIVA", bg="#b71c1c", fg="white")
        else:
            self.lbl_planta_parada.config(text="Línea: OK", bg="#37474f", fg="white")

    def btn_reset_estado_clicked(self):
        if not self.cli_reset_estado.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "El servicio /ensamblador2/reset_estado no está disponible.\nVerifique que scara_kinematics_planta_node esté en ejecución.")
            return
        req = Trigger.Request()
        future = self.cli_reset_estado.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info("Reset de estado a 'listo' solicitado."))

    def btn_simular_orden_clicked(self):
        msg = String()
        msg.data = "ensamblar"
        self.pub_planta_orden.publish(msg)
        self.get_logger().info("Orden simulada enviada a /ensamblador2/orden: 'ensamblar'")

    def btn_simular_parada_clicked(self):
        msg = Bool()
        msg.data = True
        self.pub_planta_parada.publish(msg)
        self.get_logger().warn("Señal de parada enviada a /parada: True")

    def _extraer_pnp_payload(self, solo_guardar=False):
        try:
            x_pick = float(self.ent_pnp_x_pick.get())
            y_pick = float(self.ent_pnp_y_pick.get())
            z_pick_raw = float(self.ent_pnp_z_pick.get())
            z_pick = z_pick_raw / 100.0 if abs(z_pick_raw) > 0.30 else z_pick_raw

            x_place = float(self.ent_pnp_x_place.get())
            y_place = float(self.ent_pnp_y_place.get())
            z_place_raw = float(self.ent_pnp_z_place.get())
            z_place = z_place_raw / 100.0 if abs(z_place_raw) > 0.30 else z_place_raw

            x_final = float(self.ent_pnp_x_final.get())
            y_final = float(self.ent_pnp_y_final.get())
            z_safe_raw = float(self.ent_pnp_z_safe.get())
            z_safe = z_safe_raw / 100.0 if abs(z_safe_raw) > 0.30 else z_safe_raw

            pwm_open = int(float(self.ent_pnp_pwm_open.get()))
            t_open_raw = float(self.ent_pnp_time_open.get())
            dur_open_ms = int(t_open_raw if t_open_raw >= 10.0 else t_open_raw * 1000.0)

            pwm_close = int(float(self.ent_pnp_pwm_close.get()))
            t_close_raw = float(self.ent_pnp_time_close.get())
            dur_close_ms = int(t_close_raw if t_close_raw >= 10.0 else t_close_raw * 1000.0)
        except ValueError:
            messagebox.showerror("Error", "Ingrese valores numéricos válidos en todos los campos de Pick & Place.")
            return None

        # Validaciones de alcance físico (L1 + L2 = 0.335 m)
        r_pick = math.hypot(x_pick, y_pick)
        r_place = math.hypot(x_place, y_place)
        r_final = math.hypot(x_final, y_final)
        r_max = L1 + L2

        if r_pick > r_max or r_place > r_max or r_final > r_max:
            messagebox.showwarning("Fuera de Alcance", f"Uno de los puntos supera el alcance radial máximo del SCARA ({r_max*100:.1f} cm).")
            return None

        # Limitar Z al rango físico real con gripper: 8 a 17 cm (0.080 a 0.170 m)
        z_pick = max(0.080, min(0.170, z_pick))
        z_place = max(0.080, min(0.170, z_place))
        z_safe = max(0.080, min(0.170, z_safe))

        # Rango amplio y flexible: PWM 10-35% y duración 50ms a 4000ms
        pwm_open = max(10, min(35, pwm_open))
        pwm_close = max(10, min(35, pwm_close))
        dur_open_ms = max(50, min(4000, dur_open_ms))
        dur_close_ms = max(50, min(4000, dur_close_ms))

        return {
            "x_pick": x_pick, "y_pick": y_pick, "z_pick": z_pick,
            "x_place": x_place, "y_place": y_place, "z_place": z_place,
            "x_final": x_final, "y_final": y_final, "z_safe": z_safe,
            "pwm_open": pwm_open, "dur_open_ms": dur_open_ms,
            "pwm_close": pwm_close, "dur_close_ms": dur_close_ms,
            "solo_guardar": solo_guardar
        }

    def btn_guardar_pnp_planta(self):
        payload = self._extraer_pnp_payload(solo_guardar=True)
        if payload is None:
            return
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_pick_place_custom.publish(msg)
        messagebox.showinfo("Configuración Guardada", "Los parámetros de Pick & Place se han guardado como la configuración activa para el ensamble del robot en la planta.")

    def btn_ejecutar_pick_place_custom(self):
        payload = self._extraer_pnp_payload(solo_guardar=False)
        if payload is None:
            return
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_pick_place_custom.publish(msg)
        self.get_logger().info(f"Comandando Pick & Place Personalizado: {msg.data}")

    def btn_restaurar_defaults_pnp(self):
        self.ent_pnp_x_pick.delete(0, tk.END); self.ent_pnp_x_pick.insert(0, "0.100")
        self.ent_pnp_y_pick.delete(0, tk.END); self.ent_pnp_y_pick.insert(0, "0.200")
        self.ent_pnp_z_pick.delete(0, tk.END); self.ent_pnp_z_pick.insert(0, "14.0")

        self.ent_pnp_x_place.delete(0, tk.END); self.ent_pnp_x_place.insert(0, "-0.100")
        self.ent_pnp_y_place.delete(0, tk.END); self.ent_pnp_y_place.insert(0, "0.200")
        self.ent_pnp_z_place.delete(0, tk.END); self.ent_pnp_z_place.insert(0, "14.0")

        self.ent_pnp_x_final.delete(0, tk.END); self.ent_pnp_x_final.insert(0, "0.000")
        self.ent_pnp_y_final.delete(0, tk.END); self.ent_pnp_y_final.insert(0, "0.250")
        self.ent_pnp_z_safe.delete(0, tk.END); self.ent_pnp_z_safe.insert(0, "8.0")

        self.ent_pnp_pwm_open.delete(0, tk.END); self.ent_pnp_pwm_open.insert(0, "25")
        self.ent_pnp_time_open.delete(0, tk.END); self.ent_pnp_time_open.insert(0, "0.40")

        self.ent_pnp_pwm_close.delete(0, tk.END); self.ent_pnp_pwm_close.insert(0, "18")
        self.ent_pnp_time_close.delete(0, tk.END); self.ent_pnp_time_close.insert(0, "0.35")

    def btn_pick_place_clicked(self):
        if self.cli_pick_place.wait_for_service(timeout_sec=1.0):
            self.cli_pick_place.call_async(Trigger.Request())
        else:
            messagebox.showwarning("Aviso", "El servicio /run_pick_and_place no está disponible (inicie 2_lanzar_control_esp32.sh).")

    def btn_stop_clicked(self):
        self.is_running_joint_ctrl = False
        if self.cli_stop.wait_for_service(timeout_sec=1.0):
            self.cli_stop.call_async(Trigger.Request())
        else:
            msg_stop = Float64MultiArray()
            msg_stop.data = [0.0, 0.0, 0.0]
            self.pub_joint_vel.publish(msg_stop)

    def update_plot(self):
        self.lbl_q1.config(text=self.ui_text_q1)
        self.lbl_q2.config(text=self.ui_text_q2)
        self.lbl_q3.config(text=self.ui_text_q3)
        self.lbl_xy.config(text=self.ui_text_xy)
        self.lbl_err.config(text=self.ui_text_error)

        self.ax.clear()
        r_max = L1 + L2; r_min = abs(L1 - L2)
        self.ax.add_patch(plt.Circle((0, 0), r_max, color='lightgray', fill=False, linestyle='--'))
        self.ax.add_patch(plt.Circle((0, 0), r_min, color='lightgray', fill=False, linestyle='--'))

        self.ax.plot([0, 0], [0, r_max], 'k:', alpha=0.4, label="Referencia 0 rad (+Y)")

        if self.x_desired_hist:
            self.ax.plot(self.x_desired_hist, self.y_desired_hist, 'r--', linewidth=2, label="Línea Recta Deseada")
            self.ax.plot([self.target_x], [self.target_y], 'r*', markersize=10, label="Meta XYZ")

        if self.x_real_hist:
            self.ax.plot(self.x_real_hist[-300:], self.y_real_hist[-300:], 'g-', linewidth=2, label="Trayectoria Real Medida")

        q1_abs = self.q1_real + (math.pi / 2.0)
        x1 = L1 * math.cos(q1_abs); y1 = L1 * math.sin(q1_abs)
        x2, y2 = self.fk(self.q1_real, self.q2_real)

        self.ax.plot([0, x1, x2], [0, y1, y2], 'bo-', linewidth=3, markersize=6, label="SCARA Real")
        codo_side = "Izq" if x1 < x2 else "Der"
        self.ax.plot([x1], [y1], 'mo', markersize=8, label=f"Codo {codo_side} ({x1:.2f}, {y1:.2f})")
        self.ax.plot([x2], [y2], 'ro', markersize=8, label=f"Efector ({x2:.2f}, {y2:.2f})")

        self.ax.set_xlim(-0.35, 0.35); self.ax.set_ylim(-0.35, 0.35)
        self.ax.set_aspect('equal')
        self.ax.set_title("SCARA ROS 2 - Monitoreo de Trayectoria y Precisión")
        self.ax.grid(True)
        self.ax.legend(loc="upper right", fontsize='small')

        self.canvas.draw()
        self.root.after(100, self.update_plot)

    def on_closing(self):
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    gui_node = ScaraGuiPlantaNode()

    def spin_worker():
        try:
            rclpy.spin(gui_node)
        except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
            pass

    t = threading.Thread(target=spin_worker, daemon=True)
    t.start()

    try:
        gui_node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            gui_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()