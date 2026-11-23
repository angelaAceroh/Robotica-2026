#!/usr/bin/env python3
import math
import time
import threading
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger

# Parámetros físicos del robot (Eslabones)
L1 = 0.205  # m
L2 = 0.13   # m
# Relaciones mecánicas de reducción
MOTOR_GEAR_J1 = 9.0    # Poleas 1:9
MOTOR_GEAR_J2 = 3.0    # Poleas 1:3
MOTOR_GEAR_J3 = 50.0   # Piñón 20 dientes y cremallera módulo 2 (Dp=40mm -> 2*pi/0.12566m = 50.0 rad/m)

# Límites físicos reales del 3er grado de libertad (Prismático J3 con Gripper montado)
Z_HOME = 0.080  # 8.0 cm (retraído con gripper)
Z_MIN = 0.080   # 8.0 cm
Z_MAX = 0.170   # 17.0 cm (tope máximo estirado con gripper)

def normalize_angle(angle):
    """ Normaliza un ángulo en radianes al rango [-pi, pi] """
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(val, min_val, max_val):
    """ Restringe un valor dentro de los límites min y max """
    return max(min_val, min(max_val, val))

class ScaraKinematicsNode(Node):
    def __init__(self):
        super().__init__('scara_kinematics_node')

        # Configuración (+1 Normal, -1 Invertido)
        self.declare_parameter('dir_m1', -1.0)
        self.declare_parameter('dir_m2', -1.0)
        self.declare_parameter('dir_m3', -1.0)
        self.declare_parameter('elbow_sign', -1.0)  # -1.0 = Codo Izquierdo (Físico real), +1.0 = Codo Derecho

        # Parámetros de velocidad de trabajo rápida (considerando 60 RPM = 6.28 rad/s máx en motor)
        self.declare_parameter('v_cart_default', 0.080)       # 8.0 cm/s (rápido y ágil)
        self.declare_parameter('max_joint_vel_rad_j1', 0.80)  # 0.80 rad/s máx en Hombro
        self.declare_parameter('max_joint_vel_rad_j2', 1.80)  # 1.80 rad/s máx en Codo
        self.declare_parameter('max_joint_vel_m_s', 0.060)    # 6.0 cm/s máx en Z

        self.s1 = self.get_parameter('dir_m1').get_parameter_value().double_value
        self.s2 = self.get_parameter('dir_m2').get_parameter_value().double_value
        self.s3 = self.get_parameter('dir_m3').get_parameter_value().double_value
        self.elbow_sign = self.get_parameter('elbow_sign').get_parameter_value().double_value
        self.v_cart_default = self.get_parameter('v_cart_default').get_parameter_value().double_value
        self.max_joint_vel_rad_j1 = self.get_parameter('max_joint_vel_rad_j1').get_parameter_value().double_value
        self.max_joint_vel_rad_j2 = self.get_parameter('max_joint_vel_rad_j2').get_parameter_value().double_value
        self.max_joint_vel_m_s = self.get_parameter('max_joint_vel_m_s').get_parameter_value().double_value

        self.q1_real = 0.0
        self.q2_real = 0.0
        self.q3_real = Z_HOME  # Posición inicial a 8 cm (0.080 m) con gripper montado

        # Suscriptor continuo a la lectura de encoders de la ESP32 (/joint_states)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Publicador de comandos de VELOCIDAD a los motores del ROBOT REAL
        self.pub_vel = self.create_publisher(Float64MultiArray, '/cmd_joint_vel', 10)

        # Publicador de POSICIÓN a GAZEBO
        self.pub_gz_pos = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Publicador para control de Gripper (Motor N20 3V)
        self.pub_gripper = self.create_publisher(String, '/cmd_gripper', 10)

        # Servicios de control
        self.srv_routine = self.create_service(Trigger, '/run_routine', self.run_routine_callback)
        self.srv_pick_and_place = self.create_service(Trigger, '/run_pick_and_place', self.run_pick_and_place_callback)
        self.srv_stop = self.create_service(Trigger, '/emergency_stop', self.emergency_stop_callback)

        # Suscriptor a objetivos cartesianos XYZ [x, y, z, T] (si T <= 0, calcula automático)
        self.create_subscription(Float64MultiArray, '/cmd_cartesian_target', self.cartesian_target_callback, 10)
        self.create_subscription(Pose2D, '/cmd_cartesian_target_pose2d', self.cartesian_target_pose2d_callback, 10)

        # Suscriptor para cambiar configuración de Codo en tiempo de ejecución ("left" / "right")
        self.create_subscription(String, '/cmd_elbow_config', self.elbow_config_callback, 10)
        self.create_subscription(Float64MultiArray, '/cmd_joint_direction', self.joint_direction_callback, 10)
        self.create_subscription(String, '/cmd_pick_and_place_custom', self.pick_and_place_custom_callback, 10)

        self.is_running = False
        self.rutina_activa = False

        # Timer de 20 Hz para sincronizar posición real con Gazebo
        self.create_timer(0.05, self.gazebo_sync_timer_callback)

        self.get_logger().info(f"Nodo Cinemático SCARA RRP listo. Rango Z: [{Z_MIN*100:.1f} a {Z_MAX*100:.1f}] cm. Codo: {'Izquierda (-1.0)' if self.elbow_sign < 0 else 'Derecha (+1.0)'}.")

    def joint_state_callback(self, msg: JointState):
        """ Se ejecuta al recibir lectura real de la ESP32 """
        if len(msg.position) >= 2:
            self.q1_real = msg.position[0]
            self.q2_real = msg.position[1]
        if len(msg.position) >= 3:
            self.q3_real = clamp(msg.position[2], Z_MIN, Z_MAX)

    def gazebo_sync_timer_callback(self):
        """ Envía las POSICIONES reales [J1, J2, J3] directamente a Gazebo (Espejo 1:1) """
        # Para Gazebo el desplazamiento relativo es (0.0 a 0.090 m)
        z_gz = clamp(self.q3_real - Z_HOME, 0.0, 0.090)
        msg_gz = Float64MultiArray()
        msg_gz.data = [self.q1_real, self.q2_real, z_gz]
        self.pub_gz_pos.publish(msg_gz)

    def elbow_config_callback(self, msg: String):
        """ Permite alternar configuración de codo dinámicamente ('left' / 'right') """
        val = msg.data.strip().lower()
        if "izq" in val or "left" in val or "-1" in val:
            self.elbow_sign = -1.0
            self.get_logger().info("Configuración de Codo actualizada: IZQUIERDA (-1.0) [Físico Real]")
        elif "der" in val or "right" in val or "+1" in val or "1" in val:
            self.elbow_sign = 1.0
            self.get_logger().info("Configuración de Codo actualizada: DERECHA (+1.0)")

    def joint_direction_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.s1 = msg.data[0]
            self.s2 = msg.data[1]
            self.s3 = msg.data[2]
            self.get_logger().info(f"Direcciones de motor actualizadas: M1={self.s1}, M2={self.s2}, M3={self.s3}")

    def fk(self, q1, q2, q3=Z_HOME):
        theta1 = q1 + (math.pi / 2.0)
        theta2 = theta1 + q2
        x = L1 * math.cos(theta1) + L2 * math.cos(theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta2)
        z = clamp(q3, Z_MIN, Z_MAX)
        return x, y, z

    def ik(self, x, y, z=Z_HOME):
        d_sq = x**2 + y**2
        max_reach_sq = (L1 + L2)**2 + 1e-5
        min_reach_sq = (abs(L1 - L2))**2 - 1e-5
        if d_sq > max_reach_sq or d_sq < min_reach_sq:
            return None, None, None

        cos_q2 = (d_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_q2 = max(-1.0, min(1.0, cos_q2))

        sin_q2 = math.sqrt(1.0 - cos_q2**2) * self.elbow_sign
        q2 = math.atan2(sin_q2, cos_q2)

        psi = math.atan2(y, x)
        beta = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        theta1 = psi - beta

        q1 = normalize_angle(theta1 - (math.pi / 2.0))
        q3 = clamp(z, Z_MIN, Z_MAX)
        return q1, q2, q3

    def compute_trajectory_time(self, x0, y0, z0, xf, yf, zf, q1_0, q2_0, q3_0, q1_f, q2_f, q3_f, v_desired=None):
        """Calcula automáticamente el tiempo de recorrido óptimo en base a la distancia euclidiana y límites articulares."""
        v_des = v_desired if (v_desired is not None and v_desired > 0.005) else self.v_cart_default

        # Distancia euclidiana 3D
        dist_cart = math.sqrt((xf - x0)**2 + (yf - y0)**2 + (zf - z0)**2)
        t_cart = dist_cart / v_des

        # Restricciones por velocidad máxima física de cada articulación
        dq1 = abs(normalize_angle(q1_f - q1_0))
        dq2 = abs(normalize_angle(q2_f - q2_0))
        dq3 = abs(zf - z0)

        t_j1 = dq1 / self.max_joint_vel_rad_j1
        t_j2 = dq2 / self.max_joint_vel_rad_j2
        t_j3 = dq3 / self.max_joint_vel_m_s

        # Tomar el tiempo limitante con un mínimo de 0.8 segundos para respuesta rápida
        T_optimo = max(t_cart, t_j1, t_j2, t_j3, 0.8)
        return T_optimo

    def cartesian_target_pose2d_callback(self, msg: Pose2D):
        """ Callback para Pose2D (si theta > 0.2 se usa como T manual; si no, automático) """
        if self.is_running:
            self.get_logger().info("Interrumpiendo movimiento anterior para nueva consigna Pose2D.")
            self.is_running = False
            time.sleep(0.06)

        x_target = msg.x
        y_target = msg.y
        z_target = self.q3_real
        T_in = msg.theta if msg.theta > 0.2 else 0.0

        q1_t, q2_t, q3_t = self.ik(x_target, y_target, z_target)
        if q1_t is None:
            self.get_logger().error(f"Punto ({x_target:.3f}, {y_target:.3f}) fuera de alcance del SCARA.")
            return

        threading.Thread(
            target=self.execute_cartesian_straight_line,
            args=(x_target, y_target, z_target, T_in),
            daemon=True
        ).start()

    def cartesian_target_callback(self, msg: Float64MultiArray):
        """ Callback para Float64MultiArray [x, y, z, (T_opcional)] """
        if len(msg.data) < 3:
            self.get_logger().error("Se requieren al menos 3 valores [X, Y, Z].")
            return

        x_target = msg.data[0]
        y_target = msg.data[1]
        z_in = msg.data[2]
        # Soporte para ingreso en cm (8.0 a 17.0 cm) o m (0.080 a 0.170 m)
        z_target = z_in / 100.0 if abs(z_in) > 0.30 else z_in
        z_target = clamp(z_target, Z_MIN, Z_MAX)
        T_in = msg.data[3] if len(msg.data) >= 4 and msg.data[3] > 0.2 else 0.0

        q1_t, q2_t, q3_t = self.ik(x_target, y_target, z_target)
        if q1_t is None:
            self.get_logger().error(f"Punto ({x_target:.3f}, {y_target:.3f}) fuera de alcance del SCARA.")
            return

        # Si ya hay un movimiento activo, detenerlo para ejecutar la nueva consigna
        if self.is_running:
            self.get_logger().info("Interrumpiendo movimiento anterior para nueva trayectoria.")
            self.is_running = False
            time.sleep(0.06)

        threading.Thread(
            target=self.execute_cartesian_straight_line,
            args=(x_target, y_target, z_target, T_in),
            daemon=True
        ).start()

    def execute_cartesian_straight_line(self, x_f, y_f, z_f, T_in=0.0):
        self.is_running = True

        x_0, y_0, z_0 = self.fk(self.q1_real, self.q2_real, self.q3_real)
        z_f = clamp(z_f, Z_MIN, Z_MAX)

        q1_0, q2_0, q3_0 = self.q1_real, self.q2_real, self.q3_real
        q1_f, q2_f, q3_f = self.ik(x_f, y_f, z_f)

        if q1_f is None:
            self.get_logger().error("Destino fuera de rango cinemático.")
            self.is_running = False
            return

        # Planificación automática de tiempo si no se especificó manualmente
        if T_in <= 0.2:
            T = self.compute_trajectory_time(x_0, y_0, z_0, x_f, y_f, z_f, q1_0, q2_0, q3_0, q1_f, q2_f, q3_f)
            self.get_logger().info(f"Trayectoria planificada: Distancia 3D = {math.sqrt((x_f-x_0)**2+(y_f-y_0)**2+(z_f-z_0)**2)*100:.1f} cm -> Tiempo calculado: {T:.2f} s")
        else:
            T = T_in
            self.get_logger().info(f"Trayectoria con tiempo manual: {T:.2f} s")

        # Ganancias del lazo de posición articular ajustadas para alta reactividad
        Kp_joint_1, Kp_joint_2, Kp_joint_3 = 9.0, 9.0, 9.0
        Ki_joint_1, Ki_joint_2, Ki_joint_3 = 0.5, 0.5, 0.5
        suma_e1, suma_e2, suma_e3 = 0.0, 0.0, 0.0
        LIMITE_INTEGRAL = 1.0

        # Compensación de fricción estática (rompe la zona muerta mecánica para llegar a tolerancia exacta)
        MIN_VEL_J1 = 0.05   # rad/s articular
        MIN_VEL_J2 = 0.06   # rad/s articular
        MIN_VEL_J3 = 0.003  # m/s lineal

        # Límite de velocidad en radianes/s del motor físico (~60 RPM = 6.28 rad/s máx)
        MAX_MOTOR_SPEED_RAD = 5.8
        TOLERANCIA_RAD = 0.005  # ~0.28° (~1.5 mm en efector)
        TOLERANCIA_M = 0.002    # 2 mm en eje prismático Z
        dt = 0.02

        steps = max(1, int(T / dt))
        start_time = time.time()

        q1_d_prev, q2_d_prev, q3_d_prev = q1_0, q2_0, q3_0

        # -------------------------------------------------------------
        # FASE 1: Bucle de seguimiento de trayectoria continua (Feedforward + PID)
        # -------------------------------------------------------------
        for i in range(1, steps + 1):
            if not self.is_running:
                break

            elapsed = time.time() - start_time
            tau = min(1.0, elapsed / T)
            # Perfil suave S-curve / Minimum jerk (coseno)
            s_val = 0.5 - 0.5 * math.cos(math.pi * tau)

            x_d = x_0 + (x_f - x_0) * s_val
            y_d = y_0 + (y_f - y_0) * s_val
            z_d = z_0 + (z_f - z_0) * s_val

            q1_d, q2_d, q3_d = self.ik(x_d, y_d, z_d)
            if q1_d is None:
                break

            # Velocidad Feedforward (derivada de la consigna)
            w1_ff = normalize_angle(q1_d - q1_d_prev) / dt
            w2_ff = normalize_angle(q2_d - q2_d_prev) / dt
            w3_ff = (q3_d - q3_d_prev) / dt

            q1_d_prev, q2_d_prev, q3_d_prev = q1_d, q2_d, q3_d

            # Error articular respecto a encoders reales
            e1 = normalize_angle(q1_d - self.q1_real)
            e2 = normalize_angle(q2_d - self.q2_real)
            e3 = q3_d - self.q3_real

            # Anti-windup condicional
            if (e1 > 0 and suma_e1 < 0) or (e1 < 0 and suma_e1 > 0): suma_e1 *= 0.5
            if (e2 > 0 and suma_e2 < 0) or (e2 < 0 and suma_e2 > 0): suma_e2 *= 0.5
            if (e3 > 0 and suma_e3 < 0) or (e3 < 0 and suma_e3 > 0): suma_e3 *= 0.5

            suma_e1 += e1 * dt
            suma_e2 += e2 * dt
            suma_e3 += e3 * dt

            suma_e1 = max(-LIMITE_INTEGRAL, min(LIMITE_INTEGRAL, suma_e1))
            suma_e2 = max(-LIMITE_INTEGRAL, min(LIMITE_INTEGRAL, suma_e2))
            suma_e3 = max(-LIMITE_INTEGRAL, min(LIMITE_INTEGRAL, suma_e3))

            w1_joint = w1_ff + (Kp_joint_1 * e1) + (Ki_joint_1 * suma_e1)
            w2_joint = w2_ff + (Kp_joint_2 * e2) + (Ki_joint_2 * suma_e2)
            w3_joint = w3_ff + (Kp_joint_3 * e3) + (Ki_joint_3 * suma_e3)

            w1_motor = self.s1 * w1_joint * MOTOR_GEAR_J1
            w2_motor = self.s2 * w2_joint * MOTOR_GEAR_J2
            w3_motor = self.s3 * w3_joint * MOTOR_GEAR_J3

            w1_motor = max(-MAX_MOTOR_SPEED_RAD, min(MAX_MOTOR_SPEED_RAD, w1_motor))
            w2_motor = max(-MAX_MOTOR_SPEED_RAD, min(MAX_MOTOR_SPEED_RAD, w2_motor))
            w3_motor = max(-MAX_MOTOR_SPEED_RAD, min(MAX_MOTOR_SPEED_RAD, w3_motor))

            msg = Float64MultiArray()
            msg.data = [w1_motor, w2_motor, w3_motor]
            self.pub_vel.publish(msg)

            time_to_sleep = (start_time + i * dt) - time.time()
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

        # -------------------------------------------------------------
        # FASE 2: Lazo de convergencia activa de alta precisión (< 1.5 mm)
        # -------------------------------------------------------------
        max_convergence_time = 2.5  # Tiempo suficiente para converger sin corte prematuro
        conv_start = time.time()
        MAX_MOTOR_SPEED_BRAKE = 3.2

        TOLERANCIA_RAD = 0.005   # ~0.28° (~1.5 mm en efector para cero deriva)
        TOLERANCIA_M = 0.002     # 2 mm en eje Z

        done1, done2, done3 = False, False, False

        # Piso de velocidad mínimo para vencer fricción estática del reductor
        MIN_M1_SPEED = 1.4  # rad/s en motor J1
        MIN_M2_SPEED = 1.2  # rad/s en motor J2
        MIN_M3_SPEED = 1.6  # rad/s en motor J3

        while self.is_running and (time.time() - conv_start < max_convergence_time):
            e1 = normalize_angle(q1_f - self.q1_real)
            e2 = normalize_angle(q2_f - self.q2_real)
            e3 = q3_f - self.q3_real

            # Histéresis estrecha: si entra en tolerancia estricta, fijar completado
            if not done1 and abs(e1) <= TOLERANCIA_RAD:
                done1 = True
            elif done1 and abs(e1) > (TOLERANCIA_RAD * 1.5):
                done1 = False

            if not done2 and abs(e2) <= TOLERANCIA_RAD:
                done2 = True
            elif done2 and abs(e2) > (TOLERANCIA_RAD * 1.5):
                done2 = False

            if not done3 and abs(e3) <= TOLERANCIA_M:
                done3 = True
            elif done3 and abs(e3) > (TOLERANCIA_M * 1.5):
                done3 = False

            # Si todos los ejes están dentro de tolerancia milimétrica, salir inmediatamente
            if done1 and done2 and done3:
                break

            # Cálculo de velocidad de motor con reducción suave al aproximarse
            if done1:
                w1_motor = 0.0
            else:
                w1_prop = abs(e1) * 8.0 * MOTOR_GEAR_J1
                min_1 = MIN_M1_SPEED if abs(e1) > 0.015 else (MIN_M1_SPEED * 0.75)
                w1_mag = max(min_1, min(MAX_MOTOR_SPEED_BRAKE, w1_prop))
                w1_motor = self.s1 * math.copysign(w1_mag, e1)

            if done2:
                w2_motor = 0.0
            else:
                w2_prop = abs(e2) * 8.0 * MOTOR_GEAR_J2
                min_2 = MIN_M2_SPEED if abs(e2) > 0.015 else (MIN_M2_SPEED * 0.75)
                w2_mag = max(min_2, min(MAX_MOTOR_SPEED_BRAKE, w2_prop))
                w2_motor = self.s2 * math.copysign(w2_mag, e2)

            if done3:
                w3_motor = 0.0
            else:
                w3_prop = abs(e3) * 10.0 * MOTOR_GEAR_J3
                w3_mag = max(MIN_M3_SPEED, min(3.0, w3_prop))
                w3_motor = self.s3 * math.copysign(w3_mag, e3)

            msg = Float64MultiArray()
            msg.data = [w1_motor, w2_motor, w3_motor]
            self.pub_vel.publish(msg)

            time.sleep(dt)

        # Freno total al llegar
        msg_stop = Float64MultiArray()
        msg_stop.data = [0.0, 0.0, 0.0]
        self.pub_vel.publish(msg_stop)

        x_r, y_r, z_r = self.fk(self.q1_real, self.q2_real, self.q3_real)
        err_cart = math.sqrt((x_f - x_r)**2 + (y_f - y_r)**2 + (z_f - z_r)**2)
        self.get_logger().info(f"Llegada al destino. Posición real final: ({x_r:.3f}, {y_r:.3f}, {z_r:.3f}) m | Error cartesiano: {err_cart*1000.0:.2f} mm")

        self.is_running = False

    def run_routine_callback(self, request, response):
        if self.rutina_activa:
            response.success = False
            response.message = "Ya hay una rutina en ejecución."
            return response

        threading.Thread(target=self.ejecutar_rutina_secuencial, daemon=True).start()
        response.success = True
        response.message = "Rutina iniciada con tiempos automáticos."
        return response

    def run_pick_and_place_callback(self, request, response):
        if self.rutina_activa:
            response.success = False
            response.message = "Ya hay una rutina o pick and place en ejecución."
            return response

        threading.Thread(target=self.ejecutar_pick_and_place, daemon=True).start()
        response.success = True
        response.message = "Secuencia Pick & Place iniciada con valores estándar."
        return response

    def pick_and_place_custom_callback(self, msg: String):
        """ Recibe parámetros personalizados en formato JSON y ejecuta la secuencia """
        if self.rutina_activa:
            self.get_logger().warn("Ya hay una rutina o pick and place en ejecución. Solicitud ignorada.")
            return

        try:
            cfg = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error parseando JSON de Pick & Place: {e}")
            return

        # Extraer valores numéricos con valores por defecto seguros
        try:
            x_pick = float(cfg.get('x_pick', 0.100))
            y_pick = float(cfg.get('y_pick', 0.200))
            z_pick = float(cfg.get('z_pick', 0.140))

            x_place = float(cfg.get('x_place', -0.100))
            y_place = float(cfg.get('y_place', 0.200))
            z_place = float(cfg.get('z_place', 0.140))

            x_final = float(cfg.get('x_final', 0.000))
            y_final = float(cfg.get('y_final', 0.250))
            z_safe = float(cfg.get('z_safe', Z_HOME))

            pwm_open = int(float(cfg.get('pwm_open', 25)))
            t_open_raw = float(cfg.get('dur_open_ms', 400))
            dur_open_ms = int(t_open_raw if t_open_raw >= 10.0 else t_open_raw * 1000.0)

            pwm_close = int(float(cfg.get('pwm_close', 18)))
            t_close_raw = float(cfg.get('dur_close_ms', 350))
            dur_close_ms = int(t_close_raw if t_close_raw >= 10.0 else t_close_raw * 1000.0)
        except (ValueError, TypeError) as err:
            self.get_logger().error(f"Valores numéricos no válidos en JSON de Pick & Place: {err}")
            return

        # Normalizar Z si se ingresó en cm (> 0.30)
        if abs(z_pick) > 0.30: z_pick /= 100.0
        if abs(z_place) > 0.30: z_place /= 100.0
        if abs(z_safe) > 0.30: z_safe /= 100.0

        # Limitar estrictamente dentro de los límites mecánicos del robot con gripper
        z_pick = clamp(z_pick, Z_MIN, Z_MAX)
        z_place = clamp(z_place, Z_MIN, Z_MAX)
        z_safe = clamp(z_safe, Z_MIN, Z_MAX)

        # Rango amplio y flexible: PWM 10-35% y duración 50ms a 4000ms (0.05s a 4.0s)
        pwm_open = max(10, min(35, pwm_open))
        pwm_close = max(10, min(35, pwm_close))
        dur_open_ms = max(50, min(4000, dur_open_ms))
        dur_close_ms = max(50, min(4000, dur_close_ms))

        self.get_logger().info(f"Parámetros Pick & Place recibidos:")
        self.get_logger().info(f" - Pick: ({x_pick:.3f}, {y_pick:.3f}, {z_pick:.3f}m)")
        self.get_logger().info(f" - Place: ({x_place:.3f}, {y_place:.3f}, {z_place:.3f}m)")
        self.get_logger().info(f" - Reposo Final: ({x_final:.3f}, {y_final:.3f}, {z_safe:.3f}m)")
        self.get_logger().info(f" - Gripper Abrir: {pwm_open}% PWM, {dur_open_ms}ms | Cerrar: {pwm_close}% PWM, {dur_close_ms}ms")

        threading.Thread(
            target=self.ejecutar_pick_and_place,
            kwargs={
                'x_pick': x_pick, 'y_pick': y_pick, 'z_pick': z_pick,
                'x_place': x_place, 'y_place': y_place, 'z_place': z_place,
                'x_final': x_final, 'y_final': y_final, 'z_safe': z_safe,
                'pwm_open': pwm_open, 'dur_open_ms': dur_open_ms,
                'pwm_close': pwm_close, 'dur_close_ms': dur_close_ms
            },
            daemon=True
        ).start()

    def ejecutar_rutina_secuencial(self):
        self.rutina_activa = True
        # Puntos de rutina XYZ calibrados al rango físico con gripper [8 a 17 cm]
        puntos_rutina = [
            (0.100, 0.250, 0.090, 0.0),
            (0.100, 0.180, 0.130, 0.0),
            (-0.100, 0.180, 0.090, 0.0),
            (-0.100, 0.250, 0.130, 0.0),
            (0.000, 0.250, 0.080, 0.0)
        ]
        for i, (x_dest, y_dest, z_dest, T_manual) in enumerate(puntos_rutina):
            if not self.rutina_activa:
                break
            self.get_logger().info(f"Rutina - Punto {i+1}: X={x_dest}, Y={y_dest}, Z={z_dest}")
            self.execute_cartesian_straight_line(x_dest, y_dest, z_dest, T_manual)

            if not self.rutina_activa:
                break
            time.sleep(0.8)

        self.rutina_activa = False
        self.get_logger().info("Rutina secuencial finalizada exitosamente.")

    def ejecutar_pick_and_place(self, x_pick=0.100, y_pick=0.200, z_pick=0.140,
                                x_place=-0.100, y_place=0.200, z_place=0.140,
                                x_final=0.000, y_final=0.250, z_safe=Z_HOME,
                                pwm_open=25, dur_open_ms=400,
                                pwm_close=18, dur_close_ms=350):
        """
        Secuencia completa Pick and Place 3D configurable:
        1. Subir a altura de tránsito seguro (z_safe)
        2. Abrir gripper (pwm_open, dur_open_ms)
        3. Mover horizontalmente sobre el objeto (x_pick, y_pick, z_safe)
        4. Descender al objeto (x_pick, y_pick, z_pick)
        5. Cerrar gripper para agarrar (pwm_close, dur_close_ms)
        6. Subir con el objeto a z_safe
        7. Mover horizontalmente sobre destino (x_place, y_place, z_safe)
        8. Descender al punto de entrega (x_place, y_place, z_place)
        9. Abrir gripper para soltar objeto (pwm_open, dur_open_ms)
        10. Subir a z_safe
        11. Regresar a posición final configurada (x_final, y_final, z_safe)
        12. Cerrar gripper en posición final (pwm_close, dur_close_ms)
        """
        self.rutina_activa = True

        self.get_logger().info("=== INICIANDO SECUENCIA PICK AND PLACE ===")
        self.get_logger().info(f"Param: Pick=({x_pick:.3f}, {y_pick:.3f}, {z_pick:.3f}) | Place=({x_place:.3f}, {y_place:.3f}, {z_place:.3f}) | Final=({x_final:.3f}, {y_final:.3f}, {z_safe:.3f})")

        pasos = [
            # (descripción, acción_tipo, params)
            ("1. Elevar a altura de tránsito seguro", "MOVE", (self.fk(self.q1_real, self.q2_real)[0], self.fk(self.q1_real, self.q2_real)[1], z_safe)),
            ("2. Abrir gripper", "GRIPPER", ("open", pwm_open, dur_open_ms)),
            ("3. Posicionarse sobre el objeto (Pick)", "MOVE", (x_pick, y_pick, z_safe)),
            ("4. Descender hacia el objeto", "MOVE", (x_pick, y_pick, z_pick)),
            ("5. Agarrar objeto (Cerrar gripper suave)", "GRIPPER", ("close", pwm_close, dur_close_ms)),
            ("6. Levantar objeto a altura segura", "MOVE", (x_pick, y_pick, z_safe)),
            ("7. Trasladar objeto hacia destino (Place)", "MOVE", (x_place, y_place, z_safe)),
            ("8. Descender en posición de entrega", "MOVE", (x_place, y_place, z_place)),
            ("9. Soltar objeto (Abrir gripper)", "GRIPPER", ("open", pwm_open, dur_open_ms)),
            ("10. Levantar herramienta a altura segura", "MOVE", (x_place, y_place, z_safe)),
            ("11. Retorno a posición de reposo final", "MOVE", (x_final, y_final, z_safe)),
            ("12. Cerrar gripper en reposo final", "GRIPPER", ("close", pwm_close, dur_close_ms)),
        ]

        for desc, tipo, params in pasos:
            if not self.rutina_activa:
                self.get_logger().warn("Pick & Place interrumpido por el usuario.")
                break

            self.get_logger().info(f">> {desc}...")

            if tipo == "MOVE":
                x_t, y_t, z_t = params
                self.execute_cartesian_straight_line(x_t, y_t, z_t, 0.0)
                time.sleep(0.4)

            elif tipo == "GRIPPER":
                accion, pwm, dur = params
                msg_grip = String()
                msg_grip.data = f"{accion},{pwm},{dur}"
                self.pub_gripper.publish(msg_grip)
                # Esperar a que el motor N20 complete el movimiento mecánico
                time.sleep((dur / 1000.0) + 0.3)

        self.rutina_activa = False
        self.get_logger().info("=== SECUENCIA PICK AND PLACE FINALIZADA EXITOSAMENTE ===")

    def emergency_stop_callback(self, request, response):
        self.is_running = False
        self.rutina_activa = False
        msg_stop = Float64MultiArray()
        msg_stop.data = [0.0, 0.0, 0.0]
        self.pub_vel.publish(msg_stop)

        response.success = True
        response.message = "Paro de emergencia ejecutado."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ScaraKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()