#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# Parámetros mecánicos y de calibración de encoders
L1 = 0.205  # m
L2 = 0.13   # m
# Relación de reducción por poleas según especificaciones físicas reales:
MOTOR_GEAR_J1 = 9.0   # Poleas 1:9 en Eslabón 1 (Hombro)
MOTOR_GEAR_J2 = 3.0   # Poleas 1:3 en Eslabón 2 (Codo)
PULSOS_MOTOR_BASE = 990.0

# Conversión de ticks a radianes según relación de transmisión:
# J1: 990 * 9.0 / (2*pi) = 1418.0705 ticks/rad
# J2: 990 * 3.0 / (2*pi) = 472.6902 ticks/rad
TICKS_PER_RAD_ENC_J1 = (PULSOS_MOTOR_BASE * MOTOR_GEAR_J1) / (2.0 * math.pi)
TICKS_PER_RAD_ENC_J2 = (PULSOS_MOTOR_BASE * MOTOR_GEAR_J2) / (2.0 * math.pi)

def normalize_angle(angle):
    """Normaliza un ángulo en radianes al rango [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class ScaraSerialNode(Node):
    def __init__(self):
        super().__init__('scara_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('sign_m1', -1.0)
        self.declare_parameter('sign_m2', -1.0)
        self.declare_parameter('sign_m3', -1.0)
        # Factor de conversión ticks -> metros para piñón 20 dientes y cremallera módulo 2 (Dp=40mm, Avance=125.66mm/rev, 990 ticks/rev = ~7878 ticks/m)
        self.declare_parameter('ticks_per_meter_j3', 7878.0)
        # Altura física inicial con gripper montado: 8 cm (0.080 m) retraído hasta 17 cm (0.170 m) estirado
        self.declare_parameter('z_home_offset', 0.080)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.sign_m1 = self.get_parameter('sign_m1').get_parameter_value().double_value
        self.sign_m2 = self.get_parameter('sign_m2').get_parameter_value().double_value
        self.sign_m3 = self.get_parameter('sign_m3').get_parameter_value().double_value
        self.ticks_per_meter_j3 = self.get_parameter('ticks_per_meter_j3').get_parameter_value().double_value
        self.z_home_offset = self.get_parameter('z_home_offset').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.get_logger().info(f"Conectado a la ESP32 en {port} a {baudrate} baudios.")
        except Exception as e:
            self.get_logger().error(f"Error al abrir puerto serie {port}: {e}")

        self.enc_offset = [0.0, 0.0, 0.0]
        self.raw_enc_count = [0.0, 0.0, 0.0]

        # Publicador del estado articular
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Suscriptor de comandos de velocidad continua hacia motores
        self.sub_vel = self.create_subscription(
            Float64MultiArray,
            '/cmd_joint_vel',
            self.cmd_vel_callback,
            10
        )

        # Suscriptor para control de Gripper (String: "open", "close")
        self.sub_gripper = self.create_subscription(
            String,
            '/cmd_gripper',
            self.cmd_gripper_callback,
            10
        )

        # Suscriptor para cambiar signos de encoders dinámicamente
        self.sub_signs = self.create_subscription(
            Float64MultiArray,
            '/cmd_joint_signs',
            self.cmd_signs_callback,
            10
        )

        # Servicios
        self.srv_zero = self.create_service(Trigger, '/calibrate_zero', self.calibrate_zero_callback)

        # Timer de lectura serie (100 Hz)
        self.create_timer(0.01, self.read_serial_loop)

    def calibrate_zero_callback(self, request, response):
        self.enc_offset[0] = self.raw_enc_count[0]
        self.enc_offset[1] = self.raw_enc_count[1]
        self.enc_offset[2] = self.raw_enc_count[2]
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(b"Z\n")
            except Exception:
                pass
        response.success = True
        response.message = f"Cero calibrado en +Y. Offsets: {self.enc_offset}"
        self.get_logger().info(response.message)
        return response

    def cmd_signs_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.sign_m1 = msg.data[0]
            self.sign_m2 = msg.data[1]
            self.sign_m3 = msg.data[2]
            self.get_logger().info(f"Signos de encoder actualizados: M1={self.sign_m1}, M2={self.sign_m2}, M3={self.sign_m3}")

    def cmd_vel_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3 and hasattr(self, 'ser') and self.ser.is_open:
            w1, w2, w3 = msg.data[0], msg.data[1], msg.data[2]
            cmd = f"V,{w1:.4f},{w2:.4f},{w3:.4f}\n"
            try:
                self.ser.write(cmd.encode())
                self.ser.flush()
            except Exception as e:
                self.get_logger().error(f"Error escribiendo puerto serie: {e}")

    def cmd_gripper_callback(self, msg: String):
        texto = msg.data.strip()
        partes = texto.split(",")
        accion = partes[0].lower()
        if accion in ["open", "close", "stop"] and hasattr(self, 'ser') and self.ser.is_open:
            cmd = f"G,{texto.upper()}\n"
            try:
                self.ser.write(cmd.encode())
                self.ser.flush()
                self.get_logger().info(f"Comando Gripper enviado: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Error enviando comando a Gripper: {e}")

    def read_serial_loop(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("P,"):
                    parts = line.split(",")
                    if len(parts) >= 4:
                        try:
                            c1_raw = float(parts[1])
                            c2_raw = float(parts[2])
                            c3_raw = float(parts[3])
                            self.raw_enc_count = [c1_raw, c2_raw, c3_raw]

                            # Resta de offsets de calibración a cero
                            c1 = c1_raw - self.enc_offset[0]
                            c2 = c2_raw - self.enc_offset[1]
                            c3 = c3_raw - self.enc_offset[2]

                            # Conversión de ticks a radianes para J1 y J2
                            q1_raw = self.sign_m1 * (c1 / TICKS_PER_RAD_ENC_J1)
                            q2_raw = self.sign_m2 * (c2 / TICKS_PER_RAD_ENC_J2)

                            q1_real = normalize_angle(q1_raw)
                            q2_real = normalize_angle(q2_raw)

                            # Conversión de ticks a metros para J3 prismático (Rango real con Gripper: 0.080m a 0.170m)
                            q3_desp = self.sign_m3 * (c3 / self.ticks_per_meter_j3)
                            q3_real = max(0.080, min(0.170, self.z_home_offset + q3_desp))

                            # Publicación en /joint_states
                            joint_state = JointState()
                            joint_state.header.stamp = self.get_clock().now().to_msg()
                            joint_state.name = ['J1', 'J2', 'J3']
                            joint_state.position = [q1_real, q2_real, q3_real]
                            joint_state.effort = [c1_raw, c2_raw, c3_raw]

                            self.joint_pub.publish(joint_state)
                        except ValueError:
                            pass
        except Exception:
            pass

    def stop_motors(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(b"V,0.0,0.0,0.0\n")
                self.ser.write(b"S\n")
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ScaraSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()