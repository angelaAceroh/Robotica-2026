#!/usr/bin/env python3
"""
control_directo.py — Consola Interactiva de Control de Posición Directo para Robot SCARA RRP
Permite ingresar coordenadas cartesianas XYZ, controlar el Gripper N20 y monitorear la posición real en tiempo real.
Sin intermediación de nodo maestro.
"""
import sys
import time
import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

L1 = 0.205  # m
L2 = 0.130  # m
Z_MIN = 0.080  # 8.0 cm
Z_MAX = 0.170  # 17.0 cm

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

class ControlDirectoNode(Node):
    def __init__(self):
        super().__init__('scara_control_directo_cli')
        
        self.q1_real = 0.0
        self.q2_real = 0.0
        self.q3_real = 0.080
        self.have_joint_state = False
        
        # Suscriptor a estado de encoders
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Publicadores
        self.pub_cartesian = self.create_publisher(Float64MultiArray, '/cmd_cartesian_target', 10)
        self.pub_gripper = self.create_publisher(String, '/cmd_gripper', 10)
        self.pub_elbow = self.create_publisher(String, '/cmd_elbow_config', 10)
        
        # Clientes de servicio
        self.cli_zero = self.create_client(Trigger, '/calibrate_zero')
        self.cli_stop = self.create_client(Trigger, '/emergency_stop')
        self.cli_pnp = self.create_client(Trigger, '/run_pick_and_place')
        
    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 2:
            self.q1_real = msg.position[0]
            self.q2_real = msg.position[1]
            self.have_joint_state = True
        if len(msg.position) >= 3:
            self.q3_real = msg.position[2]
            
    def get_fk(self):
        theta1 = self.q1_real + (math.pi / 2.0)
        theta2 = theta1 + self.q2_real
        x = L1 * math.cos(theta1) + L2 * math.cos(theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta2)
        z = self.q3_real
        return x, y, z

    def check_reach(self, x, y):
        d_sq = x**2 + y**2
        min_r = abs(L1 - L2)
        max_r = L1 + L2
        if d_sq > (max_r**2 + 1e-4) or d_sq < (min_r**2 - 1e-4):
            return False, math.sqrt(d_sq), min_r, max_r
        return True, math.sqrt(d_sq), min_r, max_r

    def enviar_posicion(self, x, y, z, t=0.0):
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(z), float(t)]
        self.pub_cartesian.publish(msg)

    def enviar_gripper(self, accion, pwm=None, t_ms=None):
        accion = accion.lower()
        if pwm is not None and t_ms is not None:
            cmd = f"{accion},{pwm},{t_ms}"
        else:
            cmd = accion
        msg = String()
        msg.data = cmd
        self.pub_gripper.publish(msg)

    def calibrar_cero(self):
        if not self.cli_zero.wait_for_service(timeout_sec=2.0):
            return False, "Servicio /calibrate_zero no disponible."
        req = Trigger.Request()
        fut = self.cli_zero.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is not None:
            return fut.result().success, fut.result().message
        return False, "Timeout esperando respuesta."

    def paro_emergencia(self):
        if not self.cli_stop.wait_for_service(timeout_sec=1.0):
            return False, "Servicio /emergency_stop no disponible."
        req = Trigger.Request()
        fut = self.cli_stop.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        if fut.result() is not None:
            return fut.result().success, fut.result().message
        return False, "Timeout esperando respuesta."

    def ejecutar_pnp(self):
        if not self.cli_pnp.wait_for_service(timeout_sec=2.0):
            return False, "Servicio /run_pick_and_place no disponible."
        req = Trigger.Request()
        fut = self.cli_pnp.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is not None:
            return fut.result().success, fut.result().message
        return False, "Timeout esperando respuesta."

def main():
    rclpy.init()
    node = ControlDirectoNode()
    
    # Hilo secundario para que ROS 2 procese callbacks en background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*65)
    print("      CONSOLA DE CONTROL DIRECTO DE POSICIÓN - SCARA RRP     ")
    print("="*65)
    print("Esperando telemetría del robot (/joint_states)...")
    
    for _ in range(30):
        if node.have_joint_state:
            break
        time.sleep(0.1)
        
    if not node.have_joint_state:
        print("⚠️ Advertencia: No se han recibido lecturas de /joint_states.")
        print("Asegúrese de que el control esté activo: 'bash 2_lanzar_control_esp32.sh'")
    else:
        x_act, y_act, z_act = node.get_fk()
        print(f"✅ Conectado al SCARA. Posición actual:")
        print(f"   X={x_act:+.3f} m, Y={y_act:+.3f} m, Z={z_act*100.0:.1f} cm ({z_act:.3f} m)")
        print(f"   Q1={node.q1_real:+.3f} rad, Q2={node.q2_real:+.3f} rad, Q3={node.q3_real:+.3f} m")

    print("\n-------------------------------------------------------------")
    print("COMANDOS DISPONIBLES:")
    print("  • Mover a XYZ:          x y z [tiempo]")
    print("      Ejemplos:           0.10 0.25 10       (X=10cm, Y=25cm, Z=10cm, T auto)")
    print("                          -0.10 0.20 14 3.0  (X=-10cm, Y=20cm, Z=14cm, T=3s)")
    print("                          0 25 8             (Posición Reposo inicial)")
    print("  • Gripper Abrir:        abrir  o  a        (25% PWM, ~2.5V seguro)")
    print("  • Gripper Cerrar:       cerrar o  c        (18% PWM, ~1.8V seguro)")
    print("  • Calibrar Cero (+Y):   cero   o  z")
    print("  • Rutina Pick & Place:  pnp")
    print("  • Paro de Emergencia:   stop   o  s")
    print("  • Ver Posición Actual:  pos    o  p")
    print("  • Salir:                salir  o  q")
    print("-------------------------------------------------------------")
    
    while rclpy.ok():
        try:
            x_act, y_act, z_act = node.get_fk()
            prompt = f"\n[SCARA ({x_act:+.2f}, {y_act:+.2f}, Z={z_act*100.0:.1f}cm)] > "
            entrada = input(prompt).strip()
            
            if not entrada:
                continue
                
            cmd = entrada.lower()
            
            if cmd in ['salir', 'q', 'exit']:
                print("Cerrando consola de control directo...")
                break
                
            elif cmd in ['pos', 'p']:
                x_act, y_act, z_act = node.get_fk()
                print(f"📍 Posición Cartesiana Real: X = {x_act:+.4f} m, Y = {y_act:+.4f} m, Z = {z_act*100.0:.2f} cm ({z_act:.4f} m)")
                print(f"⚙️ Articular: Q1 = {node.q1_real:+.4f} rad ({math.degrees(node.q1_real):.1f}°), Q2 = {node.q2_real:+.4f} rad ({math.degrees(node.q2_real):.1f}°)")
                continue
                
            elif cmd in ['abrir', 'a', 'open']:
                print("🖐️ Abriendo Gripper (PWM 25%, 0.4s)...")
                node.enviar_gripper("open")
                continue
                
            elif cmd in ['cerrar', 'c', 'close']:
                print("✊ Cerrando Gripper suave (PWM 18%, 0.35s)...")
                node.enviar_gripper("close")
                continue
                
            elif cmd in ['cero', 'zero', 'z']:
                print("📐 Calibrando posición Cero (+Y)...")
                ok, msg = node.calibrar_cero()
                if ok:
                    print(f"✔ {msg}")
                else:
                    print(f"❌ Error al calibrar cero: {msg}")
                continue
                
            elif cmd in ['stop', 's', 'alto', 'paro']:
                print("🛑 ¡PARO DE EMERGENCIA ENVIADO!")
                node.paro_emergencia()
                continue
                
            elif cmd in ['pnp', 'pick']:
                print("📦 Solicitando ejecución de Pick & Place...")
                ok, msg = node.ejecutar_pnp()
                if ok:
                    print(f"✔ {msg}")
                else:
                    print(f"⚠️ {msg}")
                continue
                
            # Procesamiento de coordenadas XYZ [T]
            partes = entrada.replace(',', ' ').split()
            if len(partes) >= 3:
                try:
                    val_x = float(partes[0])
                    val_y = float(partes[1])
                    val_z = float(partes[2])
                    val_t = float(partes[3]) if len(partes) >= 4 else 0.0
                    
                    # Soporte para ingreso en cm (si val_x o val_y > 1.0, o val_z > 1.0)
                    if abs(val_x) > 1.0 or abs(val_y) > 1.0:
                        val_x /= 100.0
                        val_y /= 100.0
                        print(f"ℹ️ Coordenadas XY convertidas a metros: X={val_x:.3f} m, Y={val_y:.3f} m")
                        
                    if abs(val_z) > 1.0:
                        val_z /= 100.0
                        print(f"ℹ️ Coordenada Z convertida a metros: Z={val_z:.3f} m ({val_z*100.0:.1f} cm)")
                        
                    # Validar límites de Z [0.080 m a 0.170 m]
                    if val_z < Z_MIN or val_z > Z_MAX:
                        z_ant = val_z
                        val_z = clamp(val_z, Z_MIN, Z_MAX)
                        print(f"⚠️ Z={z_ant*100.0:.1f} cm fuera de rango físico [8 a 17 cm]. Clampeado a {val_z*100.0:.1f} cm.")
                        
                    # Validar alcance en el plano horizontal
                    valido, dist, r_min, r_max = node.check_reach(val_x, val_y)
                    if not valido:
                        print(f"❌ Error cinemático: El punto ({val_x:.3f}, {val_y:.3f}) m está a {dist*100:.1f} cm.")
                        print(f"   Rango útil SCARA: [{r_min*100:.1f} a {r_max*100:.1f}] cm.")
                        continue
                        
                    print(f"🚀 Enviando consigna: X={val_x:+.3f} m, Y={val_y:+.3f} m, Z={val_z*100.0:.1f} cm (T={'Auto' if val_t <= 0 else f'{val_t:.1f}s'})...")
                    node.enviar_posicion(val_x, val_y, val_z, val_t)
                    
                    # Esperar inicio y convergencia
                    time.sleep(0.3)
                    t_inicio = time.time()
                    t_max_espera = (val_t + 5.0) if val_t > 0 else 6.0
                    
                    print("   Robot en movimiento...", end="", flush=True)
                    while (time.time() - t_inicio) < t_max_espera:
                        time.sleep(0.3)
                        x_r, y_r, z_r = node.get_fk()
                        err = math.sqrt((val_x - x_r)**2 + (val_y - y_r)**2 + (val_z - z_r)**2)
                        print(".", end="", flush=True)
                        if err < 0.003:  # Menor a 3 mm
                            break
                            
                    x_fin, y_fin, z_fin = node.get_fk()
                    err_fin = math.sqrt((val_x - x_fin)**2 + (val_y - y_fin)**2 + (val_z - z_fin)**2)
                    print(f"\n✔ Llegada confirmada en {time.time()-t_inicio:.1f}s.")
                    print(f"   Posición final obtenida: ({x_fin:+.3f}, {y_fin:+.3f}, Z={z_fin*100.0:.1f}cm)")
                    print(f"   Error residual cartesiano: {err_fin*1000.0:.2f} mm")
                    
                except ValueError:
                    print("❌ Formato numérico inválido. Use números (ej: 0.10 0.25 10).")
            else:
                print(f"❓ Comando no reconocido: '{entrada}'. Ingrese 'x y z' o 'ayuda'.")
                
        except (KeyboardInterrupt, EOFError):
            print("\nSaliendo...")
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
