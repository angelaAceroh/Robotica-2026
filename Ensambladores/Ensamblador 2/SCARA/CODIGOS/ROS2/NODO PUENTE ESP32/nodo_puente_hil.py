#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import threading
import math
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ==========================================
# GENERADORES DE TRAYECTORIAS (V, T)
# ==========================================
class GeneradorTrapezoidalVT:
    def __init__(self, q0, vmax, t_total):
        self.q0 = q0
        self.vmax = abs(vmax)
        self.t_total = max(t_total, 0.1)
        self.dir = 1.0 if vmax >= 0 else -1.0
        
        self.ta = self.t_total / 3.0
        self.tc = self.t_total / 3.0
        self.tf = self.t_total
        
        self.amax = self.vmax / self.ta if self.ta > 0 else 0.0
        self.dq = self.vmax * (self.ta + self.tc)
        self.qf = self.q0 + (self.dir * self.dq)

    def obtener_velocidad(self, t):
        if t < 0: return 0.0
        elif t < self.ta: return self.dir * (self.amax * t)
        elif t < (self.ta + self.tc): return self.dir * self.vmax
        elif t <= self.tf: return self.dir * (self.vmax - self.amax * (t - (self.ta + self.tc)))
        else: return 0.0

class GeneradorCuadraticoVT:
    def __init__(self, q0, vmax, t_total):
        self.q0 = q0
        self.vmax = vmax
        self.tf = max(t_total, 0.1)
        self.dq = (2.0 / 3.0) * self.vmax * self.tf
        self.qf = self.q0 + self.dq

    def obtener_velocidad(self, t):
        if t < 0 or t > self.tf: return 0.0
        return 4.0 * self.vmax * (t / self.tf - (t / self.tf)**2)

# ==========================================
# NODO ROS 2 HIL
# ==========================================
class NodoPuenteHIL(Node):
    def __init__(self):
        super().__init__('nodo_puente_hil')
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.declare_parameter('puerto_serial', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        puerto = self.get_parameter('puerto_serial').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Publicador de comandos de VELOCIDAD a Gazebo
        self.publisher_ = self.create_publisher(Float64MultiArray, '/scara_arm_controller/commands', 10)
        
        # Variables de estado
        self.q_actual = [0.0, 0.0, 0.0]       # Posición articular integrada desde encoders
        self.v_actual = [0.0, 0.0, 0.0]       # Velocidad teórica enviada a ESP32
        self.v_real = [0.0, 0.0, 0.0]         # Velocidad REAL medida por encoders ESP32
        self.u_actual = [0.0, 0.0, 0.0]       # Esfuerzo / PWM
        
        # Historiales para graficación
        self.hist_t = []
        self.hist_q = [[], [], []]
        self.hist_v = [[], [], []]            # Velocidad cruda
        self.hist_v_ref = [[], [], []]        # Velocidad TEÓRICA (Referencia)
        self.hist_v_filt = [[], [], []]       # Velocidad REAL FILTRADA
        self.hist_u = [[], [], []]
        
        # Filtro Pasa-Bajas Exponencial
        self.alpha_filtro = 0.25              # Entre 0 y 1 (menor = más suave)
        self.v_filtrada_prev = [0.0, 0.0, 0.0]
        
        self.perfiles = [None, None, None]
        self.inicio_movimiento = 0.0
        self.modo_actual = "NINGUNO"
        self.timer_period = 0.05
        
        try:
            self.ser = serial.Serial(puerto, baudrate, timeout=0.1)
            self.get_logger().info(f'ESP32 Conectada en {puerto}')
            threading.Thread(target=self.escuchar_esp32, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'Sin ESP32: {e}')
            self.ser = None

        self.timer = self.create_timer(self.timer_period, self.bucle_control_gazebo)

    def escuchar_esp32(self):
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if linea:
                    partes = linea.split(',')
                    if len(partes) == 9: 
                        try:
                            vals = [float(p) for p in partes]
                            self.u_actual = [vals[2], vals[5], vals[8]]
                            
                            # Filtro de ruido y asignación de signo
                            v1 = (vals[1] if vals[0] > 0 else -vals[1]) if abs(vals[0]) > 0.001 else 0.0
                            v2 = (vals[4] if vals[3] > 0 else -vals[4]) if abs(vals[3]) > 0.001 else 0.0
                            v3 = (vals[7] if vals[6] > 0 else -vals[7]) if abs(vals[6]) > 0.001 else 0.0
                            
                            self.v_real = [v1, v2, v3]
                            
                            if self.modo_actual == "ESPERANDO_ESP32":
                                self.v_actual = [v1, v2, v3]
                                
                        except ValueError:
                            pass
                    else:
                        self.get_logger().info(f"[ESP32]: {linea}")
            except: pass

    def cinematica_inversa(self, x, y, z):
        L1 = 0.20  # Longitud Eslabón 1 (m)
        L2 = 0.128 # Longitud Eslabón 2 (m)

        r_cuadrado = x**2 + y**2
        if r_cuadrado > (L1 + L2)**2:
            self.get_logger().error(f"¡El punto ({x}, {y}) está fuera de alcance!")
            raise ValueError("Punto fuera de alcance")

        cos_q2 = (r_cuadrado - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q2 = max(min(cos_q2, 1.0), -1.0) 
        q2 = math.acos(cos_q2)

        beta = math.atan2(L2 * math.sin(q2), L1 + L2 * cos_q2)
        alpha = math.atan2(y, x)
        q1 = alpha - beta
        q3 = z 

        return [q1, q2, q3]

    def ir_a_xyz(self, x, y, z, tiempo_total):
        try:
            q_objetivo = self.cinematica_inversa(x, y, z)
            
            vel_max_necesaria = [0.0, 0.0, 0.0]
            tiempos = [tiempo_total, tiempo_total, tiempo_total]
            
            for i in range(3):
                delta_q = q_objetivo[i] - self.q_actual[i]
                vel_max_necesaria[i] = (3.0 * delta_q) / (2.0 * tiempo_total)
            
            self.get_logger().info(f"XYZ a Target Articular: q1={q_objetivo[0]:.2f}, q2={q_objetivo[1]:.2f}, q3={q_objetivo[2]:.2f}")
            self.ejecutar_trayectoria("Trapezoidal (Vel, Tiempo)", vel_max_necesaria, tiempos)
            
        except ValueError:
            pass 

    def ejecutar_trayectoria(self, tipo, vel_max, tiempos):
        self.modo_actual = "PERFIL"
        
        # Limpiar historiales
        self.hist_t.clear()
        self.hist_q = [[], [], []]
        self.hist_v = [[], [], []]
        self.hist_v_ref = [[], [], []]
        self.hist_v_filt = [[], [], []]
        self.hist_u = [[], [], []]
        self.v_filtrada_prev = [0.0, 0.0, 0.0]

        if tipo == "Trapezoidal (Vel, Tiempo)":
            self.perfiles = [GeneradorTrapezoidalVT(self.q_actual[i], vel_max[i], tiempos[i]) for i in range(3)]
        elif tipo == "Cuadrático (Vel, Tiempo)":
            self.perfiles = [GeneradorCuadraticoVT(self.q_actual[i], vel_max[i], tiempos[i]) for i in range(3)]
        elif tipo == "Control en ESP32 (Vel, Tiempo)":
            self.modo_actual = "ESPERANDO_ESP32"
            self.inicio_movimiento = self.get_clock().now().nanoseconds / 1e9
            
            if self.ser and self.ser.is_open:
                q_obj1 = self.q_actual[0] + (vel_max[0] * tiempos[0])
                q_obj2 = self.q_actual[1] + (vel_max[1] * tiempos[1])
                q_obj3 = self.q_actual[2] + (vel_max[2] * tiempos[2])
                
                cmd = f"P,{q_obj1:.3f},{abs(vel_max[0]):.3f},1.0,{q_obj2:.3f},{abs(vel_max[1]):.3f},1.0,{q_obj3:.3f},{abs(vel_max[2]):.3f},1.0\n"
                self.ser.write(cmd.encode('utf-8'))
            return
            
        self.inicio_movimiento = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Iniciando {tipo}")

    def bucle_control_gazebo(self):
        dt = self.timer_period
        
        # Sincronización continua de la posición articular usando la velocidad real
        for i in range(3):
            self.q_actual[i] += self.v_real[i] * dt

        if self.modo_actual == "PERFIL":
            t_actual = (self.get_clock().now().nanoseconds / 1e9) - self.inicio_movimiento
            terminados = 0

            for i in range(3):
                if self.perfiles[i]:
                    v_teor = self.perfiles[i].obtener_velocidad(t_actual)
                    self.v_actual[i] = v_teor
                    
                    # Filtro pasa-bajas para suavizar la medición del encoder
                    v_filt = (self.alpha_filtro * self.v_real[i]) + ((1.0 - self.alpha_filtro) * self.v_filtrada_prev[i])
                    self.v_filtrada_prev[i] = v_filt
                    
                    # Guardar datos en historiales
                    self.hist_q[i].append(self.q_actual[i])
                    self.hist_v[i].append(self.v_real[i]) 
                    self.hist_v_ref[i].append(v_teor)       # Referencia teórica
                    self.hist_v_filt[i].append(v_filt)      # Medición filtrada
                    self.hist_u[i].append(self.u_actual[i])
                    
                    if t_actual >= self.perfiles[i].tf:
                        terminados += 1
                        
            self.hist_t.append(t_actual)

            if self.ser and self.ser.is_open:
                cmd = f"V,{self.v_actual[0]:.3f},0.15,{self.v_actual[1]:.3f},0.15,{self.v_actual[2]:.3f},0.15\n"
                self.ser.write(cmd.encode('utf-8'))

            if terminados == 3:
                self.modo_actual = "NINGUNO"
                self.v_actual = [0.0, 0.0, 0.0]
                if self.ser and self.ser.is_open:
                    self.ser.write("V,0.0,0.1,0.0,0.1,0.0,0.1\n".encode('utf-8'))

        elif self.modo_actual == "ESPERANDO_ESP32":
            t_actual = (self.get_clock().now().nanoseconds / 1e9) - self.inicio_movimiento
            
            for i in range(3):
                v_filt = (self.alpha_filtro * self.v_real[i]) + ((1.0 - self.alpha_filtro) * self.v_filtrada_prev[i])
                self.v_filtrada_prev[i] = v_filt
                
                self.hist_q[i].append(self.q_actual[i])
                self.hist_v[i].append(self.v_real[i])
                self.hist_v_ref[i].append(0.0)
                self.hist_v_filt[i].append(v_filt)
                self.hist_u[i].append(self.u_actual[i])
            
            self.hist_t.append(t_actual)

            if all(abs(v) == 0.0 for v in self.v_actual) and t_actual > 0.5:
                self.modo_actual = "NINGUNO"
                self.v_actual = [0.0, 0.0, 0.0]

        # Publicar velocidad real a Gazebo
        self.publicar_estado_gazebo(self.q_actual, self.v_real)

    def publicar_estado_gazebo(self, posiciones, velocidades):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocidades]
        self.publisher_.publish(msg)

# ==========================================
# INTERFAZ GRÁFICA (TKINTER)
# ==========================================
class InterfazHIL(tk.Tk):
    def __init__(self, nodo_ros):
        super().__init__()
        self.nodo = nodo_ros
        self.title("Panel HIL - SCARA")
        self.geometry("820x680")
        
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill='both')
        
        self.tab_control = ttk.Frame(self.notebook)
        self.tab_graf = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_control, text='Control por Articulaciones')
        self.notebook.add(self.tab_graf, text='Análisis (Gráficas)')
        
        self.combo_perfil = ttk.Combobox(self.tab_control, values=[
            "Trapezoidal (Vel, Tiempo)", 
            "Cuadrático (Vel, Tiempo)", 
            "Control en ESP32 (Vel, Tiempo)"
        ], width=40)
        self.combo_perfil.current(1) # Por defecto cuadrático
        self.combo_perfil.pack(pady=10)
        
        self.frame_inputs = tk.Frame(self.tab_control)
        self.frame_inputs.pack(pady=10)
        
        headers = ["Articulación", "Velocidad Máx", "Tiempo Total (s)"]
        for col, h in enumerate(headers):
            tk.Label(self.frame_inputs, text=h, font=("Arial", 9, "bold")).grid(row=0, column=col)

        self.entradas = []
        titulos = ["J1", "J2", "J3"]
        for i, nombre in enumerate(titulos):
            tk.Label(self.frame_inputs, text=nombre).grid(row=i+1, column=0)
            fila = []
            for j in range(2):
                ent = tk.Entry(self.frame_inputs, width=10)
                ent.insert(0, "1.0" if j==0 else "3.0")
                ent.grid(row=i+1, column=j+1, padx=5, pady=2)
                fila.append(ent)
            self.entradas.append(fila)

        tk.Button(self.tab_control, text="Ejecutar Movimiento", bg="green", fg="white", command=self.enviar_comando).pack(pady=10)
        tk.Button(self.tab_control, text="Ver Gráficas Resultantes", bg="blue", fg="white", command=self.graficar_datos).pack(pady=5)

        ttk.Separator(self.tab_control, orient='horizontal').pack(fill='x', pady=15)
        tk.Label(self.tab_control, text="Mover a Punto XYZ (Pick & Place)", font=("Arial", 11, "bold")).pack(pady=5)
        
        self.frame_xyz = tk.Frame(self.tab_control)
        self.frame_xyz.pack(pady=5)
        
        tk.Label(self.frame_xyz, text="X (m)").grid(row=0, column=0)
        tk.Label(self.frame_xyz, text="Y (m)").grid(row=0, column=1)
        tk.Label(self.frame_xyz, text="Z (m)").grid(row=0, column=2)
        tk.Label(self.frame_xyz, text="Tiempo (s)").grid(row=0, column=3)
        
        self.ent_x = tk.Entry(self.frame_xyz, width=8)
        self.ent_x.insert(0, "-0.328")
        self.ent_x.grid(row=1, column=0, padx=5)
        
        self.ent_y = tk.Entry(self.frame_xyz, width=8)
        self.ent_y.insert(0, "0.0")
        self.ent_y.grid(row=1, column=1, padx=5)
        
        self.ent_z = tk.Entry(self.frame_xyz, width=8)
        self.ent_z.insert(0, "0.0")
        self.ent_z.grid(row=1, column=2, padx=5)
        
        self.ent_t = tk.Entry(self.frame_xyz, width=8)
        self.ent_t.insert(0, "4.0")
        self.ent_t.grid(row=1, column=3, padx=5)
        
        tk.Button(self.tab_control, text="Ir al punto XYZ", bg="purple", fg="white", 
                  command=self.comando_xyz).pack(pady=10)

        self.fig, self.axs = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
        self.fig.tight_layout(pad=3.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.tab_graf)
        self.canvas.get_tk_widget().pack(expand=True, fill='both')

    def enviar_comando(self):
        try:
            tipo = self.combo_perfil.get()
            vels = [float(e[0].get()) for e in self.entradas]
            tiempos = [float(e[1].get()) for e in self.entradas]
            self.nodo.ejecutar_trayectoria(tipo, vels, tiempos)
        except ValueError:
            messagebox.showerror("Error", "Ingrese solo números.")

    def comando_xyz(self):
        try:
            x = float(self.ent_x.get())
            y = float(self.ent_y.get())
            z = float(self.ent_z.get())
            tiempo = float(self.ent_t.get())
            self.nodo.ir_a_xyz(x, y, z, tiempo)
        except ValueError:
            messagebox.showerror("Error", "Ingrese solo números.")

    def graficar_datos(self):
        t = self.nodo.hist_t
        if not t:
            messagebox.showwarning("Aviso", "No hay datos para graficar. Ejecute un perfil primero.")
            return

        for ax in self.axs: 
            ax.clear()
        
        colores = ['tab:blue', 'tab:orange', 'tab:green']
        
        # --- SUBPLOT 1: POSICIÓN ---
        for i in range(3):
            self.axs[0].plot(t, self.nodo.hist_q[i], label=f'J{i+1} Pos', color=colores[i], linewidth=1.5)
        self.axs[0].set_ylabel('Posición (rad/m)')
        self.axs[0].legend(loc='upper right')
        self.axs[0].grid(True, linestyle='--', alpha=0.6)

        # --- SUBPLOT 2: VELOCIDAD (REFERENCIA Y REAL FILTRADA) ---
        # Referencia teórica de J1 (Parábola/Trapezoide)
        self.axs[1].plot(t, self.nodo.hist_v_ref[0], '--', label='J1 Ref Teórica', color='black', alpha=0.8, linewidth=1.5)
        
        # Velocidades filtradas de las 3 articulaciones
        for i in range(3):
            self.axs[1].plot(t, self.nodo.hist_v_filt[i], label=f'J{i+1} Vel Real', color=colores[i], linewidth=1.8)
            
        self.axs[1].set_ylabel('Velocidad (rad/s)')
        self.axs[1].legend(loc='upper right')
        self.axs[1].grid(True, linestyle='--', alpha=0.6)

        # --- SUBPLOT 3: ESFUERZO / PWM ---
        for i in range(3):
            self.axs[2].plot(t, self.nodo.hist_u[i], label=f'J{i+1} Esfuerzo', color=colores[i], linewidth=1.5)
        self.axs[2].set_ylabel('Esfuerzo (u_k)')
        self.axs[2].set_xlabel('Tiempo (s)')
        self.axs[2].legend(loc='upper right')
        self.axs[2].grid(True, linestyle='--', alpha=0.6)

        self.canvas.draw()
        self.notebook.select(self.tab_graf)

def main():
    rclpy.init()
    nodo = NodoPuenteHIL()
    threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True).start()
    app = InterfazHIL(nodo)
    app.mainloop()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()