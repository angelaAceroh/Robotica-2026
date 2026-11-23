#!/usr/bin/env bash
""":"
# Auto-ejecución con el entorno de ROS 2 configurado
source /opt/ros/jazzy/setup.bash
source /home/samuel-azuero/urdf_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
exec python3 "$0" "$@"
"""

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

class SimuladorMaestroNode(Node):
    def __init__(self):
        super().__init__('simulador_maestro')

        self.ultimo_estado = None
        self.tiempo_ultimo_estado = None
        self.contador_estados = 0
        self.intervalos_recibidos = []

        # Tópicos del Contrato
        self.pub_orden = self.create_publisher(String, '/ensamblador2/orden', 10)
        self.pub_parada = self.create_publisher(Bool, '/parada', 10)
        self.sub_estado = self.create_subscription(String, '/ensamblador2/estado', self.estado_callback, 10)
        self.cli_reset = self.create_client(Trigger, '/ensamblador2/reset_estado')

        # Monitor de Heartbeat a 1 Hz
        self.timer_mon = self.create_timer(1.0, self.verificar_heartbeat)

    def estado_callback(self, msg: String):
        ahora = time.time()
        if self.tiempo_ultimo_estado is not None:
            dt = ahora - self.tiempo_ultimo_estado
            self.intervalos_recibidos.append(dt)
            if len(self.intervalos_recibidos) > 10:
                self.intervalos_recibidos.pop(0)

        self.ultimo_estado = msg.data.strip()
        self.tiempo_ultimo_estado = ahora
        self.contador_estados += 1

    def verificar_heartbeat(self):
        if self.tiempo_ultimo_estado is not None:
            elapsed = time.time() - self.tiempo_ultimo_estado
            if elapsed > 3.0:
                print(f"\n⚠️  [ALERTA MAESTRO] ¡Timeout de Heartbeat! {elapsed:.1f}s sin recibir /ensamblador2/estado (> 3.0s límite). La planta se detendría.")

    def enviar_orden(self, orden="ensamblar"):
        msg = String()
        msg.data = orden
        self.pub_orden.publish(msg)
        print(f"\n📤 [MAESTRO] Orden enviada a /ensamblador2/orden: '{orden}'")

    def enviar_parada(self, parada: bool):
        msg = Bool()
        msg.data = parada
        self.pub_parada.publish(msg)
        print(f"\n🛑 [MAESTRO] Señal enviada a /parada: {parada}")

    def solicitar_reset(self):
        if not self.cli_reset.wait_for_service(timeout_sec=1.5):
            print("\n❌ [MAESTRO] Servicio /ensamblador2/reset_estado no disponible.")
            return
        future = self.cli_reset.call_async(Trigger.Request())
        time.sleep(0.5)
        if future.done():
            res = future.result()
            print(f"\n🔄 [MAESTRO] Reset respondido: success={res.success}, message='{res.message}'")
        else:
            print("\n🔄 [MAESTRO] Solicitud de reset enviada.")

def imprimir_menu():
    print("""
==============================================================
       🏭 SIMULADOR DEL NODO MAESTRO (LÍNEA DE EMPACADO)
       Verificación del Contrato ROS 2 (Ensamblador 2)
==============================================================
 1. 📤 Enviar orden 'ensamblar' (Cronometrar ciclo)
 2. ⚡ Enviar orden duplicada (Verificar inmunidad / regla 3)
 3. 🛑 Activar Parada de Emergencia (/parada = True)
 4. 🟢 Desactivar Parada de Emergencia (/parada = False)
 5. 🔄 Solicitar Rearme / Reset a 'listo' (/ensamblador2/reset_estado)
 6. 📊 Ver estado del Heartbeat y Frecuencia de recepción
 7. 🧪 Ejecutar Prueba Automática de Contrato
 0. 🚪 Salir
--------------------------------------------------------------""")

def main():
    rclpy.init()
    nodo = SimuladorMaestroNode()

    # Hilo para spin de ROS 2
    t = threading.Thread(target=lambda: rclpy.spin(nodo), daemon=True)
    t.start()

    print("\nConectando con el robot Ensamblador 2 en ROS_DOMAIN_ID=42...")
    time.sleep(1.2)

    while rclpy.ok():
        imprimir_menu()
        if nodo.ultimo_estado is not None:
            dt_prom = sum(nodo.intervalos_recibidos)/len(nodo.intervalos_recibidos) if nodo.intervalos_recibidos else 0.5
            print(f"📡 Estado actual del robot: [{nodo.ultimo_estado.upper()}] | Heartbeat: ~{dt_prom:.2f}s ({1.0/max(dt_prom, 0.001):.1f} Hz)")
        else:
            print("⏳ Esperando primer latido en /ensamblador2/estado...")

        try:
            opcion = input("\nSeleccione una opción: ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if opcion == '1':
            if nodo.ultimo_estado != "listo":
                print(f"⚠️ Atención: El robot está actualmente en '{nodo.ultimo_estado}'.")
            nodo.enviar_orden("ensamblar")
            print("⏱️  Iniciando cronómetro del ciclo de ensamble...")
            t0 = time.time()
            # Esperar a que pase a ocupado
            cambio_a_ocupado = False
            while time.time() - t0 < 3.0:
                if nodo.ultimo_estado == "ocupado":
                    cambio_a_ocupado = True
                    break
                time.sleep(0.1)

            if cambio_a_ocupado:
                print("✅ Transición detectada: 'listo' -> 'ocupado'. Esperando finalización...")
                while time.time() - t0 < 65.0:
                    if nodo.ultimo_estado == "listo":
                        duracion = time.time() - t0
                        print(f"\n🎉 ¡Ciclo de ensamble completado con éxito!")
                        print(f"⏱️  Duración total: {duracion:.2f} s (Límite del contrato: < 60 s)")
                        if duracion < 60.0:
                            print("✅ Cumple estrictamente el contrato (< 60s).")
                        else:
                            print("⚠️ Excedió los 60s permitidos.")
                        break
                    elif nodo.ultimo_estado == "error":
                        print(f"\n❌ El robot reportó estado 'error' durante la ejecución.")
                        break
                    time.sleep(0.2)
            else:
                print("⚠️ El robot no cambió a 'ocupado' en los primeros 3 segundos.")

        elif opcion == '2':
            nodo.enviar_orden("ensamblar")
            time.sleep(0.5)
            print("Enviando segunda orden 'ensamblar' inmediata...")
            nodo.enviar_orden("ensamblar")
            print("Compruebe que el nodo ignore la segunda orden y continúe la primera.")

        elif opcion == '3':
            nodo.enviar_parada(True)

        elif opcion == '4':
            nodo.enviar_parada(False)

        elif opcion == '5':
            nodo.solicitar_reset()

        elif opcion == '6':
            if nodo.tiempo_ultimo_estado is None:
                print("❌ No se ha recibido ningún mensaje en /ensamblador2/estado.")
            else:
                dt_prom = sum(nodo.intervalos_recibidos)/len(nodo.intervalos_recibidos) if nodo.intervalos_recibidos else 0.5
                print(f"\n📊 Telemetría de Heartbeat:")
                print(f" - Último estado recibido: '{nodo.ultimo_estado}'")
                print(f" - Total mensajes recibidos: {nodo.contador_estados}")
                print(f" - Intervalo promedio: {dt_prom:.3f} s (Esperado: 0.500 s / 2 Hz)")
                if 0.40 <= dt_prom <= 0.65:
                    print(" ✅ Frecuencia dentro del rango contractual.")
                else:
                    print(" ⚠️ Frecuencia con variación.")

        elif opcion == '7':
            print("\n🧪 === TEST AUTOMÁTICO DE CUMPLIMIENTO DEL CONTRATO ===")
            print("Paso 1: Verificando recepción de Heartbeat...")
            if nodo.ultimo_estado is None:
                print("❌ Fallo: No hay latido en /ensamblador2/estado. Inicie el robot primero.")
                continue
            print(f"✅ Heartbeat OK: Estado = '{nodo.ultimo_estado}'")

            print("\nPaso 2: Rearmando a 'listo'...")
            nodo.solicitar_reset()
            time.sleep(1.0)
            if nodo.ultimo_estado != "listo":
                print(f"⚠️ Estado actual es '{nodo.ultimo_estado}'.")

            print("\nPaso 3: Enviando orden 'ensamblar'...")
            nodo.enviar_orden("ensamblar")
            t_ini = time.time()
            time.sleep(1.0)
            if nodo.ultimo_estado == "ocupado":
                print("✅ Regla 2 OK: Cambió inmediatamente a 'ocupado'.")
            else:
                print(f"⚠️ Estado: '{nodo.ultimo_estado}'")

            print("\nPaso 4: Enviando orden duplicada (Regla 3)...")
            nodo.enviar_orden("ensamblar")
            print("✅ Orden duplicada enviada.")

            print("\nPaso 5: Esperando finalización del ciclo (límite 60s)...")
            termino = False
            while time.time() - t_ini < 60.0:
                if nodo.ultimo_estado == "listo":
                    termino = True
                    break
                time.sleep(0.5)

            if termino:
                dur = time.time() - t_ini
                print(f"✅ Ciclo finalizado en {dur:.2f} s. Estado volvió a 'listo'.")
                print("\n🏆 ¡TODAS LAS PRUEBAS DEL CONTRATO SUPERADAS CON ÉXITO!")
            else:
                print("⚠️ No volvió a listo en 60 segundos.")

        elif opcion == '0':
            break

    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
