#!/usr/bin/env python3
"""
prueba_pick_and_place.py
Script para ejecutar la secuencia completa de Pick and Place en el robot SCARA RRP.
Se conecta al nodo de control cinemático mediante ROS 2.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = Node('pick_and_place_caller')
    
    print("==================================================")
    print("      SECUENCIA DE PICK AND PLACE - SCARA RRP     ")
    print("==================================================")
    print("Secuencia programada:")
    print("  1. Elevar Z a 8.0 cm (Z_HOME seguro retraído)")
    print("  2. Abrir Gripper (25% PWM, ~2.5V)")
    print("  3. Mover a aproximación Pick (X=10 cm, Y=20 cm, Z=8 cm)")
    print("  4. Descender a agarrar el objeto (Z=14 cm)")
    print("  5. Cerrar Gripper con fuerza suave y segura (18% PWM, ~1.8V)")
    print("  6. Levantar con objeto a Z=8 cm")
    print("  7. Trasladar a posición Place (X=-10 cm, Y=20 cm, Z=8 cm)")
    print("  8. Descender a posición de entrega (Z=14 cm)")
    print("  9. Abrir Gripper (soltar objeto)")
    print(" 10. Subir a Z=8 cm a altura segura")
    print(" 11. Regresar a posición de reposo final (+Y)")
    print(" 12. Cerrar Gripper en reposo final (18% PWM, ~1.8V)")
    print("--------------------------------------------------")
    
    cli = node.create_client(Trigger, '/run_pick_and_place')
    
    print("Conectando con el controlador cinemático...")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("❌ Error: El servicio /run_pick_and_place no está respondiendo.")
        print("Asegúrese de haber iniciado primero: 'bash 2_lanzar_control_esp32.sh'")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
        
    print("🚀 Enviando orden de ejecución de Pick & Place...")
    req = Trigger.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        print(f"Respuesta: {future.result().message}")
        if future.result().success:
            print("✔ Secuencia Pick & Place iniciada exitosamente.")
            print("Monitoree los movimientos en el robot y en la interfaz gráfica.")
        else:
            print("⚠️ El controlador reportó:", future.result().message)
    else:
        print("❌ Error al llamar al servicio.")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
