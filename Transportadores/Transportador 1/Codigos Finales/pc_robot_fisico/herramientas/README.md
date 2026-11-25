# Herramientas

| Archivo | Qué hace |
|---|---|
| `esp_repl.py` | REPL crudo de MicroPython por pyserial: sube archivos a la ESP32 y verifica su suma |
| `graficar_log.py` | grafica el CSV que deja `mover_robot_vel.py --log` (plan contra medida) |
| `vigilar_encoder.py` | muestra en vivo las cuentas del encoder del gripper |
| `pares.py` | par de gravedad en las juntas frente al par que dan los motores |

Se ejecutan desde esta carpeta; usan `config.py` de la carpeta superior.
