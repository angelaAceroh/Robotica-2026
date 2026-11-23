# Simulador de Placa ESP32 (`test/micropython_falso/`)

Permite ejecutar el código real de MicroPython (`main_control.py`) dentro del computador simulando los pines electrónicos y el comportamiento de los motores.

## Archivos importantes
* **`machine.py`**: Imita los módulos de hardware de MicroPython (`Pin`, `PWM`, `ADC`) en Python estándar.
* **`placa_falsa.py`**: Simula respuesta de encoders, atascos mecánicos y desconexiones para probar la seguridad del sistema.

## Instructivo básico
* Es utilizado automáticamente por `placa_simulada.py` y `pytest`. No requiere ejecución manual.