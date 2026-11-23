# Firmware en MicroPython (`firmware/micropython/`)

Aquí se encuentra el código que corre dentro de la placa **ESP32** para ejecutar el lazo de control PI de velocidad con freno activo y accionar la garra.

## Archivos importantes
* **`main_control.py`** *(Principal)*: Programa activo de control en lazo cerrado para los tres motores. Debe guardarse dentro de la memoria de la ESP32 con el nombre **`main.py`** para que arranque automáticamente al encender la placa.
* **`garra.py`**: Controla el servomotor de la garra en el pin **GPIO 33** (`1` abre, `0` cierra) y supervisa el voltaje de alimentación.
* **`main.py`**: Versión anterior de control punto a punto.
* **`Teodoro.py`**: Script de prueba individual para el eje vertical.

## Instructivo básico
1. Abre **Thonny**, elige el intérprete *MicroPython (ESP32)* y selecciona `/dev/ttyUSB0`.
2. Sube **`main_control.py`** (guardándolo como `main.py` en la placa) y **`garra.py`**.
3. Cierra Thonny para liberar el puerto USB antes de iniciar ROS 2.