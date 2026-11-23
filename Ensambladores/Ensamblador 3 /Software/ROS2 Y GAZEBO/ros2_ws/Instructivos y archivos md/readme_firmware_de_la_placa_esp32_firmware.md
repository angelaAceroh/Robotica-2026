# Firmware de la Placa ESP32 (`firmware/`)

Aquí está el código que **se graba directamente dentro del chip ESP32** del robot para leer los encoders, controlar la potencia de los 3 motores DC y mover el servomotor de la garra.

## Subcarpetas
* **`micropython/`** *(Versión activa)*: Código escrito en MicroPython (`main_control.py`, `garra.py`, etc.) que se sube a la placa con **Thonny**.
* **`scara_esp32/`**: Código alternativo escrito en C++ para **Arduino IDE** (`scara_esp32.ino`).

## Instructivo básico
1. Antes de programar la placa, asegúrate de que ningún nodo de ROS 2 esté abierto (solo un programa puede usar el puerto USB `/dev/ttyUSB0` a la vez).
2. Abre **Thonny**, conéctate a la ESP32 y copia los archivos de `micropython/` a la placa.
3. **Importante:** Cierra Thonny por completo antes de arrancar ROS 2 para liberar el puerto serie.