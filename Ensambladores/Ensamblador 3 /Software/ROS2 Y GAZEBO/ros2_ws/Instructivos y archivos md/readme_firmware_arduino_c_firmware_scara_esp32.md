# Firmware en Arduino C++ (`firmware/scara_esp32/`)

Contiene una versión alternativa del firmware de la ESP32 escrita en C++ para Arduino IDE.

## Archivos importantes
* **`scara_esp32.ino`**: Código principal para Arduino con control PID de posición y comunicación serie con verificación de *checksum*.
* **`scara_esp32.ino.bak4gdl`**: Respaldo histórico de la versión de 4 grados de libertad.

## Instructivo básico
* Si deseas usar este firmware en vez de MicroPython, ábrelo en **Arduino IDE**, selecciona la placa *ESP32 Dev Module* y cárgalo a `115200` baudios.