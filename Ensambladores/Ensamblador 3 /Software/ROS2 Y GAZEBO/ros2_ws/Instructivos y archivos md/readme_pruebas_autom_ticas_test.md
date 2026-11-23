# Pruebas Automáticas (`test/`)

Contiene la batería de pruebas unitarias escritas con `pytest` para comprobar que las matemáticas, límites mecánicos y comunicaciones funcionan correctamente antes de mover el brazo físico.

## Archivos importantes
* **`test_kinematics.py` y `test_alcance.py`**: Verifican la cinemática directa/inversa y el área alcanzable del robot.
* **`test_control.py` y `test_control_placa.py`**: Prueban la estabilidad del controlador cinemático y el enlace con la placa.
* **`test_protocolo.py`**: Comprueba el formato de tramas USB y el cálculo de *checksum*.
* **`test_garra.py` y `test_juntas.py`**: Validan las secuencias de movimiento y apertura/cierre de la garra.
* **`micropython_falso/`**: Simulador en software de la placa ESP32 para correr pruebas sin hardware real.

## Instructivo básico
* Para ejecutar todas las pruebas desde la carpeta raíz `ros2_ws`:
  `pytest src/scara_kinematics/test`