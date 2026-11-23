# Archivos de Configuración (`config/`)

Esta carpeta guarda las medidas reales del robot y la calibración de los motores. Si necesitas ajustar un límite o cambiar una medida física, se edita aquí sin tocar el código fuente.

## Archivos importantes
* **`scara_urdf3.yaml`** *(Principal)*: Medidas reales del SCARA de 3 grados de libertad (longitudes de eslabones `l1` y `l2`, ángulos iniciales `b1` y `b2`, recorrido del eje Z y límites de giro).
* **`micropython.yaml`**: Calibración de los motores reales en la ESP32 (recorrido mínimo/máximo en grados y milímetros, sentido de giro, posición HOME y velocidades).
* **`scara_urdf.yaml`**: Configuración de respaldo para la versión teórica de 4 grados de libertad.

## Instructivo básico
* Todos los nodos cargan `scara_urdf3.yaml` al iniciar (los archivos de `launch/` lo hacen automáticamente).
* Si editas números en estos archivos `.yaml`, escribe siempre el punto decimal (por ejemplo `60.0` en lugar de `60`) para que ROS 2 los interprete como números flotantes.