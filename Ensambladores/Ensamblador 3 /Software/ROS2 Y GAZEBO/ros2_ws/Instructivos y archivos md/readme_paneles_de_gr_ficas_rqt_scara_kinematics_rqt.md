# Paneles de Gráficas RQT (`scara_kinematics/rqt/`)

Configuraciones de pantalla para **rqt_plot** que permiten ver gráficas matemáticas en tiempo real mientras se mueve el robot.

## Archivos importantes
* **`control.perspective`**: Muestra cuatro gráficas simultáneas: posición `(X, Y)`, altura `Z`, error de seguimiento y velocidades de las juntas.
* **`ruta.perspective`**: Configuración de gráficas adaptada al seguimiento de rutas.

## Instructivo básico
* Activa estas gráficas automáticamente agregando `rqt:=true` al comando de lanzamiento:
  `ros2 launch scara_kinematics control.launch.py rqt:=true`