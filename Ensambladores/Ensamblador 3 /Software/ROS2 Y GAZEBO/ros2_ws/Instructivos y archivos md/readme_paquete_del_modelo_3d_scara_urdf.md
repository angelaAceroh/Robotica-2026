# Paquete del Modelo 3D y Simulación (`scara_urdf`)

Este paquete contiene **el diseño tridimensional y físico** del robot SCARA exportado desde SolidWorks para dibujarlo en **RViz2** y simularlo en **Gazebo**.

## Subcarpetas
* **`urdf/`** *(Planos 3D)*: Archivos que definen las piezas, articulaciones, masas y física del robot.
* **`meshes/`**: Piezas tridimensionales en formato `.STL`.
* **`launch/`**: Lanzadores rápidos para abrir el modelo 3D solo.
* **`rviz/`**: Configuraciones de vista para RViz2.
* **`config/` y `env-hooks/`**: Lista de articulaciones y rutas de recursos para Gazebo.

## Archivos importantes
* **`CMakeLists.txt` y `package.xml`**: Instrucciones para que `colcon build` instale el modelo 3D en el sistema.
* **`URDF.md`**: Documentación detallada de las medidas y estructura de cada eslabón.

## Instructivo básico
* Para abrir únicamente el modelo 3D y mover sus articulaciones con barras deslizantes en pantalla:
  `ros2 launch scara_urdf display.launch.py`