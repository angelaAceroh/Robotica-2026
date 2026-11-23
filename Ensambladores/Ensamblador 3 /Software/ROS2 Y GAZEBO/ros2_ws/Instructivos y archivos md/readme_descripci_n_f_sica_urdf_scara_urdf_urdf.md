# Descripción Física del Robot (`scara_urdf/urdf/`)

Aquí se define cómo se conectan las piezas del robot, qué ejes giran, qué eje sube y baja y cuáles son sus límites de movimiento.

## Archivos importantes
* **`scara_urdf_gz.urdf`** *(Principal para simulación)*: Modelo completo para **Gazebo** con anclaje al suelo, límites reales y controladores de articulación.
* **`scara_urdf.urdf`**: Modelo base exportado desde SolidWorks para visualización estática en RViz2.

## Instructivo básico
* No edites estos archivos a menos que modifiques el diseño mecánico en CAD. Si haces cambios, vuelve a compilar con `colcon build --packages-select scara_urdf`.