# Variables de Entorno para Gazebo (`scara_urdf/env-hooks/`)

## Archivos importantes
* **`gz_resource_path.dsv.in`**: Archivo plantilla que añade la carpeta `share` a la ruta de búsqueda de **Gazebo** (`GZ_SIM_RESOURCE_PATH`) cada vez que ejecutas `source install/setup.bash`.

## Instructivo básico
* Permite que Gazebo encuentre las piezas `.STL` de la carpeta `meshes/`. No lo borres ni lo modifiques.