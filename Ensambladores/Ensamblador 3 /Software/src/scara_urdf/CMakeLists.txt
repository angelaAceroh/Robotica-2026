cmake_minimum_required(VERSION 3.8)
project(scara_urdf)

find_package(ament_cmake REQUIRED)

# Paquete de solo-datos: instalamos las carpetas en share/scara_urdf/
install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Permite que Gazebo resuelva package://scara_urdf/meshes/... anadiendo
# el directorio share/ del workspace a GZ_SIM_RESOURCE_PATH.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_resource_path.dsv.in")

ament_package()
