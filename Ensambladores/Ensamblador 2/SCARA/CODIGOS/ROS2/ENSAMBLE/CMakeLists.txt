cmake_minimum_required(VERSION 3.5)
project(Ensamble)

find_package(ament_cmake REQUIRED)

# Instalar el script de Python para que ros2 launch pueda encontrarlo
install(PROGRAMS
  gui_interactiva.py
  gui_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()