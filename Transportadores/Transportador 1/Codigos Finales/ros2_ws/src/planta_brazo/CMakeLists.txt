cmake_minimum_required(VERSION 3.8)
project(planta_brazo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz config worlds
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/trayectorias.py
  scripts/cinematica.py
  scripts/puntos_prueba.py
  scripts/enlace_juntas.py
  scripts/remuestreo_juntas.py
  scripts/gui_articulaciones.py
  scripts/cinematica_directa.py
  scripts/trayectoria_demo.py
  scripts/comparacion_cinematicas.py
  scripts/pares.py
  scripts/perfiles.py
  scripts/panel_trayectoria.py
  scripts/trayectoria_lineal.py
  scripts/jacobiano.py
  scripts/control_cinematico.py
  scripts/graficas_guia5.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
