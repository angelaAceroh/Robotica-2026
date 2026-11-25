# planta_brazo · scripts (nodos y módulos)

Los nodos se instalan como ejecutables (`CMakeLists.txt`); los módulos sin ROS los importan.

| Archivo | Qué es |
|---|---|
| `cinematica.py` | Cinemática directa e inversa (fuente única de la geometría). |
| `cinematica_directa.py` | Nodo: pose del TCP y de la brida desde /joint_states, y traza para RViz. |
| `comparacion_cinematicas.py` | Nodo: recorre los puntos de prueba y compara cinematica directa e inversa. |
| `control_cinematico.py` | Nodo: control cinematico por jacobiano inverso. |
| `enlace_juntas.py` | Envia posiciones a RViz (/joint_states) o a Gazebo (cmd_pos). |
| `graficas_guia5.py` | Gráficas y métricas de la Guía 5 desde el CSV. |
| `gui_articulaciones.py` | Nodo GUI: sliders de juntas, cinematica directa e inversa. |
| `jacobiano.py` | Jacobiano del robot 3R. |
| `panel_trayectoria.py` | Nodo GUI: planeacion de trayectorias (recta o ciclo de 14 puntos) y perfiles. |
| `pares.py` | Pares de gravedad en las juntas. |
| `perfiles.py` | Perfiles de velocidad trapezoidal y cubico. |
| `puntos_prueba.py` | Puntos de prueba del informe. |
| `remuestreo_juntas.py` | Nodo: baja /joint_states de Gazebo de 1000 Hz a 60 Hz. |
| `trayectoria_demo.py` | Nodo: recorre la trayectoria de demostracion de 10 puntos. |
| `trayectoria_lineal.py` | Nodo: recta A-B por cinemática inversa. |
| `trayectorias.py` | Puntos de la trayectoria de demostracion y su interpolacion. |
