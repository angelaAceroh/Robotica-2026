# Carpeta de Código Fuente (`src/`)

Aquí se guarda **todo el código real y los modelos 3D** del proyecto SCARA, dividido en dos paquetes de ROS 2:

1. **`scara_kinematics/`** *(El cerebro)*: Contiene todos los nodos de ROS 2, cálculos de cinemática y control, ventanas gráficas (GUI) y el firmware que va dentro de la placa ESP32.
2. **`scara_urdf/`** *(El cuerpo 3D)*: Contiene las piezas CAD exportadas de SolidWorks (`.STL`) y los planos físicos (`.urdf`) para visualizar el robot en RViz y simularlo en Gazebo.

## Instructivo básico
* No ejecutes los archivos `.py` directamente con doble clic.
* Cuando modifiques algún archivo aquí dentro, vuelve a la carpeta raíz `ros2_ws` y ejecuta `colcon build` seguido de `source install/setup.bash`.