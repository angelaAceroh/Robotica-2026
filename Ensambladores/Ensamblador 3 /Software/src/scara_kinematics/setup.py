from glob import glob

from setuptools import find_packages, setup

package_name = 'scara_kinematics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/firmware/scara_esp32',
         glob('firmware/scara_esp32/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Quevedo',
    maintainer_email='squevedo1850@gmail.com',
    description='El SCARA en seis nodos: aduana, Alvin, Simon, Teodoro, cinematica directa y receptor.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            # Los seis nodos, en el orden en que circulan los datos.
            'esp = scara_kinematics.esp:main',              # 5. la aduana
            'alvin = scara_kinematics.alvin:main',          # 2. hombro
            'simon = scara_kinematics.simon:main',          # 3. codo
            'teodoro = scara_kinematics.teodoro:main',      # 4. husillo
            'cc = scara_kinematics.cc:main',                # 1. cinematica directa
            'esp_rec = scara_kinematics.esp_rec:main',      # 6. el receptor
            # La interfaz del brazo.
            'brazo_hw = scara_kinematics.brazo_hw:main',
            # El que vigila que los datos lleguen enteros de un nodo al otro.
            'auditor = scara_kinematics.auditor:main',
            # 7. Planificacion de trayectorias (anadido, no toca a los seis).
            'tray = scara_kinematics.tray:main',
            'tray_gui = scara_kinematics.tray_gui:main',    # su ventana
            # Trayectorias reales por Jacobiano (la guia, con las medidas de la mesa).
            'ruta = scara_kinematics.ruta:main',            # calcula: informe, CSV, graficas
            'ruta_nodo = scara_kinematics.ruta_nodo:main',  # la manda a gz o a la placa
        ],
    },
)
