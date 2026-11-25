import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'linea_maestro'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo 5 - Planta Brazo',
    maintainer_email='est.julian.castaneda@unimilitar.edu.co',
    description='Nodo maestro de la linea de empacado: coordina los 5 robots.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maestro = linea_maestro.maestro:main',
            'robot_simulado = linea_maestro.robot_simulado:main',
        ],
    },
)
