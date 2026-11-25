import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'linea_planta'


def arbol(origen):
    """Instala una carpeta entera respetando su estructura (las mallas van en
    modelos/<paquete>/meshes/ para que Gazebo resuelva package://<paquete>)."""
    salida = []
    for raiz, _, archivos in os.walk(origen):
        if archivos:
            salida.append((os.path.join('share', package_name, raiz),
                           [os.path.join(raiz, a) for a in archivos]))
    return salida


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ] + arbol('modelos'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo 5 - Planta Brazo',
    maintainer_email='est.julian.castaneda@unimilitar.edu.co',
    description='Linea de ensamble en Gazebo con los cinco robots.',
    license='Apache-2.0',
)
