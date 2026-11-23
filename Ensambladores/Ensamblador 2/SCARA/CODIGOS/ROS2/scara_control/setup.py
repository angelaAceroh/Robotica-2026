import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scara_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Samuel Azuero',
    maintainer_email='samuel@example.com',
    description='Controlador SCARA en ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scara_serial_node = scara_control.scara_serial_node:main',
            'scara_kinematics_node = scara_control.scara_kinematics_node:main',
            'scara_gui_node = scara_control.scara_gui_node:main',
            'scara_kinematics_planta_node = scara_control.scara_kinematics_planta_node:main',
            'scara_gui_planta_node = scara_control.scara_gui_planta_node:main',
        ],
    },
)