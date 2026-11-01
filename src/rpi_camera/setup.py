import os
from glob import glob

from setuptools import setup

package_name = 'rpi_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jhualde@lizarte.com',
    description='Publica la imagen de la camara CSI (Raspberry Pi) en ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_camera_node = rpi_camera.rpi_camera_node:main',
        ],
    },
)
