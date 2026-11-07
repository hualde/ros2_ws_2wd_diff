import os
from glob import glob

from setuptools import setup

package_name = 'mpu6050_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jhualde@lizarte.com',
    description='Nodo ROS 2 para el IMU MPU6050.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['mpu6050_node = mpu6050_imu.mpu6050_node:main'],
    },
)
