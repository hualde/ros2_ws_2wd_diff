import os
from glob import glob

from setuptools import setup

package_name = 'obstacle_stop'

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
    description='Frena el avance del robot si la OAK-D detecta un obstáculo delante.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = obstacle_stop.obstacle_stop_node:main',
        ],
    },
)
