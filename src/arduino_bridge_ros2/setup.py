from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arduino_bridge_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josemsotov',
    description='ROS2 bridge para MOTOR-INTERFACE-V14 (Arduino Mega)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_node = arduino_bridge_ros2.arduino_node:main',
            'stadia_node = arduino_bridge_ros2.stadia_node:main',
        ],
    },
)
