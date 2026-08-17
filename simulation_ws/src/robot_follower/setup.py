from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_follower'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'supervision==0.27.0'],
    zip_safe=True, maintainer='josemsotov', license='MIT',
    entry_points={'console_scripts': [
        'follower_node = robot_follower.follower_node:main',
        'open_palm_node = robot_follower.open_palm_node:main',
        'field_supervisor = robot_follower.field_supervisor:main',
    ]},
)
