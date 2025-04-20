from setuptools import setup
import os
from glob import glob

package_name = 'ld06_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eitoldas',
    maintainer_email='eitoldas@gmail.com',
    description='LD06 LIDAR publisher for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'ld06_node = ld06_lidar.lidar_node:main',
        ],
    },
)
