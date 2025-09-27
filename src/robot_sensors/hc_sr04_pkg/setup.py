from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'hc_sr04_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi4',
    maintainer_email='pi4@example.com',
    description='HC-SR04 Ultrasonic Sensor ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hc_sr04_node = hc_sr04_pkg.hc_sr04_node:main',
        ],
    },
)
