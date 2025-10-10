# from setuptools import setup
# import os
# from glob import glob
#
# package_name = 'mecanum_hardware_interface'
#
# setup(
#     name=package_name,
#     version='1.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
#         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Your Name',
#     maintainer_email='your_email@example.com',
#     description='Hardware interface for mecanum wheel robot via serial',
#     license='MIT',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'mecanum_hardware_interface = mecanum_hardware_interface.mecanum_hardware_interface:main',
#         ],
#     },
# )

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mecanum_hardware_interface'

setup(
    name=package_name,  # This gets converted to 'mecanum-hardware-interface' by setuptools
    version='1.0.0',
    packages=find_packages(exclude=['test']),  # Use find_packages instead
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 hardware interface for mecanum robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This must match: executable_name = package.module:function
            'mecanum_hardware_interface = mecanum_hardware_interface.mecanum_hardware_interface:main',
        ],
    },
)
