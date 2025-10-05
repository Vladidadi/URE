from setuptools import setup
import os
from glob import glob

package_name = 'motor_pwm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # ADD THIS LINE
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Motor PWM package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_motor_node = motor_pwm_pkg.dual_motor_node:main',
            'motor_pwm_node = motor_pwm_pkg.motor_pwm_node:main',
        ],
    },
)
