from setuptools import setup

package_name = 'motor_pwm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi4',
    maintainer_email='pi4@example.com',
    description='Motor PWM control with gpiozero',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_pwm_node = motor_pwm_pkg.motor_pwm_node:main',
            'dual_motor_node = motor_pwm_pkg.dual_motor_node:main',

        ],
    },
)
