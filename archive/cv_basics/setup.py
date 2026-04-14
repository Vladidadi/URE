from setuptools import find_packages, setup

package_name = 'cv_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vlad',
    maintainer_email='vmvitmaer@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	'img_publisher = cv_basics.webcam_pub:main',
        'img_subscriber = cv_basics.webcam_sub:main',
        ],
    },
)
