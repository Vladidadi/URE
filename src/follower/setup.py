from setuptools import setup

package_name = 'follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/centroid_publisher_launch.py', 'launch/follower_launch.py', 'launch/nav_to_pose_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR NAME',
    maintainer_email='YOUR EMAIL',
    description='Centroid follower ROS 2 package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'centroid_follower = follower.centroid_follower:main',
            'centroid_publisher = follower.centroid_publisher:main',
            'nav_to_pose = follower.nav_to_pose:main'
        ],
    },
)
