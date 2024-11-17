from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='WRO perception system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_to_bgr_converter = robot_perception.rgb_to_bgr_converter:main',
            'lidar2image_node = robot_perception.lidar2image_node:main',
            'teleop_to_robot_controller = robot_perception.teleop_to_robot_controller:main',
            'lap_counter_node = robot_perception.lap_counter_node:main',
        ],
    },
)
