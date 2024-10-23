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
            # 'yolo2pose_node = smart_track.yolo2pose_node:main',
        ],
    },
)