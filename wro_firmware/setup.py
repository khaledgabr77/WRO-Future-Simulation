from setuptools import setup
import os
from glob import glob

package_name = 'wro_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'firmware'), glob('firmware/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khaled Gabr',
    maintainer_email='khaledgabr77@gmail.com',
    description='WRO firmware package with Arduino and Python files.',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_name = wro_firmware.your_script:main',
        ],
    },
)
