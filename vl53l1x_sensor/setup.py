from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vl53l1x_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installs the launch files so you can use 'ros2 launch'
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Installs config files (like calibration or sensor params)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stvenmobile@gmail.com',
    description='VL53L1X Time-of-Flight sensor driver for Mecca',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'vl53l1x_node = vl53l1x_sensor.vl53l1x_node:main',
        ],
    },
)