from setuptools import setup
import os
from glob import glob

package_name = 'mecca_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'vl53l1x_sensor'],
    zip_safe=True,
    maintainer='steve phillips',
    maintainer_email='stvenmobile@gmail.com',
    description='Launch package for Mecca robot',
    license='MIT',
    entry_points={
    'console_scripts': [
        'vl53l1x_node = vl53l1x_sensor.vl53l1x_node:main'],
    },
)

