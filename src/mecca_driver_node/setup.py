from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='steve phillips',
    maintainer_email='stvenmobile@gmail.com',
    description='ROS2 Node for Motor Driver Communication',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',          # Fixed
            'simple_serial = mecca_driver_node.simple_serial:main',                 # Fixed  
            'led_controller_node = mecca_driver_node.led_controller_node:main',     # Fixed
            'print_controller_parameters = scripts.print_controller_parameters:main',
        ],
    },
)
