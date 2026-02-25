from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.0.1',
    # find_packages handles the nested folder structure
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='steve@todo.todo',
    description='Driver node for Mecca robot STM32 bridge',
    license='Apache-2.0',
    entry_points={
    'console_scripts': [
        'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',
        'led_controller_node = mecca_driver_node.led_controller_node:main'
    ],
    },
)
