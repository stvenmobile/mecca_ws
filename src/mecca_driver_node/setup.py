from setuptools import setup
from setuptools import find_packages

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joy_teleop.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Mecanum wheel driver node for ROS2 Jazzy',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',
            'simple_serial = mecca_driver_node.serial_comm.simple_serial:main'
        ],
    },
)
