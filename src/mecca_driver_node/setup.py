from setuptools import setup
import os

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joy_teleop.yaml']),
        ('share/' + package_name + '/config', ['config/mecanum_controller.yaml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='stvenmobile@gmail.com',
    description='ROS2 Node for Motor Driver Communication',
    license='MIT',
    entry_points={
    'console_scripts': [
        'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',
        'simple_serial = mecca_driver_node.simple_serial:main',
        'led_controller_node = mecca_driver_node.led_controller_node:main',
    ],
},

)

