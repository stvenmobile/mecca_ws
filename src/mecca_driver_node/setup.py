from setuptools import setup
from setuptools import find_packages

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    py_modules=[
        'mecca_driver_node.ws2812'  # Add ws2812.py as a module
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steve Phillips',
    maintainer_email='stvenmobile@gmail.com',
    description='Mecca Driver Node for controlling motors and LEDs',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',
            'simple_serial = mecca_driver_node.serial_comm.simple_serial:main',
            'led_controller_node = mecca_driver_node.led_controller_node:main' 
        ],
    },
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joy_teleop.yaml'])
    ],
    
)
