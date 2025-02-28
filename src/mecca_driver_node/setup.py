from setuptools import setup

package_name = 'mecca_driver_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='steve phillips',
    maintainer_email='stvenmobile@gmail.com',
    description='Motor driver node for Mecca',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecca_driver_node = mecca_driver_node.mecca_driver_node:main',
            'simple_serial = mecca_driver_node.serial_comm.simple_serial:main'
        ],
    },
)
