from setuptools import find_packages, setup

package_name = 'face_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stvenmobile@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
    'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'face_display_node = face_display.face_display_node:main',
        ],
    },

)
