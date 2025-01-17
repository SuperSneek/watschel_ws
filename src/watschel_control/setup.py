from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'watschel_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niklas Conen',
    maintainer_email='niklas.conen@student.kit.edu',
    description='ROS2 Control configuration for the watschel robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testMovements = watschel_control.testMovements:main',
            'node = watschel_control.node_backup:main',
        ],
    },
)
