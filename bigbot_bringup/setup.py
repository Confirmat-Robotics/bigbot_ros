#!/usr/bin/env python3
from setuptools import setup
import os
from glob import glob

package_name = 'bigbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name), glob('launch/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edward Hage',
    maintainer_email='edward@confirmatrobotics.com',
    description='Bigbot bringup and transform utilities for simulation and real robot.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        ],
    },
)
