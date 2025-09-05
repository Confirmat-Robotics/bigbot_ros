#!/usr/bin/env python3
from setuptools import setup
import os
from glob import glob

package_name = 'bigbot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edward Hage',
    maintainer_email='edward@confirmatrobotics.com',
    description='Teleoperation package for BigBot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'joymapper = bigbot_teleop.joymapper:main',
            'rcmapper = bigbot_teleop.rcmapper:main',
            'speaknode = bigbot_teleop.speaknode:main'
        ],
    },
)
