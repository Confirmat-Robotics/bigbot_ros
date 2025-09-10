#!/usr/bin/env python3
from setuptools import find_packages, setup

package_name = 'bigbot_cloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bigbot_cloud.launch.py']),
        ('share/' + package_name + '/launch', ['launch/bigbot_fleetly.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edward Hage',
    maintainer_email='edward@confirmatrobotics.com',
    description='Cloud (Fleetly) connection robot status (including services)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        'bigbot_cloud = bigbot_cloud.cloud_interface:main',
        'path_conversion_node = bigbot_cloud.path_conversion_node:main',
        'path_conversion_local = bigbot_cloud.path_conversion_local:main',
        'cloud_interface = bigbot_cloud.cloud_interface_toLL:main',
        'mqtt_systemd_monitor = bigbot_cloud.mqtt_systemd_monitor:main',
        ],
    },
)
