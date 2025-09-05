#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

cloud_interface = Node(
        package='bigbot_cloud',
        executable='bigbot_cloud',
        name='cloud_interface_node',
        output='screen',
        parameters=[{
            'poi_frame': 'base_link',
            'ecef_frame': 'FP_ECEF',
            'map_frame': 'map'
        }]
    )

path_from_mqtt = Node(
        package='bigbot_cloud',
        executable='path_conversion_node',
        output='screen'
    )


def generate_launch_description():
    return LaunchDescription([
        cloud_interface,
        path_from_mqtt,
    ])
