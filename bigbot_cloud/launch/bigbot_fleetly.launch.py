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
        executable='path_conversion_local',
        output='screen'  # dont use name becaus 2 nodes are started (and should not have the same name)
    )

services_via_mqtt = Node(
        package='bigbot_cloud',
        executable='mqtt_systemd_monitor',
        output='screen'  
    )

def generate_launch_description():
    return LaunchDescription([
        cloud_interface,
        path_from_mqtt,
        services_via_mqtt
    ])
