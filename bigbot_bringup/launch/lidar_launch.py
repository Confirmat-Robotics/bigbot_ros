#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    sick_scan_launch_description = get_package_share_directory('bigbot_bringup')
    launch_file_path = os.path.join(sick_scan_launch_description, 'bigbot_sick.launch')
    node_arguments=[launch_file_path]
    
    # append optional commandline arguments in name:=value syntax
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
    
    node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=node_arguments
    )

    ld.add_action(node)
    return ld
