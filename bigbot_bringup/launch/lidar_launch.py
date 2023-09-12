from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


import os
import sys
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sick_scan_launch_description = get_package_share_directory('bigbot_bringup')
    launch_file_path = os.path.join(sick_scan_launch_description, 'bigbot_sick_lms.launch')
    node_arguments=[launch_file_path]
    
    # append optional commandline arguments in name:=value syntax
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
   
    sickscan_node = Node(
        package='sick_scan',
        executable='sick_generic_caller',
        name='lidar',
        output='screen',
        arguments=node_arguments
    )
    # bigbot_obstacle obstacle
    obstacle_detection_node = Node(
        package='bigbot_obstacle',
        executable='bigbot_obstacle',
        name='obstacle',
        output='screen'
    )
    followme_node = Node(
        package='bigbot_followme',
        executable='followme',
        name='followme',
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(sickscan_node) 
    ld.add_action(obstacle_detection_node)
    ld.add_action(followme_node)
    return ld

