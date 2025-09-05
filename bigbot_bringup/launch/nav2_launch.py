#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
#from bigbot_bringup.mapgenerator import MapGenerator # generate empty map

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    #python_commander_dir = get_package_share_directory('nav2_simple_commander')
    bigbot_control_dir = get_package_share_directory('bigbot_control')
    mapfile = os.path.join(bigbot_control_dir, 'config', 'empty_map.yaml')
    #generated_map = MapGenerator(500,500) # must not be destroyed before launch
    
    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': mapfile }.items())

    # start the demo autonomy task
    # demo_cmd = Node(
    #     package='nav2_simple_commander',
    #     executable='example_waypoint_follower',
    #     emulate_tty=True,
    #     output='screen')

    ld = LaunchDescription()
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    #ld.add_action(demo_cmd) #start apart op (anders begint die meteen)
    return ld
