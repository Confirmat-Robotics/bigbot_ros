#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from bigbot_common.transform import Transformer

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gps_sim_pkg = FindPackageShare('bigbot_gazebo')
    bigbot_cloud_pkg = FindPackageShare('bigbot_cloud')
    bigbot_gazebo_dir = get_package_share_directory('bigbot_gazebo')

    nav2params = os.path.join(bigbot_gazebo_dir, 'config', 'nav2_pursuit_control.yaml')
    configured_params = RewrittenYaml(
        source_file=nav2params, 
        root_key="", 
        param_rewrites={'use_sim_time': 'True'}, 
        convert_types=True
    )

    bigbot_description = os.path.join(get_package_share_directory('bigbot_description'))
    xacro_file = os.path.join(bigbot_description, 'urdf', 'bigbot.urdf')
    doc = Transformer(xacro_file)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both', # log
        parameters=[ {'robot_description': doc.geturdf()},
                     {'use_sim_time' : True},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    robot_state_publisher_delayed = TimerAction(
        period=2.0,  # Delay by 2 seconds
        actions=[robot_state_publisher]
    )
    
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bigbot_gazebo_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )

    odom_connect_node = Node(
        package='bigbot_gazebo',
        executable='odom_connect',
        name='odom_connect',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bigbot_gazebo_dir, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'params_file': configured_params,
            'autostart': 'True',
            'use_sim_time': 'True',
        }.items()
    )

    # Include: BigBot Cloud
    bigbot_cloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bigbot_cloud_pkg, 'launch', 'bigbot_fleetly.launch.py'
        ]))
    )

    # RViz config
    rviz_config_path = PathJoinSubstitution([
        gps_sim_pkg, 'rviz', 'nav.rviz'
    ])

    # Launch RViz
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    bigbot_cloud_launch_delayed = TimerAction(
        period=6.0,  # Delay by 2 seconds
        actions=[bigbot_cloud_launch]
    )
    ld = LaunchDescription([
        nav2_bringup,
        simulator,
        odom_connect_node,
        robot_state_publisher_delayed,
        bigbot_cloud_launch_delayed,
        rviz_launch,
    ])
    return ld

