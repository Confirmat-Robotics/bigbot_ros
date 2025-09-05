#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from bigbot_common.transform import Transformer
from launch_ros.actions import Node

def generate_launch_description():

    bigbot_description = os.path.join(get_package_share_directory('bigbot_description'))
    
    world = 'myworld'

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world}.items()
             )

    xacro_file = os.path.join(bigbot_description, 'urdf', 'bigbot.urdf')
    doc = Transformer(xacro_file)
    params = {'robot_description': doc.geturdf()}
    sdf_file = 'curmodel.sdf'
    doc.savesdf(sdf_file)
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'Bigbot',
                                   #'-topic', 'robot_description',],
                                   '-file', sdf_file,],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_bigbot_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'bigbot_velocity_controller'],
        output='screen'
    )

    load_camera_pan_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'camera_pan_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_bigbot_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_bigbot_velocity_controller,
                on_exit=[load_camera_pan_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
