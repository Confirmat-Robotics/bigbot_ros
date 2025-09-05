#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # This is via joy_linux and should be working on all Pis, but not on Nuc
        #Node(
        #    package='joy_linux',
        #    executable='joy_linux_node',
        #    output='screen',
        #    emulate_tty=True,
        #    remappings=[ ('dev', '/dev/input/js1')],
        #    ), 
        # This is via joy joy_node (not working on all Pis)
        # Beware: mapping of joystick axis is different than joy_linux so do not
        # use without changing joy_speed_mapper.py
        Node( 
             package='joy',
             executable='joy_node',
             output='screen',
             emulate_tty=True,
             remappings=[ ('dev', '/dev/input/js1')],
             ), 
        Node(
            package='bigbot_teleop',
            executable='joymapper',
            output='screen',
            emulate_tty=True,
            parameters= [{ 'MaxSpeed' : 3.0, 'MaxRotSpeed' : 0.22 }],
            )
    ])