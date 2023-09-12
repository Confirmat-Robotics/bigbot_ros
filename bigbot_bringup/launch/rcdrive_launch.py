from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess ( cmd=['sudo inputattach --baud 115200 --fsia6b /dev/ttyRC'],
            name='joystick_attach',
            output='screen',
            respawn=True,
            shell=True),
        Node( 
             package='joy',
             executable='joy_node',
             output='screen',
             emulate_tty=True,
             parameters= [{'device_name' : 'FS-iA6B iBus RC receiver'}],
             #remappings=[ ('dev', '/dev/input/js0')],
             ), 
        Node(
            package='bigbot_teleop',
            executable='rcmapper',
            output='screen',
            emulate_tty=True,
            parameters= [{ 'MaxSpeed' : 2.0, 'MaxRotSpeed' : 5.0 }],
            ),
        Node(
           package='bigbot_base',
           executable='drivernode',
           output='screen',
           respawn=True,
           ),
        Node(
            package='bigbot_teleop',
            executable='speaknode',
            output='screen',
            respawn=True,
            ),
    ])
