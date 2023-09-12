from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node 
from launch.conditions import LaunchConfigurationEquals 

def generate_launch_description():

    rcreceiver = "FS-iA6B iBus RC receiver"
    controller = 'Microsoft X-Box 360 pad'

    declare_joystick = DeclareLaunchArgument( name='joystick', default_value="rc", description='controller or rc')

    joy_node_controller = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters= [ { "dev_name": controller}],
        emulate_tty=True,
        condition=LaunchConfigurationEquals('joystick', "controller")
    )
    joy_node_rcreceiver = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters= [ { "dev_name": rcreceiver}],
        emulate_tty=True,
        condition=LaunchConfigurationEquals('joystick', "rc")
    )
    # dit zit al in bigbot_bringup (deze node niet met bigbot_bringup gebruiken)
    exe_input_attach = ExecuteProcess(
        cmd=['sudo inputattach --baud 115200 --fsia6b /dev/ttyRC'],
        name='joystick_attach',
        output='screen',
        respawn=True,
        condition=LaunchConfigurationEquals('joystick', "rc"),
        shell=True
    )

    teleop_mapper = Node(
        package='bigbot_teleop',
        executable='rcmapper',
        output='screen',
        emulate_tty=True,
        parameters= [{ 'MaxSpeed' : 3.5, 'MaxRotSpeed' : 17.0 }],
        )

    ld = LaunchDescription()
    ld.add_action(declare_joystick)
    # ld.add_action(exe_input_attach)
    ld.add_action(joy_node_controller)
    ld.add_action(joy_node_rcreceiver)
    ld.add_action(teleop_mapper)
    return ld
    
