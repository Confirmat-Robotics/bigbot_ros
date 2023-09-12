import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    gps_config_directory = os.path.join(get_package_share_directory('ublox_gps'), 'config')
    gps_params = os.path.join(get_package_share_directory('bigbot_bringup'), 'zed_f9p.yaml')
    #gps_params = os.path.join(gps_config_directory, 'zed_f9p.yaml') #TODO: BRING THIS YAML TO BIGBOT_BRINGUP
    navsat_params = os.path.join(get_package_share_directory('bigbot_bringup'), 'navsat_transform.yaml')

    ublox_gps_node = Node(package='ublox_gps',
                         executable='ublox_gps_node',
                         output='both',
                         parameters=[gps_params])
    # produceert /fix stream (sensor_msgs/msg/NatSatFix)
    gps_exit_event = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=ublox_gps_node,
             on_exit=[EmitEvent(event=launch.events.Shutdown())]
         ))

    wit_node = Node(package="wit_node", 
                    executable="wit_node",
                    parameters=[{
                        "port": "/dev/ttyIMU",
                        "baud_rate": 115200, 
                        "frame_id": "/imu_link",
                        "publish_hz": 100.0
                    }],
                )
    # produceert /imu

    navsat_transform_node = Node(package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform_node',
                output='screen',
                parameters=[navsat_params],
                remappings=[('gps/fix', 'fix'),('imu', 'imu')]
        )
    
    # cmdline call robot_localization/SetDaumService

    return launch.LaunchDescription([   ublox_gps_node,
                                        #wit_node,
                                        #gps_exit_event,
                                        #navsat_transform_node,
                                     ])
