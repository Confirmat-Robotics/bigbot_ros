# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2025 Edward Hage - Confirmat Robotics
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from bigbot_bringup.transform import Transformer

def generate_launch_description():
    # Get the urdf file
    ROBOT_MODEL = 'bigbot'   
    sdf_file = os.path.join(
        get_package_share_directory('bigbot_gazebo'),
        'models', 
        ROBOT_MODEL,
        f'{ROBOT_MODEL}.sdf'
    )
    # xacro_file = os.path.join(
    #     get_package_share_directory('bigbot_description'),
    #     'urdf', 
    #     'bigbot_harmonic.urdf'
    # )
    # urdf_description = Transformer(xacro_file).geturdf()

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='3.5')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='2.0',
        description='Specify X position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-2.5',
        description='Specify Y position of the robot')
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.5',
        description='Specify Z position of the robot')
    
    # The 'ros_gz_sim create' node expects a file path for the '-file' argument.
    # To spawn from a string, use the '-string' argument instead of '-file'.
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT_MODEL,
            #'-string', urdf_description,  # Pass the URDF as a string
            '-file', sdf_file,  # Pass the SDF file path
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory('bigbot_gazebo'),
        'params',
        ROBOT_MODEL+'_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    #ld.add_action(start_gazebo_ros_image_bridge_cmd)

    return ld
