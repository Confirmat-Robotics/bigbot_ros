import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('bigbot_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='15.0',  #2.0
        description='Initial X position of the robot'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='30.0',  #-2.5
        description='Initial Y position of the robot'
    )
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='1.0', #drop from height
        description='Initial Z position of the robot'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='sonoma_raceway.world',
        description='Name of the Gazebo world file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        FindPackageShare('bigbot_gazebo'),
        'worlds',
        world_file
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Added --headless-rendering
        launch_arguments={
            'gz_args': ['-r -s -v2 --headless-rendering ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('bigbot_gazebo'),
            'models'
        )
    )

    ld = LaunchDescription()

    # Add the argument declarations to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_world_cmd)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(spawn_robot_cmd)
    return ld
