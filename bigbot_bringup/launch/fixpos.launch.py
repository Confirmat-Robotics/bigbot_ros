from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="fixposition_driver_ros2",
        description="Node name",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value="fixpos_config.yaml",
        description="Configuration file to use (relative to this package's 'config' dir)",
    )

    # Build config path: <share>/bigbot_bringup/config/<config>
    config_path = PathJoinSubstitution(
        [FindPackageShare("bigbot_bringup"), "config", LaunchConfiguration("config")]
    )

    # Build per-node log-level argument: "<node_name>:=info"
    log_level_arg = PythonExpression(
        ['"', LaunchConfiguration("node_name"), '" + ":=info"']
    )

    node = Node(
        package="fixposition_driver_ros2",
        executable="fixposition_driver_ros2_exec",
        name=LaunchConfiguration("node_name"),
        output="screen",
        respawn=True,
        respawn_delay=5.0,
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level_arg],
    )

    return LaunchDescription([
        node_name_arg,
        config_arg,
        node,
    ])