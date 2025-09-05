# bigbot_gazebo

A ROS 2 package for simulating GPS-based autonomous navigation with Bigbot in Gazebo Harmonic, including Nav2 integration, GPS localization, and cloud connectivity.

## Overview

`bigbot_gazebo` provides a complete simulation environment for Bigbot robots, supporting GPS-based navigation, sensor simulation, and integration with cloud services. The package includes launch files for Gazebo Harmonic, Nav2 navigation stack, robot localization, and cloud connectivity via the `bigbot_cloud` package.

## Features

- **Bigbot simulation in Gazebo Harmonic** with custom SDF and URDF models
- **GPS-based localization** using simulated GPS sensors and transforms
- **Nav2 navigation stack** for autonomous waypoint following and path execution
- **Dual EKF localization** (local odometry + global GPS)
- **Cloud integration** for remote monitoring and path commands (via `bigbot_cloud`)
- **RViz visualization** for navigation and sensor data
- **Drive status simulation** for testing robot feedback topics

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2 packages (`nav2_bringup`, `nav2_common`, etc.)
- `bigbot_description` and `bigbot_bringup` packages
- `gps_fix_simulator` Gazebo plugin (for GPS transforms)
- `bigbot_cloud` package (optional, for cloud integration)

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> bigbot_gazebo
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select bigbot_gazebo
   source install/setup.bash
   ```

## Launch Files

### Main Launch Files

- **`simulation.launch.py`**: Launches Gazebo Harmonic, Nav2 stack, robot state publisher, odometry transforms, cloud integration, and RViz.
- **`minsim.launch.py`**: Minimal simulation launch for testing, includes drive status simulator.
- **`gazebo.launch.py` / `mingazebo.launch.py`**: Launches Gazebo Harmonic with configurable world and robot spawn options.
- **`spawn_robot.launch.py`**: Spawns the Bigbot model in Gazebo and sets up ROS-Gazebo bridges.
- **`nav2.launch.py`**: Launches Nav2 navigation stack with configurable parameters.

### Supporting Nodes

- **`odom_connect.py`**: Publishes static transforms for GPS and odometry frames
- **`drive_status_simulator.py`**: Simulates drive feedback messages for testing.

## Robot Model

- **SDF Model**: `models/bigbot/bigbot.sdf`
- **URDF Model**: Provided via `bigbot_description`
- **Sensors Simulated**:
  - GPS (NavSat)
  - LiDAR
  - IMU
  - Odometry
  - Camera (optional)

## Configuration

- **Nav2 Parameters**: `config/nav2_pursuit_control.yaml`
- **Localization Parameters**: `config/dual_ekf_navsat_params.yaml`
- **Bridge Configuration**: `params/bigbot_bridge.yaml`
- **RViz Config**: `rviz/nav.rviz`

## Usage Examples

### Full Simulation

```bash
ros2 launch bigbot_gazebo simulation.launch.py
```

### Minimal Simulation

```bash
ros2 launch bigbot_gazebo minsim.launch.py
```

### Custom World

```bash
ros2 launch bigbot_gazebo simulation.launch.py world_file:=your_world.world
```

## Cloud Integration

- The package can be integrated with `bigbot_cloud` for remote monitoring, path commands, and status reporting.
- Cloud launch is included in the main launch files and can be customized.

## Troubleshooting

- Ensure all dependencies are built and sourced.
- Check Gazebo plugin paths for `gps_fix_simulator`.
- Use `--ros-args --log-level debug` for detailed logs.

## License

Apache-2.0 License
- `models/turtlebot3_waffle_gps/model.sdf`: Gazebo simulation model
- Adapted from standard TurtleBot3 Waffle with added GPS sensor

## Configuration

### Nav2 Parameters
- `config/nav2_waffle_params.yaml`: Main Nav2 configuration
- `config/dual_ekf_navsat_params.yaml`: Robot localization parameters
- `config/nav2_no_map_params.yaml`: Map-free navigation settings

### Bridge Configuration
- `params/turtlebot3_waffle_gps_bridge.yaml`: ROS-Gazebo topic bridging

## Usage Examples

### Basic Simulation
```bash
# Start complete simulation
ros2 launch nav2_gps_simulations bringup_all.launch.py

# Start only Gazebo simulation
ros2 launch nav2_gps_simulations turtlebot3_gps.launch.py

# Start with custom world
ros2 launch nav2_gps_simulations bringup_all.launch.py world_file:=custom_world.world
```

### Send GPS Waypoints
```bash
# Example: Send GPS coordinates to the robot
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

## Integration with Cloud Services

This package is designed to work with external cloud applications that can:
- Define navigation waypoints and paths
- Send goals to the BasicNavigator
- Monitor navigation progress
- Provide mission management capabilities

The cloud integration interface is through standard Nav2 topics and services.

