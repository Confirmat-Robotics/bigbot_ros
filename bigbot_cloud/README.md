# bigbot_cloud

`bigbot_cloud` is a ROS 2 package that provides MQTT-based cloud connectivity for mobile robots, enabling integration with the Fleetly app ([fleetly.confirmat.nl](https://fleetly.confirmat.nl)). It allows remote monitoring of robot status, control of systemd services, and sending navigation paths to the robot via MQTT, which are then executed using the Nav2 navigation stack.

---

## Features

- **Fleetly Cloud Integration:** Seamless connection to the Fleetly app for real-time robot monitoring and control.
- **Status Publishing:** Publishes robot status, location, and battery information to Fleetly via MQTT.
- **Service Control:** Allows remote control (start/stop/restart) of systemd services on the robot through MQTT messages.
- **Path Following:** Receives navigation paths over MQTT and forwards them to Nav2 using the `NavigateThroughPoses` action client.
- **Flexible Location Reporting:** Supports publishing robot location using either TF transforms or the `robot_localization` ToLL service.

---

## Nodes

### 1. `cloud_interface`
Publishes robot status, location, and battery voltage to Fleetly via MQTT. Subscribes to relevant ROS topics and periodically publishes updates.

### 2. `cloud_interface_toLL`
Alternative interface that can use the `robot_localization` ToLL service for accurate GPS reporting.

### 3. `mqtt_systemd_monitor`
Monitors and controls systemd services on the robot via MQTT, allowing remote management from Fleetly.

### 4. `path_conversion_node` / `path_conversion_local`
Listens for path commands from Fleetly over MQTT and sends them to Nav2 for execution. Supports different orientation strategies for waypoints.

---

## Installation

1. **Clone the repository:**
   ```sh
   cd ~/ros2_ws/src
   git clone <this-repo-url> bigbot_cloud
   ```

2. **Install dependencies:**
   ```sh
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```sh
   colcon build --packages-select bigbot_cloud
   source install/setup.bash
   ```

---

## Configuration

- Edit the parameters in the launch files or YAML config files to set:
  - MQTT broker address and credentials
  - Fleetly topic names
  - Robot-specific settings (frame IDs, battery topic, etc.)

---

## Usage

### Launching

Use the provided launch files to start the cloud interface and path conversion nodes:

```sh
ros2 launch bigbot_cloud bigbot_cloud.launch.py
```

You can also launch individual nodes as needed, e.g.:

```sh
ros2 run bigbot_cloud cloud_interface
ros2 run bigbot_cloud mqtt_systemd_monitor
```

---

## Example: Sending a Path from Fleetly

1. In the Fleetly app, send a navigation path to the robot.
2. The `path_conversion_node` receives the path over MQTT and forwards it to Nav2.
3. The robot executes the path using the Nav2 navigation stack.


---

## Contributing

Pull requests are welcome! Please open an issue first to discuss your proposed changes.

---

## License

[MIT](LICENSE)

---

## Contact

For support or questions, contact [Confirmat Robotics](mailto:support@confirmatrobotics.com).