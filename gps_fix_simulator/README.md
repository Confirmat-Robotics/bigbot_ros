# GPS Fix Simulator Plugin for Gazebo Harmonic

This plugin simulates GPS+orientation sensor and publishes relevant transforms for robot models in Gazebo Harmonic. It is designed to work with ROS 2 and supports both model-scoped and world-scoped usage.

This is used to simulate a Fixposition Vision-RTK device which gives GPS location and orientation of a FP_POI (point of interest) wrt a FP_ENU (which is defined at startup of the device once a 'GPS fix' is obtained).

## Features

- Publishes GPS-based transforms (ENU/ECEF/map/odom) for a robot model.
- ROS 2 integration for TF broadcasting.
- Supports multiple robots when attached at model scope.

## Usage

### 1. Attach in Robot Model SDF (Recommended)

Add the plugin block inside your robot's `<model>` tag in the SDF file (e.g., `bigbot.sdf`):

```xml
<plugin filename="libgps_fix_simulator.so" name="gps_fix_simulator">
  <link_name>bigbot_gps</link_name>
  <child_frame>FP_POI</child_frame>
  <parent_frame>FP_ECEF</parent_frame>
  <enu_frame>FP_ENU</enu_frame>
</plugin>
```

- This will automatically attach the plugin to the correct robot instance, regardless of the model name used at spawn time.

### 2. Attach in World SDF (Advanced, Single Robot Only)

Add the plugin block inside your `<world>` tag in the world SDF file, and specify the target model name:

```xml
<plugin filename="libgps_fix_simulator.so" name="gps_fix_simulator">
  <model_name>bigbot</model_name>
  <link_name>bigbot_gps</link_name>
  <child_frame>FP_POI</child_frame>
  <parent_frame>FP_ECEF</parent_frame>
  <enu_frame>FP_ENU</enu_frame>
</plugin>
```

- Replace `<model_name>` with the name of the robot model you want to attach to (must match the spawned model name).
- **Note:** If you do not specify `<model_name>`, the plugin will not attach to any model when loaded at world scope.

## Build Instructions

1. **Clone the repository:**
   ```bash
   git clone <your_repo_url> gps_fix_simulator
   cd gps_fix_simulator
   ```

2. **Build with colcon:**
   ```bash
   colcon build --packages-select gps_fix_simulator
   ```

3. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```

4. **Ensure Gazebo can find the plugin:**
   - Add the plugin library path to `GAZEBO_PLUGIN_PATH` or install to a standard location.

## Notes

- For multi-robot simulations, always attach the plugin at model scope (inside each robot's SDF).
- If using world scope, you must specify `<model_name>`, and only one robot will be supported per plugin instance.
- **This plugin is used in the `bigbot_gazebo` simulation package.**


