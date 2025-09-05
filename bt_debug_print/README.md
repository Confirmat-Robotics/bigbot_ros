# DebugPrint BT Node

This is a simple Behavior Tree action node for ROS 2 (Jazzy) using `behaviortree_cpp` that prints debug information to the ROS log output.

## How to Build

```bash
cd ~/ros2_ws/src
# Copy or unzip this project here
cd ..
colcon build --packages-select bt_debug_print
source install/setup.bash
```

## How to Use in BT XML

```xml
<BehaviorTree>
  <Sequence>
    <DebugPrint message="Starting navigation to pose A"/>
    <FollowPath />
  </Sequence>
</BehaviorTree>
```

