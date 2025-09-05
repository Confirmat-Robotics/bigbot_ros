#!/usr/bin/env python3
import os
import signal
import json
import time

import paho.mqtt.client as paho
from paho import mqtt

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geographic_msgs.msg import GeoPose
from bigbot_common.transform import Transformer

class MqttPathMonitor:
    """
    MQTT client that listens for geopath messages and triggers waypoint processing in the ROS node.
    Uses loop_start() so no extra thread is needed.
    """
    def __init__(self, robot_name, on_path_received):
        broker = os.environ.get("MQTT_BROKER")
        username = os.environ.get("MQTT_USERNAME")
        password = os.environ.get("MQTT_PASSWORD")
        self.client = paho.Client(paho.CallbackAPIVersion.VERSION2)
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(username, password)
        self.client.connect(broker, 8883)
        self.client.on_message = self.on_message
        self.client.subscribe(f"{robot_name}/geopath", qos=0)
        self.on_path_received = on_path_received

    def on_message(self, client, userdata, msg):
        try:
            path = json.loads(msg.payload.decode())
            if isinstance(path, list):
                print(f"[MQTT] Received {len(path)} waypoints")
                self.on_path_received(path)
            else:
                print("[MQTT] Received geopath is not a list")
        except Exception as e:
            print(f"[MQTT] Failed to process geopath: {e}")

    def start(self):
        self.client.loop_start()  # Non-blocking, runs MQTT in its own thread

class RosWaypointNode(Node):
    """
    ROS node that receives waypoints and commands the robot to follow them using BasicNavigator.
    """
    def __init__(self):
        super().__init__('path_conversion_node')  # Default name, launch file can override
        self.declare_parameter('altitude', 0.0)
        self.declare_parameter('align_angle', True)
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.align_angle = self.get_parameter('align_angle').get_parameter_value().bool_value

        self.navigator = BasicNavigator("basic_navigator")
        self.get_logger().info("Nav2 Simple commando active...")
        self._pending_waypoints = None
        self._timer = self.create_timer(0.5, self._process_pending_waypoints)

    def set_pending_waypoints(self, path):
        self._pending_waypoints = path

    def _process_pending_waypoints(self):
        if self._pending_waypoints is None:
            return
        path = self._pending_waypoints
        self._pending_waypoints = None

        geoposes = []
        for i, wp in enumerate(path):
            geopose = GeoPose()
            geopose.position.latitude = wp['lat']
            geopose.position.longitude = wp['lng']
            geopose.position.altitude = self.altitude
            geopose.orientation.x = 0.0
            geopose.orientation.y = 0.0
            geopose.orientation.z = 0.0
            geopose.orientation.w = 1.0
            # If align_angle is True and not last waypoint, you could compute orientation here
            geoposes.append(geopose)

        self.get_logger().info(f"Sending {len(geoposes)} GPS waypoints to BasicNavigator")
        self.navigator.followGpsWaypoints(geoposes)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        self.get_logger().info("Waypoints completed successfully")

def main():
    rclpy.init()
    node = RosWaypointNode()

    # Get robot_name parameter (default: robot11)
    robot_name = "SanFran_Waffle"
    if node.has_parameter('robot_name'):
        robot_name = node.get_parameter('robot_name').get_parameter_value().string_value

    # Start MQTT monitor (no extra thread needed with loop_start)
    mqtt_monitor = MqttPathMonitor(robot_name, node.set_pending_waypoints)
    mqtt_monitor.start()

    # Handle shutdown gracefully
    def shutdown_handler(signum, frame):
        node.get_logger().info("Shutting down...")
        mqtt_monitor.client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()
        exit(0)
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown - always stop MQTT client
        mqtt_monitor.client.loop_stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
