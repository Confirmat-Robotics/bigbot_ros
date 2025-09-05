#!/usr/bin/env python3
import time
import paho.mqtt.client as paho
from paho import mqtt
import threading
import json
import os

from datetime import datetime
import uuid # use poi_frame MAC as identifier

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import NavSatFix
from bigbot_interfaces.msg import DriveFeedback
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from bigbot_common.projections import Projections
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class MqttPublisher:
    def __init__(self):
        broker = os.environ.get("MQTT_BROKER")
        username = os.environ.get("MQTT_USERNAME")
        password = os.environ.get("MQTT_PASSWORD")
        robot_name = os.environ.get("ROBOT_NAME", "SanFran_Waffle")

        self.drivefeedbacktopic = f"{robot_name}/livestatus" # not used yet
        self.voltagetopic = f"{robot_name}/voltage"
        self.locationtopic = f"{robot_name}/location"
        self.statustopic = f"{robot_name}/status"

        self.client = paho.Client(paho.CallbackAPIVersion.VERSION2)
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(username, password)
        self.client.connect(broker, 8883)
        self.client.loop_start()
        self.mylocation = None

    def publish_status(self, message : String): self.client.publish(self.statustopic, message.data)
    def publish_drivefeedback(self, message : DriveFeedback): self.client.publish(self.voltagetopic, message.main_battery_voltage)

class CloudInterface(Node):
    
    def __init__(self):
        super().__init__('cloud_interface')
        self.declare_parameter('poi_frame', 'FP_POI')
        self.poi_frame = self.get_parameter('poi_frame').get_parameter_value().string_value
        self.declare_parameter('ecef_frame', 'FP_ECEF')
        self.ecef = self.get_parameter('ecef_frame').get_parameter_value().string_value
        self.declare_parameter('map_frame', 'map')
        self.map = self.get_parameter('map_frame').get_parameter_value().string_value
        
        self.mqttpub = MqttPublisher()

        self.sub2_ = self.create_subscription(DriveFeedback, "/driveinfo", self.mqttpub.publish_drivefeedback, 10)
        self.sub3_ = self.create_subscription(String, "/status", self.mqttpub.publish_status, 10)
        
        self._timer = self.create_timer(2.0, self.lookup_poi_frame_pose)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lookup_poi_frame_pose(self):
        rotation_in_degrees = self.lookup_angle()
        latlon_result = self.lookup_latlon()
        
        if rotation_in_degrees is not None and latlon_result is not None:
            lat, lon, altitude = latlon_result
            mydict = {
                'lat': lat,
                'lng': lon,
                'altitude': altitude,
                'angle': rotation_in_degrees
            }
            self.mqttpub.client.publish(self.mqttpub.locationtopic, json.dumps(mydict))
            #self.get_logger().info(f"Published location: {lat}, {lon}, {altitude}, angle: {rotation_in_degrees}")
        else:
            self.get_logger().warn("Could not get all required transforms for location publishing.")    
    
    def lookup_angle(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map, self.poi_frame, rclpy.time.Time())
            rot = trans.transform.rotation
            r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            _, _, rotation_in_degrees = r.as_euler('xyz', degrees=True)
            #print("in lookup_angle: ", rotation_in_degrees)
            return rotation_in_degrees
        except Exception as e:
            self.get_logger().warn(f"Could not get transform {self.map}->{self.poi_frame}: {e}")
            return None 

    def lookup_latlon(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.ecef, self.poi_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            lat, lon, altitude = Projections.ecef2gnss(x, y, z)

            #print("in lookup_poi_frame_pose: ", lat, lon, rotation_in_degrees)
            return lat, lon, altitude
        except Exception as e:
            self.get_logger().warn(f"Could not get transform {self.ecef}->{self.poi_frame}: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)

    ci = CloudInterface()
    rclpy.spin(ci)

    ci.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
