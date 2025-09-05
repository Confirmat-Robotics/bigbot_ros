#!/usr/bin/env python3
import paho.mqtt.client as paho
from paho import mqtt
import json
import os

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import NavSatFix
from bigbot_interfaces.msg import DriveFeedback
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from bigbot_common.projections import Projections
from robot_localization.srv import ToLL
from std_msgs.msg import String
from enum import Enum


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

    def publish_location(self, lat, lng, angle):
        mydict = dict()
        mydict['lat'] = lat
        mydict['lng'] = lng
        mydict['angle'] = angle
        payload = json.dumps(mydict)
        self.client.publish(self.locationtopic, payload)

    def publish_drivefeedback(self, message : DriveFeedback):
        mydict = dict()
        mydict['current'] = message.current
        mydict['speed'] = message.speed
        mydict['logic_batery_voltage'] = message.logic_battery_voltage
        mydict['main_batery_voltage'] = message.main_battery_voltage
        payload = json.dumps(mydict)
        self.client.publish(self.drivefeedbacktopic, payload)

class CloudInterface(Node):
    
    class LocationSource(Enum):
        NAVSAT_ODOM = 'navsat_odom'
        TF = 'tf'

    def __init__(self):
        super().__init__('cloud_interface')
        self.declare_parameter('poi_frame', 'base_link'); #'FP_POI')
        self.poi_frame = self.get_parameter('poi_frame').get_parameter_value().string_value

        self.declare_parameter('navsat_mapref', 'map')
        self.map = self.get_parameter('navsat_mapref').get_parameter_value().string_value

        self.declare_parameter('ecef_frame', 'FP_ECEF')
        self.ecef = self.get_parameter('ecef_frame').get_parameter_value().string_value

        self.declare_parameter('location_source', CloudInterface.LocationSource.TF.value)
        location_source_value = self.get_parameter('location_source').get_parameter_value().string_value

        if location_source_value in [source.value for source in CloudInterface.LocationSource]:
            self.location_source = location_source_value
        else:
            valid_options = [source.value for source in CloudInterface.LocationSource]
            self.get_logger().warn(
                f"Invalid location_source parameter: {location_source_value}. "
                f"Valid options are: {valid_options}. Defaulting to 'navsat_odom'."
            )
            self.location_source = CloudInterface.LocationSource.NAVSAT_ODOM.value

        self.mqttpub = MqttPublisher()

        self.sub2_ = self.create_subscription(DriveFeedback, "/driveinfo", self.mqttpub.publish_drivefeedback, 10)
        self.sub3_ = self.create_subscription(String, "/status", self.mqttpub.publish_status, 10)
        # Only subscribe or create timer based on location_source

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.location_source == CloudInterface.LocationSource.NAVSAT_ODOM.value:
            # NavSat service client
            self.to_ll_client = self.create_client(ToLL, '/toLL')
            self.get_logger().info('Waiting for NavSat toLL service...')
            while not self.to_ll_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('toLL service not available, waiting...')
            self._timer2 = self.create_timer(6.0, self.lookup_llm)
            self.get_logger().info('NavSat toLL service up ...')
        elif self.location_source == CloudInterface.LocationSource.TF.value:
            self._timer = self.create_timer(2.0, self.lookup_poi_frame_pose)
            # Only subscribe to status/drivefeedback for completeness
        else:
            self.get_logger().warn(f"Unknown location_source parameter: {self.location_source}")

        
    def call_LL(self, x, y, z):
        """
        Makes a NavSat toLL request and returns the response.
        Not working, because the service returns nothing.
        Use call_LL_cmdline instead.
        """
        try:
            to_ll_request = ToLL.Request()
            to_ll_request.map_point.x = x
            to_ll_request.map_point.y = y
            to_ll_request.map_point.z = z
            #self.get_logger().info(f"Request: {to_ll_request}")

            future = self.to_ll_client.call_async(to_ll_request)
            #self.get_logger().info("Waiting for future to complete...")
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.done():
                #self.get_logger().info("Future completed.")
                return future.result()
            else:
                self.get_logger().warn("Service call toLL did not complete.")
                return None
        except Exception as e:
            self.get_logger().warn(f"Error during NavSat toLL request: {e}")
            return None

    def call_LL_cmdline(self, x, y, z):
        """
        Makes a NavSat toLL request using a command-line service call and returns the response.
        Handles timeout errors gracefully.
        """
        try:
            # Construct the command-line call
            command = f'ros2 service call /toLL robot_localization/srv/ToLL "{{map_point: {{x: {x}, y: {y}, z: {z}}}}}"'
            self.get_logger().info(f"Executing command: {command}")

            # Execute the command and capture the output
            process = os.popen(command)
            result = process.read()
            process.close()

            if "timeout" in result:
                self.get_logger().warn("Service call to /toLL timed out. No response received.")
                return None

            # Parse the result string (assuming the output contains JSON-like data)
            # Adjust parsing logic based on the actual format of the service response
            if "ll_point" in result:
                lat_start = result.find("latitude=") + len("latitude=")
                lon_start = result.find("longitude=") + len("longitude=")
                lat_end = result.find(",", lat_start)
                lon_end = result.find(",", lon_start)

                latitude = float(result[lat_start:lat_end].strip())
                longitude = float(result[lon_start:lon_end].strip())

                return {"latitude": latitude, "longitude": longitude}
            else:
                self.get_logger().warn("Failed to parse command output.")
                return None
        except Exception as e:
            self.get_logger().warn(f"Error during cmdline NavSat toLL request: {e}")
            return None

    def lookup_llm(self):
        try:
            # Get base_link transform from map
            transform = self.tf_buffer.lookup_transform(
                self.map,
                self.poi_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            trans = transform.transform.translation
            q = transform.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            _, _, yaw_in_degrees = r.as_euler('xyz', degrees=True)
            # Use call_LL to get GPS coordinates
            # to_ll_response = self.call_LL(trans.x, trans.y, trans.z)
            # if to_ll_response:
            #     latlng = to_ll_response.ll_point
            #     self.mqttpub.publish_location(latlng.latitude, latlng.longitude, yaw_in_degrees)
            latlng = self.call_LL_cmdline(trans.x, trans.y, trans.z)
            self.get_logger().info("Publishing location on mqtt")
            self.mqttpub.publish_location(latlng['latitude'], latlng['longitude'], yaw_in_degrees)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform {self.map}->{self.poi_frame}: {e}")
            return None

    
    def lookup_poi_frame_pose(self):
        """
        Looks up the transform from FP_ECEF to self.poi_frame.
        Returns (x, y, z, yaw_deg) or None if not available.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.ecef, self.poi_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            lat, lon, _ = Projections.ecef2gnss(x, y, z)

            # Determine rotation (yaw) in degrees from quaternion
            q = trans.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            _, _, rotation_in_degrees  = r.as_euler('xyz', degrees=True)
            #print("in lookup_poi_frame_pose: ", lat, lon, rotation_in_degrees)
            self.mqttpub.publish_location(lat, lon, rotation_in_degrees)
        except Exception as e:
            self.get_logger().warn(f"Could not get transform {self.ecef}->{self.poi_frame}: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)

    ci = CloudInterface()
    executor = rclpy.executors.MultiThreadedExecutor()  # Use MultiThreadedExecutor
    executor.add_node(ci)

    try:
        executor.spin()  # Spin the executor
    except KeyboardInterrupt:
        pass
    finally:
        ci.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()