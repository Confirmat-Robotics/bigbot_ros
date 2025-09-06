#!/usr/bin/env python3
import os
import signal
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Transform, PoseStamped, TransformStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header, String

import paho.mqtt.client as paho
from paho import mqtt

import rclpy
from rclpy.node import Node
from bigbot_common.transform import Transformer
from bigbot_common.projections import Projections
import enum
import tf2_ros
from rclpy.duration import Duration

class OrientationStrategy(enum.Enum):
    TOWARDS_NEXT = 1
    PAIRED = 2 

class Nav2ErrorCode(enum.Enum):
    UNKNOWN = 0
    INVALID_GOAL = 100
    FAILED_TO_PLAN = 101
    FAILED_TO_CONTROL = 102
    FAILED_TO_MAKE_PROGRESS = 103
    TASK_CANCELED = 104
    GOAL_TOLERANCE_VIOLATED = 105
    TASK_FAILED = 106
    def __str__(self): return self.name.replace('_', ' ').title()

class GeneratePoses:
    
    def determine_orientation(next_pose, pose):
        ''' get quaternion from pose to next_pose
            next_pose and pose are lists of [x,y,z] 
        '''
        v = np.array(next_pose) - np.array(pose)
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-8:
            # No direction, return identity quaternion
            return [0.0, 0.0, 0.0, 1.0]
        # Full 3D orientation: align x-axis with direction vector
        # Use align_vectors for a more concise solution
        v_unit = v / v_norm
        x_axis = np.array([1.0, 0.0, 0.0])
        rotation = R.align_vectors([v_unit], [x_axis])[0]
        return rotation.as_quat().tolist()
    
    def make_pose(x, y, z, qx, qy, qz, qw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose
    
    def make_tf(x, y, z, qx, qy, qz, qw):
        tf = Transform()
        tf.translation.x = float(x)
        tf.translation.y = float(y)
        tf.translation.z = float(z)
        tf.rotation.x = float(qx)
        tf.rotation.y = float(qy)
        tf.rotation.z = float(qz)
        tf.rotation.w = float(qw)
        return tf
    
    @staticmethod
    def generate_poses_from_latlng(latlng_array, map_ref_ECEF : Transform, squash_altitude : bool = True, orientation_strategy : OrientationStrategy = OrientationStrategy.TOWARDS_NEXT):
        """ Generate a list of PoseStamped from a list of lat,lng coordinates
            map_ref_ECEF is the ENU0 frame in ECEF coordinates (tf)
            squash_altitude: if True, the altitude of the poses will be set to 0.0
            orientation_strategy: OrientationStrategy enum value
        """
        _, _, altitude_ref = Projections.ecef2gnss(map_ref_ECEF.translation.x, map_ref_ECEF.translation.y, map_ref_ECEF.translation.z)
        poses_rel = list()        
        for loc in latlng_array:
            pose = Projections.transform_geopose_to_ENUframe(map_ref_ECEF, loc["lat"], loc["lng"], altitude_ref)
            poses_rel.append([pose.position.x, pose.position.y, pose.position.z])
        poses_stamped = []
        if len(poses_rel) == 0:
            return None
        if len(poses_rel) == 1:
            pose = poses_rel[0]
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.0 if squash_altitude else pose[2]
            poses_stamped.append(pose_stamped)
            return poses_stamped
        for i, pose in enumerate(poses_rel):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.0 if squash_altitude else pose[2]
            pose_stamped.pose.orientation.w = 1.0
            if orientation_strategy == OrientationStrategy.TOWARDS_NEXT:
                if i+1 < len(poses_rel):
                    next_pose = poses_rel[i+1] 
                    quat = GeneratePoses.determine_orientation(next_pose, pose)
                    pose_stamped.pose.orientation.x = quat[0]
                    pose_stamped.pose.orientation.y = quat[1]
                    pose_stamped.pose.orientation.z = quat[2]
                    pose_stamped.pose.orientation.w = quat[3]
                else:
                    # Last pose: copy previous orientation or set identity
                    if i > 0:
                        pose_stamped.pose.orientation = poses_stamped[-1].pose.orientation
                    else:
                        pose_stamped.pose.orientation.w = 1.0
            elif orientation_strategy == OrientationStrategy.PAIRED:
                # For PAIRED, orient in pairs (1->2, 3->4, etc.)
                if i % 2 == 0 and i+1 < len(poses_rel):
                    next_pose = poses_rel[i+1]
                    quat = GeneratePoses.determine_orientation(next_pose, pose)
                    pose_stamped.pose.orientation.x = quat[0]
                    pose_stamped.pose.orientation.y = quat[1]
                    pose_stamped.pose.orientation.z = quat[2]
                    pose_stamped.pose.orientation.w = quat[3]
                elif i % 2 == 1:
                    # Copy orientation from previous
                    pose_stamped.pose.orientation = poses_stamped[-1].pose.orientation
                else:
                    pose_stamped.pose.orientation.w = 1.0
            else:
                pose_stamped.pose.orientation.w = 1.0
            poses_stamped.append(pose_stamped)
        return poses_stamped

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

    def on_message(self, client, userdata, msg): #msg.topic, msg.payload
        try:
            data = json.loads(msg.payload.decode())
            if not isinstance(data, dict):
                print("[MQTT] Unsupported geopath payload: expected an object with 'strategy' and 'points'")
                return

            strategy_str = data.get("strategy")
            if not isinstance(strategy_str, str):
                print("[MQTT] Missing or invalid 'strategy' in payload")
                return

            # Safe enum lookup by name (returns None if not found)
            strategy = OrientationStrategy.__members__.get(strategy_str.upper())
            if strategy is None:
                print(f"[MQTT] Invalid 'strategy': {strategy_str}. Allowed: {', '.join(OrientationStrategy.__members__.keys())}")
                return

            points = data.get("points")
            if not isinstance(points, list):
                print("[MQTT] Missing or invalid 'points' list in payload")
                return

            print(f"[MQTT] Received {len(points)} waypoints, strategy={strategy.name}")
            self.on_path_received(points, strategy)
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
        self.declare_parameter('ecef_frame', 'FP_ECEF')
        self.declare_parameter('map_frame', 'map')
        self.frame_ref_ECEF = self.get_parameter('ecef_frame').get_parameter_value().string_value
        self.frame_ref_map = self.get_parameter('map_frame').get_parameter_value().string_value

        self._action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.pub_status = self.create_publisher(String, '/status', 10)
        self.get_logger().info("Nav2 NavigateThroughPoses action client initialized.")
        self.get_logger().info("Nav2 Simple commando active...")
        self._pending = None
        self._timer = self.create_timer(0.5, self._process_pending_waypoints)
        self._throttle_timer = self.create_timer(2, self._allow_processing)  # Throttle timer to avoid too frequent processing
        self._allow_feedback = False
        self.ecef_MAP_TF = None
        # get  the map reference ECEF from the broadcasted transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF Buffer and Listener initialized.")
        try:
            self.ecef_MAP_TF = self.tf_buffer.lookup_transform(
                self.frame_ref_ECEF, 
                self.frame_ref_map, 
                rclpy.time.Time(), 
                timeout=Duration(seconds=3.0)
            )
            self.get_logger().info(f"Map reference ECEF transform found")
        except Exception as e:
            self.get_logger().error(f"Failed to get map reference ECEF transform: {e}")
            self._retry_counter = 0
            self._retry_timer = self.create_timer(7.0, self._retry_ecef_tf)
    
    def _allow_processing(self):
        self._allow_feedback = True

    def _retry_ecef_tf(self):
        try:
            # Attempt to get the transform
            trans = self.tf_buffer.lookup_transform(self.frame_ref_ECEF, 
                                                    self.frame_ref_map,
                                                    rclpy.time.Time(),
                                                    timeout=Duration(seconds=3.0))
            # If successful, stop the timer and proceed
            self._retry_timer.cancel()
            self.get_logger().info("Transform found!")
            self.ecef_MAP_TF = trans
        except Exception as e:
            self._retry_counter += 1
            self.get_logger().info(f"Retry {self._retry_counter}: Transform not found, will retry.")

    def set_pending_waypoints(self, points, strategy: OrientationStrategy):
        # Store both the waypoint list and the orientation strategy
        self._pending = (points, strategy)

    def _process_pending_waypoints(self):
        if self._pending is None:
            return
        latlng_array, orientation_strategy = self._pending
        self._pending = None

        poses = GeneratePoses.generate_poses_from_latlng(
            latlng_array,
            self.ecef_MAP_TF.transform,
            squash_altitude=True,
            orientation_strategy=orientation_strategy
        )
        if poses is None:
            self.get_logger().error("No valid poses generated from the provided waypoints")
            return
        
        header = Header()
        header.frame_id = self.frame_ref_map
        header.stamp = self.get_clock().now().to_msg()
        for i, _ in enumerate(poses):
            poses[i].header = header

        goal_msg = NavigateThroughPoses.Goal(poses = poses, behavior_tree = "")

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        error_code = result.result.error_code
        if result.status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
             status_text = "✅ Navigation finished succesfull."
             self.get_logger().info(status_text)
        elif result.status == GoalStatus.STATUS_ABORTED:
            error_string = str(Nav2ErrorCode(error_code))
            status_text = f"Result code: {error_code} ({error_string})"
            self.get_logger().info(status_text)
            if hasattr(result, 'total_time_sec'):
                self.get_logger().info(f"Total navigation time: {result.result.total_time_sec:.2f} seconds")
                status_text += f", time: {result.result.total_time_sec:.2f} sec"
            if hasattr(result, 'number_of_recoveries'):
                if result.number_of_recoveries > 0:
                    self.get_logger().info(f"Number of recoveries: {result.result.number_of_recoveries}")
                    status_text += f", recoveries: {result.result.number_of_recoveries}"
        self.pub_status.publish(String(data=status_text))

    def _feedback_callback(self, feedback_msg):
        if self._allow_feedback == False:
            return
        self._allow_feedback = False
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        self.pub_status.publish(
            String(data=f"Feedback received: Distance remaining: {distance_remaining:.2f} m")
        )


def main():
    rclpy.init()
    node = RosWaypointNode()

    robot_name = os.environ.get("ROBOT_NAME")
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
