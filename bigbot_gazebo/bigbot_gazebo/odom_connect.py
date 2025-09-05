#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdomConnectNode(Node):
    def __init__(self):
        super().__init__('odom_connect_node')
        
        # Create transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create timer for 10 Hz broadcasting
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
        self.get_logger().info("OdomConnect node started - broadcasting transforms at 10 Hz")

    def broadcast_transforms(self):
        now = self.get_clock().now()
        
        # Create FP_ENU0 -> map transform (identity)
        fp_enu0_to_map = TransformStamped()
        fp_enu0_to_map.header.stamp = now.to_msg()
        fp_enu0_to_map.header.frame_id = 'FP_ENU0'
        fp_enu0_to_map.child_frame_id = 'map'
        fp_enu0_to_map.transform.translation.x = 0.0
        fp_enu0_to_map.transform.translation.y = 0.0
        fp_enu0_to_map.transform.translation.z = 0.0
        fp_enu0_to_map.transform.rotation.x = 0.0
        fp_enu0_to_map.transform.rotation.y = 0.0
        fp_enu0_to_map.transform.rotation.z = 0.0
        fp_enu0_to_map.transform.rotation.w = 1.0
        
        # Create odom -> base_link transform (identity)
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = now.to_msg()
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'
        odom_to_base_link.transform.translation.x = 0.0
        odom_to_base_link.transform.translation.y = 0.0
        odom_to_base_link.transform.translation.z = 0.0
        odom_to_base_link.transform.rotation.x = 0.0
        odom_to_base_link.transform.rotation.y = 0.0
        odom_to_base_link.transform.rotation.z = 0.0
        odom_to_base_link.transform.rotation.w = 1.0
        
        # Broadcast both transforms
        self.tf_broadcaster.sendTransform([fp_enu0_to_map, odom_to_base_link])

def main(args=None):
    rclpy.init(args=args)
    node = OdomConnectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
