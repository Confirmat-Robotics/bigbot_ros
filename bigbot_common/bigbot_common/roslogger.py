#!/usr/bin/env python3
# RosLogger class for non-ros nodes
#
# This class initializes a ROS node and provides logging methods into the ROS ecosystem,
# so that it can be used in non-ROS Python scripts or modules.

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity


class RosLogger:
    _ros_initialized = False

    def __init__(self, name="ros_logger", level=LoggingSeverity.INFO):
        if not rclpy.ok():
            rclpy.init()
            RosLogger._ros_initialized = True
        self._node = Node(name)
        self._node.get_logger().set_level(level)

    def debug(self, msg):
        self._node.get_logger().debug(msg)

    def info(self, msg):
        self._node.get_logger().info(msg)

    def warning(self, msg):
        self._node.get_logger().warning(msg)

    def error(self, msg):
        self._node.get_logger().error(msg)

    def fatal(self, msg):
        self._node.get_logger().fatal(msg)

    def __del__(self):
        if hasattr(self, "_node") and self._node is not None:
            self._node.destroy_node()
        if RosLogger._ros_initialized and rclpy.ok():
            rclpy.shutdown()
