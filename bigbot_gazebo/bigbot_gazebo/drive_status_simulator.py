import rclpy
from rclpy.node import Node
from bigbot_interfaces.msg import DriveFeedback
from std_msgs.msg import Header

class DriveStatusSimulator(Node):
    def __init__(self):
        super().__init__('drive_status_simulator')
        self.publisher_ = self.create_publisher(DriveFeedback, '/driveinfo', 10)
        self.timer = self.create_timer(2.0, self.publish_feedback)  # Publish every 2 seconds
        self.get_logger().info(f'Defined DriveFeedback status simulator')

    def publish_feedback(self):
        msg = DriveFeedback()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.temperature_drive = 45.0  # Example temperature for primary drive
        msg.temperature_drive_2nd = 40.0  # Example temperature for secondary drive
        msg.main_battery_voltage = 24.5  # Example main battery voltage
        msg.logic_battery_voltage = 12.0  # Example logic battery voltage
        msg.speed = [100, 120]  # Example encoder ticks per second for two motors
        msg.speedmpsec = [1.5, 1.8]  # Example speed in meters per second for two motors
        msg.dutycycle = [75.0, 80.0]  # Example duty cycle percentage for two motors
        msg.current = [10.0, 12.0]  # Example current in amperes for two motors
        msg.encoder = [5000, 5200]  # Example encoder counts for two motors
        msg.poserror = [5, 3]  # Example position error for two motors
        msg.speederror = [2, 1]  # Example speed error for two motors
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveStatusSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

