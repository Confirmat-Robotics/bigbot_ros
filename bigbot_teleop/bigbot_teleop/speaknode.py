import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Speak(Node):

    def __init__(self):
        super().__init__('message_speaker')
        self.sub_ = self.create_subscription(String, "/speak", self.message_received, 10)
        ready_msg = String(data = "Robot is ready for service")
        self.message_received(ready_msg)

    def message_received(self,msg):
        os.system('espeak "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=None) 
    
    node = Speak()
    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
