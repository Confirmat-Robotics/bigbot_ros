import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor as PD
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from threading import Timer
from bigbot_interfaces.msg import PTZSetpoint

class JoyMapper(Node):
    ''' Translation joy into Twist equivalent to speed-setpoint x,y,phi (x=forward) and camera PTZ movement'''

    def __init__(self):
        super().__init__('joymapper')
        self.declare_parameter('MaxSpeed', 3.0, PD(description="Maximum horizontal speed in m/s"),)
        self.declare_parameter('MaxRotSpeed', 1.0, PD(description="Maximum rotational speed in rad/s")) 

        self.pubspeed = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subjoy = self.create_subscription(Joy, '/joy', self.listener_callback, 10)
        self.pubptz = self.create_publisher(PTZSetpoint, '/ptz_setpoint', 10)

    def listener_callback(self, inmsg):
        # dead-mans knobs 4 or 5 must be triggered, otherwise output will be 0
        vx = 0.0
        vrot = 0.0
        if len(inmsg.axes) < 8:
            return None #joy_node not always produces nicely formatted /joy
        leftdeadman = inmsg.buttons[4]
        rightdeadman = inmsg.buttons[5]
        deadman = leftdeadman + rightdeadman
        forwardaxis = inmsg.axes[1]
        rotationaxis = inmsg.axes[0]

        ptzupaxis = inmsg.axes[4] 
        ptzhoraxis = -1 * inmsg.axes[3]
        ptzzoomaxis = inmsg.axes[6] * 0.1 #hor knob cross
        #front_knobs_pushed_extensively = inmsg.axes[2] < -0.8 and inmsg.axes[5] < -0.8
        if deadman > 0:
            # if both knob 3 and 6 (on frontside big knobs) are both pushed, this means I want all wheels to the defined 0-value
            # (should be facing to front) 
            # this is done by steering vx,vrot=0 and vy = very small (so rotation is zero)
            vx = forwardaxis * self.get_parameter('MaxSpeed').value
            vrot = rotationaxis * self.get_parameter('MaxRotSpeed').value
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vrot
            self.pubspeed.publish(msg)

            ptzmsg = PTZSetpoint()
            ptzmsg.horizontalspeed = ptzhoraxis
            ptzmsg.verticalspeed = ptzupaxis
            ptzmsg.zoomspeed = ptzzoomaxis
            self.pubptz.publish(ptzmsg)

def main(args=None):
    rclpy.init(args=args)

    joymapper = JoyMapper()
    rclpy.spin(joymapper)

    joymapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()