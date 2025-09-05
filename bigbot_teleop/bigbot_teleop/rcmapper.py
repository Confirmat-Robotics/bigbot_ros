#!/usr/bin/env python3
import rclpy
import os # shutdown
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor as PD
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
from threading import Timer
from bigbot_interfaces.msg import PTZSetpoint
import subprocess
from enum import Enum
from bigbot_interfaces.srv import PredefinedPath
from bigbot_obstacle.timers import IdleTimer

class Mode(Enum):
    NO_MODE = 0,
    FREE_MODE = 1,
    AUTONOMOUS_MODE = 2,
    FOLLOWME_MODE = 3

class Toggle:
    def __init__(self, start_value):
        self.value = start_value
        self._changed_true = False
        self._changed_false = False
    def ChangedTrue(self, value = None):
        if value == True and self.value == False:
            self.value = value
            self._changed_true = True
            self._changed_false = False
            return True
        elif value == False and self.value == True:
            self.value = value
            self._changed_true = False
            self._changed_false = True
            return False
        elif value is None:
            return self._changed_true
        else:
            self.value = value
            self._changed_true = False
            self._changed_false = False
            return False
    def ChangedFalse(self, value = None):
        if value == False and self.value == True:
            self.value = value
            self._changed_true = False
            self._changed_false = True
            return True
        elif value == True and self.value == False:
            self.value = value
            self._changed_true = True
            self._changed_false = False
            return False
        elif value is None:
            return self._changed_false
        else:
            self.value = value
            self._changed_true = False
            self._changed_false = False
            return False

class RcMapper(Node):
    ''' Translation joy into Twist equivalent to speed-setpoint x,y,phi (x=forward) and camera PTZ movement'''

    def __init__(self):
        super().__init__('rcmapper')
        self.declare_parameter('MaxSpeed', 2.0, PD(description="Maximum horizontal speed in m/s"),) # max speed per motor = 2 m/s
        self.declare_parameter('MaxRotSpeed', 4.0, PD(description="Maximum rotational speed in rad/s")) 
        self.process = None
        self.temptimer = None
        self.mode = Mode.NO_MODE
        self.pubspeak = self.create_publisher(String, '/speak', 10)
        self.pubspeed = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subjoy = self.create_subscription(Joy, '/joy', self.listener_callback, 10)
        self.pubptz = self.create_publisher(PTZSetpoint, '/ptz_setpoint', 10)        
        self.pubvellimit = self.create_publisher(Float32, '/velocity_limit', 10) # ONLY FOR PATHFOLLOWER (NORMALLY COMES FROM LIDAR)

        ## LET OP: TESTING, DIT VERANDERD !! 
        self.pubfollowme = self.create_publisher(Bool, '/start_person_tracking', 10)
        self.pubpath = self.create_publisher(Bool, '/start_pathfollowing', 10)
        self.path_client = self.create_client(PredefinedPath, 'execute_path')

        self.output_directory = '/media/bigbot/KINGSTON' #logging #make this a parameter
        self.initrc()
	
    def initrc(self):
        self.startloggingbutton = Toggle(False)
        self.stoploggingbutton = Toggle(False)
        self.monitor_manual_free_mode = Toggle(False)
        self.monitor_manual_radius_mode = Toggle(False)
        self.monitor_manual_autonomous_mode = Toggle(False)
        self.monitor_stop_autonomous_mode = Toggle(False)
        self.startshutdownbutton = Toggle(False)
        self.stopshutdownbutton = Toggle(False)
        self.gopathbutton = Toggle(False)
        
    def maprc(self, inmsg):
        ''' Mapping of rc. '''
        currentlylogging = inmsg.buttons[0] == 1 # swA down
        startlogging = self.startloggingbutton.ChangedTrue(currentlylogging)  
        stoplogging = self.stoploggingbutton.ChangedTrue(currentlylogging == False)
        do_new_path = self.gopathbutton.ChangedTrue(inmsg.axes[5] == -1)
        
        shutdownpushed = inmsg.axes[5] == -1 # Key1 pressed
        startshutdown = self.startshutdownbutton.ChangedTrue(shutdownpushed)
        stopshutdown = self.stopshutdownbutton.ChangedTrue(shutdownpushed == False)

        if startshutdown == True: self.temptimer = IdleTimer(2)
        if stopshutdown == True: 
            try:
                if self.temptimer.istriggered() == True:
                    self._shutdown_now()
            except:
                pass

        goto_free_mode = self.monitor_manual_free_mode.ChangedTrue(inmsg.buttons[6] == 1)
        goto_autonomous_mode = self.monitor_manual_radius_mode.ChangedTrue(inmsg.buttons[4] == 1)
        goto_followme_mode = self.monitor_manual_autonomous_mode.ChangedTrue(inmsg.buttons[5] == 1)
        stopping_followme_mode = self.monitor_stop_autonomous_mode.ChangedTrue(inmsg.buttons[5] == 0)

        vxMAP = inmsg.axes[1] * -1 * self.get_parameter('MaxSpeed').value
        vrotMAP = inmsg.axes[0] * self.get_parameter('MaxRotSpeed').value
        #vx,vy,vrot similar mapping rc as controller

        # in radiusmode min(vx,vy) is speed_setpoint
        # and angle(vx,vy) is desired maxrotspeed
        vradiusMAP = max(abs(inmsg.axes[1]), abs(inmsg.axes[0])) * self.get_parameter('MaxSpeed').value
        vrotradiusMAP = np.arctan2(inmsg.axes[0] * 2, -1 * inmsg.axes[1]) #scaling 2
        if vrotradiusMAP > np.pi/2:
            vradiusMAP = -1*vradiusMAP
            vrotradiusMAP = np.pi - vrotradiusMAP
        if vrotradiusMAP < -np.pi/2:
            vradiusMAP = -1*vradiusMAP
            vrotradiusMAP = -np.pi + vrotradiusMAP 

        # This is not correct but not relevant at this point 
        ptzupaxis = inmsg.axes[2]  # [-1,1] this is position, not speed
        ptzhoraxis = -1 * inmsg.axes[3]  # [-1,1]
        ###########################switchoff = inmsg.axes[5]
        if inmsg.buttons[7] == 1:
            ptzzoomaxis = 1.0
        elif inmsg.buttons[8] == 1:
            ptzzoomaxis = -1.0
        else:
            ptzzoomaxis = 0.0

        return (vxMAP, vrotMAP, vradiusMAP, vrotradiusMAP, do_new_path, ptzupaxis, ptzhoraxis, ptzzoomaxis, startlogging, stoplogging, goto_free_mode, goto_autonomous_mode, goto_followme_mode, stopping_followme_mode)

    def listener_callback(self, inmsg):
        vx, vrot, vradiusMAP, vrotradiusMAP, do_new_path, ptzupaxis, ptzhoraxis, ptzzoomaxis, startlogging, stoplogging, \
            goto_free_mode, goto_autonomous_mode, goto_followme_mode, stopping_followme_mode = self.maprc(inmsg)
        
        if goto_free_mode == True: 
            self.speak("Go to manual mode")
            self.mode = Mode.FREE_MODE
            ## kerst2024 geen pub vellimit in free_mode
            msg = Float32( data = 100.0)
            self.pubvellimit.publish(msg)
        if goto_autonomous_mode == True: 
            self.speak("Go to autonomous mode") # "Go to manual mode with radius restriction"
            self.mode = Mode.AUTONOMOUS_MODE #AUTONOMOUS_MODE IS PATHFOLLOWER (only map maxvel along path, path setting via GUI)
            ## kerst2024  vellimit is van joystick, maar begin met stilstand
            msg = Float32( data = 0.0)
            self.pubvellimit.publish(msg)
        if goto_followme_mode == True:
            self.speak("Go to follow me mode")  # "Go to autonomous mode"
            self.pubspeed.publish(Twist())
            self._start_followme() # or make this a serive ? (to be decided)
            self.mode = Mode.FOLLOWME_MODE
        if stopping_followme_mode == True:
            self._stop_followme() # or make this a serive ? (to be decided)
        if do_new_path == True:
            self.speak("Oval left")
            self.get_logger().info("starting oval left")
            path_req = PredefinedPath.Request()
            path_req.path = PredefinedPath.Request.OVALLEFT
            future = self.path_client.call_async(path_req)
            future.add_done_callback(self.receive_path_response)

        #if self.mode == Mode.NO_MODE: #11 aug, uitgevinkt want PTZSetpoint en logging mag wel werken in NO_MODE
        #    return
        if self.mode == Mode.FREE_MODE:
            #if len(inmsg.axes) < 8:
            #    return None #joy_node not always produces nicely formatted /joy

            # Assign Twist
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vrot
            self.pubspeed.publish(msg)
        if self.mode == Mode.AUTONOMOUS_MODE: # DIT IS DE PATHFOLLOWER, LET NIET OP DE NAAMGEVING (NOG DOEN)
            msg = Float32( data = vx)
            self.pubvellimit.publish(msg)
            # # Assign Twist
            # minradius = 0.7 # [m]
            # msg = Twist()
            # #vradiusMAP, vrotradiusMAP
            # vrotmax = np.abs(vradiusMAP) / minradius
            # if vrotradiusMAP > 0:
            #     msg.angular.z = min(vrotradiusMAP, vrotmax)
            # else:
            #     msg.angular.z = max(vrotradiusMAP, -vrotmax)
            # msg.linear.x = vradiusMAP
            # self.pubspeed.publish(msg)

        ptzmsg = PTZSetpoint()
        ptzmsg.horizontalspeed = ptzhoraxis
        ptzmsg.verticalspeed = ptzupaxis # verticalposition , to do
        ptzmsg.zoomspeed = ptzzoomaxis
        self.pubptz.publish(ptzmsg)
        
        if startlogging == True: 
            self._start_recording()
        if stoplogging == True: 
            self._stop_recording()

        # LET OP: Met nieuwe conotroller moet dit worden geremapped

    def _shutdown_now(self):
        self.get_logger().info("shutdown now -h")
        os.system("shutdown now -h")

    def receive_path_response(self, future):
        msg = future.result() 
        self.get_logger().info('Path Response Received. SUCCESS = %s'  % (str(msg.success)))
        self.pubpath.publish(Bool(data = True)) # publishgo ahead follow path, now we know path is received
       
    def _start_recording(self):
        #command = ['ros2', 'bag', 'record', '-a', '-b', '20000000', '--storage', 'mcap', '--no-compression'] # Before ROS2 Jazzy
        command = ['ros2', 'bag', 'record', '-a', '--compression-mode', 'none', '--max-bag-size', '20000000', '--storage', 'mcap'] # ROS2 Jazzy
        try:
            self.process = subprocess.Popen(command, cwd = self.output_directory)
        except: 
            self.speak("Could not start recording")
        else:
            self.speak("Start recording")

    def _start_followme(self): self.pubfollowme.publish(Bool(data = True))
    def _stop_followme(self): self.pubfollowme.publish(Bool(data = False))
    def speak(self, mystring): self.pubspeak.publish(String( data = mystring))

    def _stop_recording(self):
        if self.process is not None:
            self.process.kill()
            self.speak("Stop recording")
        self.process = None
        
def main(args=None):
    rclpy.init(args=args)

    rcmapper = RcMapper()
    rclpy.spin(rcmapper)

    rcmapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
