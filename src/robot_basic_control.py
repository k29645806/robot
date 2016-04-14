#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, String
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import radians,cos,sin
from SerialDataGateway import SerialDataGateway

'''
Received sensor messages from ST board
'''

class Robot(object):
    def __init__(self):
        rospy.init_node('robot_basic_control')
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))
        self.robotSerial = SerialDataGateway(port, baud_rate, self._handle_received_line)
        rospy.Subscriber("cmd_vel", Twist, self.pub_cmd_vel)
        self.x, self.y, self.yaw, self.v, self.w = 0.0, 0.0, 0.0, 0.0, 0.0    # start point
#------------------------------simulation-------------------------------------
    def pub_cmd_vel(self,data,b=1.0):
        v, w = data.linear.x, data.angular.z
        # cvt center v and w to vl and vr
        vr = v + b/2*w
        vl = v - b/2*w
        cmd = "spd %d %d"%(vl,vr)   # 0~480
        #print cmd
        #self.robotSerial.Write(cmd)

        rospy.loginfo("x:%.2f y:%.2f theta:%.2f v:%.2f w:%.2f"%(self.x,self.y,self.yaw,self.v,self.w))
#-------------------------------------------------------------------------------
    def _handle_received_line(self):
        pass




    def start(self):
        rate = rospy.Rate(30.0)
        rospy.loginfo("Robot Engine Start")
        #self.startSerialPort()
        while not rospy.is_shutdown():
            self.broadcastTF()
            self.publishOdometry()
            self.publishLaserScan()
            rate.sleep()


if __name__ == '__main__':
    kai = Robot()
    kai.start()
