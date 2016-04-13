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
1. Received sensor messages from ST board
2. Publish odometry information
3. Publish sensor information (Laser, sonar)
4. Publish robot transform configuration
'''

class Robot(object):
    def __init__(self):
        rospy.init_node('robot_setup')
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.odomPublisher = rospy.Publisher('odom',Odometry, queue_size=10)
        self.laserScanPublisher = rospy.Publisher('laserScan',LaserScan, queue_size=10)
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))
        self.robotSerial = SerialDataGateway(port, baud_rate, self._handle_received_line)
        rospy.Subscriber("cmd_vel", Twist, self.pub_cmd_vel)
        self.x, self.y, self.yaw, self.v, self.w = 0.0, 0.0, 0.0, 0.0, 0.0    # start point
#------------------------------handle serial command--------------------------
    def _handle_received_line(self,data):
        # ['x:1.00', 'y:2.00', 'heading:3.00', 'omega:55.00', 'v:66.00']
        try:
            robotState = data.split(',')
            robotState = [item[item.index(':')+1:] for item in robotState]
            robotState = map(float,robotState)
            self.x, self.y, self.yaw, self.w, self.v = robotState
        except:
            pass
    def pub_cmd_vel(self,data,b=0.8):
        v, w = data.linear.x, data.angular.z
        # cvt center v and w to vl and vr
        vr = v + b/2*w
        vl = v - b/2*w
        cmd = "spd %d %d"%(vl,vr)   # 0~480
        #print cmd
        #self.robotSerial.Write(cmd)

#------------------------------simulation-------------------------------------
    def simulation(self,data):
        # Simulation odometry!
        pass
        '''
        v, w, dt = data.linear.x, data.angular.z, 0.1
        self.x += (v*dt)*cos(self.yaw)
        self.y += (v*dt)*sin(self.yaw)
        self.yaw += w*dt
        self.v = v
        self.w = w
        rospy.loginfo("x:%.2f y:%.2f theta:%.2f v:%.2f w:%.2f"%(self.x,self.y,self.yaw,self.v,self.w))
        '''

#-------------------------------------------------------------------------------
    def broadcastTF(self,data=None):
        x, y, theta = self.x, self.y, self.yaw
        # translation, orientation, time, child, parent
        self.tfBroadcaster.sendTransform((0, 0, 0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "odom",
                         "/map")
        self.tfBroadcaster.sendTransform((x, y, 0),
                         quaternion_from_euler(0, 0, theta),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        self.tfBroadcaster.sendTransform((0.0, 0.0, 0.5),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_laser",
                         "base_link")

    def publishOdometry(self,data=None):
        odom = Odometry()
        odom.header.seq = 0
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0 ,0.0 ,self.yaw)
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odomPublisher.publish(odom)

    def publishLaserScan(self,data=None):
        scan = LaserScan()
        scan.header.seq = 1
        scan.header.stamp = rospy.Time.now()
        num_readings = 100
        laser_frequency = 40
        scan.header.frame_id = "base_laser"
        scan.angle_min = radians(-30)
        scan.angle_max = radians(30)
        scan.angle_increment = radians(60) / num_readings
        scan.time_increment = (1 / laser_frequency) / (num_readings)
        scan.range_min = 0.5
        scan.range_max = 6
        scan.ranges = [5]*num_readings
        self.laserScanPublisher.publish(scan)
    def logInfo(self):
        rospy.loginfo("x:%.2f y:%.2f theta:%.2f v:%.2f w:%.2f"%(self.x,self.y,self.yaw,self.v,self.w))

    def start(self):
        rate = rospy.Rate(10.0)
        rospy.loginfo("Robot Engine Start")
        self.robotSerial.Start()
        while not rospy.is_shutdown():
            try:
                self.broadcastTF()
                self.publishOdometry()
                self.publishLaserScan()
                self.logInfo()
                rate.sleep()
            finally:
                self.robotSerial.Stop()
    def testScript(self):
        rate = rospy.Rate(10.0)
        rospy.loginfo("Robot Engine Start")
        #self.robotSerial.Start()
        #self.robotSerial.Write("start")
        while not rospy.is_shutdown():
            try:
                #self.broadcastTF()
                #self.publishOdometry()
                #self.publishLaserScan()
                #self.logInfo()
                rate.sleep()
            finally:
                #self.robotSerial.Write("stop")
                #self.robotSerial.Stop()
                pass

if __name__ == '__main__':
    kai = Robot()
    kai.testScript()
