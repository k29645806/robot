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
        #port = rospy.get_param("~port", "/dev/ttyACM0")
        #baud_rate = int(rospy.get_param("~baudRate", 115200))
        #self.robotSerial = SerialDataGateway(port, baud_rate, self._handle_received_line)
        rospy.Subscriber("cmd_vel", Twist, self.simulation)
        self.sim_x, self.sim_y, self.sim_yaw, self.sim_v, self.sim_w = 0.0, 0.0, 0.0, 0.0, 0.0    # start point
#------------------------------simulation-------------------------------------
    def simulation(self,data):
        v, w, dt = data.linear.x, data.angular.z, 0.1
        self.sim_x += (v*dt)*cos(w*dt)
        self.sim_y += (v*dt)*sin(w*dt)
        self.sim_yaw += w*dt
        self.sim_v = v
        self.sim_w = w

#-------------------------------------------------------------------------------
    def broadcastTF(self,data=None):
        x, y, theta = self.sim_x, self.sim_y, self.sim_yaw
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
        odom.pose.pose.position.x = self.sim_x
        odom.pose.pose.position.y = self.sim_y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0 ,0.0 ,self.sim_yaw)
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odom.twist.twist.linear.x = self.sim_v
        odom.twist.twist.angular.z = self.sim_w
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

    def start(self):
        rate = rospy.Rate(10.0)
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
