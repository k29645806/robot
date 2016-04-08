#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, String
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import radians
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
#-------------------------------------------------------------------------------
    def _handle_received_line(self, line):
        """
        This will run every time a line is received over the serial port (USB)
        from the Propeller board and will send the data to the correct function.
        """
        self.serialReadData = str(line)

    def _handle_robot_command(self, cmd):
        rate = rospy.Rate(10.0)
        if (self.available == True):
            cmdData = map(int, cmd.data.split(",")) # chnage string to integer
            output = []
            output.append(0xc9) # start byte
            for joint_angle in cmdData:     # J1,J2,J3,J4,J5,J6
                output.append(joint_angle)

            check_sum = 0
            for num in output[1:]:    # check sum
                check_sum ^= num

            output.append(check_sum)
            output.append(0xca) # end byte

            string = ''
            for i in output:
                string += struct.pack('!B',i)
            self.robotSerial.Write(string)
            rate.sleep()

        else:
            rospy.error("robot is in action")

    def startSerialPort(self):
        rospy.loginfo("Serial Data Gateway starting . . .")
        self.robotSerial.Start()
        rospy.Subscriber("cmd_vel", String, self._handle_robot_command)
#-------------------------------------------------------------------------------
    def broadcastTF(self,data=None):
        x, y, theta = 1, 1, radians(5)
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
        self.tfBroadcaster.sendTransform((0.2, 0, 0.5),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_laser",
                         "base_link")

    def publishOdometry(self,data=None):
        odom = Odometry()
        odom.header.seq = 0
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = 1
        odom.pose.pose.position.y = 1
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0 ,0.0 ,radians(5))
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odom.twist.twist.linear.x = 0
        odom.twist.twist.angular.z = 0
        self.odomPublisher.publish(odom)

    def publishLaserScan(self,data=None):
        scan = LaserScan()
        scan.header.seq = 1
        scan.header.stamp = rospy.Time.now()
        num_readings = 100
        laser_frequency = 40
        scan.header.frame_id = "base_laser"
        scan.angle_min = radians(-90)
        scan.angle_max = radians(90)
        scan.angle_increment = radians(180) / num_readings
        scan.time_increment = (1 / laser_frequency) / (num_readings)
        scan.scan_time = 0.05
        scan.range_min = 0.5
        scan.range_max = 6
        scan.ranges = [3]*num_readings
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
