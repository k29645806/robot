#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, String
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


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
#-------------------------------------------------------------------------------
    def broadcastTF(self,data=None):
        # translation, orientation, time, child, parent
        self.tfBroadcaster.sendTransform((3, 3, 0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "odom",
                         "map")
        self.tfBroadcaster.sendTransform((0, 0, 0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "base_footprint")
        self.tfBroadcaster.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "camera_link",
                         "base_link")
        self.tfBroadcaster.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_laser",
                         "base_link")

    def start(self):
        rate = rospy.Rate(20.0)
        rospy.loginfo("Robot Engine Start")
        while not rospy.is_shutdown():
            self.broadcastTF()
            rate.sleep()

if __name__ == '__main__':
    kai = Robot()
    kai.start()
