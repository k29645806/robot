#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class Navigation(object):
    def __init__(self):
        rospy.init_node('publish_nav_cmd')
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def setGoal(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = 10.0
        ps.pose.position.y = 10.0
        q = quaternion_from_euler(0.0 ,0.0 ,0.0)
        ps.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.pub.publish(ps)


if __name__ == '__main__':
    try:
        nav = Navigation()
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            nav.setGoal()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error rospy")
    finally:
        rospy.loginfo("Publish_nav_cmd stopped")
