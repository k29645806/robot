#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def sender():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('send_command')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 12
        goal.pose.position.y = 17
        goal.pose.orientation.w = 1.0
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
