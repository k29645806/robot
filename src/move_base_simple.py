#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__=='__main__':
    rospy.init_node('send_command_client')
    client = actionlib.SimpleActionClient('move_base_simple', MoveBaseAction)
    rospy.loginfo('request has been sent!')
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 5.42
    goal.target_pose.pose.position.y = 10.74
    goal.target_pose.pose.orientation.w = 1.0
    ## Fill in the goal here
    client.send_goal(goal)
    ## Wait for five secs if no result stop
    client.wait_for_result(rospy.Duration.from_sec(5.0))
