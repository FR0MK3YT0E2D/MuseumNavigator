#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def callback(msg):
    goal = MoveBaseGoal()
    goal.target_pose = msg
    client.send_goal(goal)

rospy.init_node("goal_bridge")
client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
client.wait_for_server()
rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
rospy.spin()
