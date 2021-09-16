#!/usr/bin/env python

# Task Space Imports
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from scipy.spatial.transform import Rotation as R
# MoveIt Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('python_moveit_interface_test', anonymous=True)
rate = rospy.Rate(0.5)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "coborg_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

# move robot arm to tucked position (joint goal)
# -1.91986, -2.37365, -2.51327
tucked_pos = group.get_current_joint_values()
tucked_pos[0] = -1.91986
tucked_pos[1] = -2.37365
tucked_pos[2] = -2.51327
# tucked_pos[0] = 0.0
# tucked_pos[1] = -1.91986
# tucked_pos[2] = -2.37365
# tucked_pos[3] = -2.51327
group.go(tucked_pos, wait=True)
group.stop()
rospy.sleep(2)
# move robot arm to ready position (position goal)
# 0.353297, 0.11838, 0.342266
ready_pos = geometry_msgs.msg.Pose()
# ready_pos.orientation.w = 1.0
# ready_pos.position.x = 0.431809
# ready_pos.position.y = 0.153884
# ready_pos.position.z = 0.618589
# ready_pos.position.x = -0.00229401
# ready_pos.position.y = -0.0859048
# ready_pos.position.z = 0.114537
ready_pos.position.x = 0.0
ready_pos.position.y = 0.0
ready_pos.position.z = 0.0
group.set_planner_id("RRTConnect")
group.set_planning_time(1.0)
group.set_position_target([ready_pos.position.x, ready_pos.position.y, ready_pos.position.z], group.get_end_effector_link())
plan = group.go(wait=True)
print(plan)
group.stop()
group.clear_pose_targets()

# while not rospy.is_shutdown():
#     # group.set_pose_target(ready_pos)
#     group.set_position_target([ready_pos.position.x, ready_pos.position.y, ready_pos.position.z], group.get_end_effector_link())
#     plan = group.go(wait=True)
#     print(plan)
#     group.stop()
#     group.clear_pose_targets()
#     rate.sleep()