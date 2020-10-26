#!/usr/bin/env python

# The code is modified based on https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# The file displays the trajectory for the backpack arm to move to the pose commanded by the user. Currently there are four pose options, which are pose_1, pose_2, pose_3, and pose_4. Use the command "rosrun move_backpack move_to_defined_pose.py 'pose option' " from the terminal to execute the command

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import roslaunch
import os

# Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveToPose(object):
  def __init__(self, joint0, joint1, joint2):
    super(MoveToPose, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv[0])
    rospy.init_node('backpack_move',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    '''
    #Print frame, end_effector_link, and group names
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Print additional information of robot states
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    '''
    # Load values into object variables
    self.robot = robot
    self.group = group
    self.joint0 = joint0
    self.joint1 = joint1
    self.joint2 = joint2
    

  # Plan and execute the path to joint states of defined pose
  def go_to_joint_state(self):
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = self.joint0
    joint_goal[1] = self.joint1
    joint_goal[2] = self.joint2

    group.go(joint_goal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
                                            

def main():
  try:
    joint0 = float(sys.argv[1])
    joint1 = float(sys.argv[2])
    joint2 = float(sys.argv[3])
    move_to_pose_client = MoveToPose(joint0, joint1, joint2)
    if move_to_pose_client.go_to_joint_state():
      #os.system("roslaunch move_backpack hebimove.launch")
      return

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

main()
