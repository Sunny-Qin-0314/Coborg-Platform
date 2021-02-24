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
    pose_name = sys.argv[1]
    # Various joint values taken from matlab code https://github.com/biorobotics/modularArmControl/blob/master/setup/backpackSetup.m
    if pose_name == "pose_1":
      joint0 = float(1.2512 )
      joint1 = float(-0.7863)
      joint2 = float(-0.8820)
    if pose_name == "pose_2":
      joint0 = float(1.0225)
      joint1 = float(-1.4381)
      joint2 = float(-1.3667)
    if pose_name == "pose_3":
      joint0 = float(1.0734)
      joint1 = float(-0.7035)
      joint2 = float(-0.7844)
    if pose_name == "pose_4":
      joint0 = float(-2.4394)
      joint1 = float(-2.4953)
      joint2 = float(-2.5588)

    print "============ Press `Enter` to obtain the current state of the backpack arm (press ctrl-d to exit) ..."
    raw_input()
    move_to_pose_client = MoveToPose(joint0, joint1, joint2)
    
    print "============ Press `Enter` to execute a movement to " + pose_name
    raw_input()
    
    if move_to_pose_client.go_to_joint_state():
      print "============ " + pose_name + " successfully executed!"
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
