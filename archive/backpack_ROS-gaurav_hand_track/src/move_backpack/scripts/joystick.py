#!/usr/bin/env python

import varsList as vL
import pygame
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import os
from sensor_msgs.msg import JointState

pygame.init()
pygame.joystick.init()
pygame.display.set_mode((1, 1))

positions = [0]*12

with open("/home/deepwireless/catkin_ws/src/move_backpack/scripts/all_poses.txt", "rt") as f:
    content = f.read()
    poses = content.split("\n")
    index = 0
    for pose in poses:
        if index == 12:
	    break
        values = pose.split(" ")
        positions[index] = values
        index += 1
        

for x in range(pygame.joystick.get_count()):
    pygame.joystick.Joystick(x).init()

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
    def __init__(self):
      super(MoveToPose, self).__init__()

      group_name = "right_arm"

      self.group = moveit_commander.MoveGroupCommander(group_name)
      self.robot = moveit_commander.RobotCommander()
      self.pre_plan = rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.preplan, queue_size=1)

      # No predefined plan initialized yet
      self.preplan_pose = False
      self.run()

    def preplan(self, data):
        group = self.group
        joint1 = data.position[0]
        joint2 = data.position[1]
        joint3 = data.position[2]
        
        # For testing purposes, when in pose1, preplan to pose4
        if 1.1512 < joint1 < 1.3512 and -0.8863 < joint2 < -0.6863 and -0.9820 < joint2 < -0.7820:
            joint_goal = self.group.get_current_joint_values()
            joint_goal[0] = -2.4394
            joint_goal[1] = -2.4953 
            joint_goal[2] = -2.5588
            plan_pose4 = group.plan(joint_goal)
            self.preplan_pose = plan_pose4
            print("just calculated plan for pose 4")
	    

    def go_to_joint_state(self, joint0, joint1, joint2):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = joint0
    	joint_goal[1] = joint1
    	joint_goal[2] = joint2
    	self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        current_joints = self.group.get_current_joint_values()  
        return all_close(joint_goal, current_joints, 0.01)


    def run(self):
    	while True: 
            for event in pygame.event.get():
		pass
    
            for ax in range(pygame.joystick.Joystick(0).get_numaxes()):
                vL.axes[ax] = pygame.joystick.Joystick(0).get_axis(ax)
            for button in range(pygame.joystick.Joystick(0).get_numbuttons()):
                vL.buttons[button] = pygame.joystick.Joystick(0).get_button(button)
            for hat in range(pygame.joystick.Joystick(0).get_numhats()):
                vL.povs[hat] = pygame.joystick.Joystick(0).get_hat(hat)

            # Comment out defined values. Read from text file instead
            '''
            # Various joint values taken from matlab code https://github.com/biorobotics/modularArmControl/blob/master/setup/backpackSetup.m
            if vL.buttons[0]:
            print("button 1 pressed")
            joint0 = float(1.2512 )
            joint1 = float(-0.7863)
            joint2 = float(-0.8820)
            move_to_pose_client = MoveToPose(joint0, joint1, joint2)
	    move_to_pose_client.go_to_joint_state()
            elif vL.buttons[1]:
                print("button 2 pressed")
            joint0 = float(1.0225)
            joint1 = float(-1.4381)
            joint2 = float(-1.3667)
            move_to_pose_client = MoveToPose(joint0, joint1, joint2)
	    move_to_pose_client.go_to_joint_state()
            elif vL.buttons[2]:
                print("button 3 pressed")
            joint0 = float(1.0734)
            joint1 = float(-0.7035)
            joint2 = float(-0.7844)
            move_to_pose_client = MoveToPose(joint0, joint1, joint2)
	    move_to_pose_client.go_to_joint_state()
            elif vL.buttons[3]:
                print("button 4 pressed")
            joint0 = float(-2.4394)
            joint1 = float(-2.4953)
            joint2 = float(-2.5588)
            move_to_pose_client = MoveToPose(joint0, joint1, joint2)
	    move_to_pose_client.go_to_joint_state() 
            '''
    
            for pose in range(12):
                # First value of pose is 1 indicates the pose is defined
                if vL.buttons[pose] and positions[pose][0] == "1":
                    print("button %d pressed" % (pose+1))
                    print("this is self.preplan_pose", self.preplan_pose)

                    # Check if a predefined pose already exists
                    if self.preplan_pose != False:
                        self.group.execute(self.preplan_pose[1])
                        self.preplan_pose = False
                    else:
                        joint0 = float(positions[pose][1])
                        joint1 = float(positions[pose][2])
                        joint2 = float(positions[pose][3])
	                self.go_to_joint_state(joint0, joint1, joint2)     
        
            pygame.time.wait(10)


if __name__ == '__main__':
  rospy.init_node('joystick', anonymous=True)
  movegroup = MoveToPose()
  rospy.spin()

    	



