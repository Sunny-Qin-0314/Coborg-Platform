#!/usr/bin/env python

import hebi
import sys
import os
import rospy
import make_kin_utils
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import make_kin_utils

# numpy is aliased to np throughout this documentation.
import numpy as np

from time import sleep

lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
print(lookup.entrylist)


class RightArm(object):
  
  def __init__(self):

    #subscribers for moving to goal states and hold at obstacle position
    go = rospy.Subscriber("/joint_states", JointState, self.feedback_callback, queue_size=1)
    stop = rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.fb, queue_size=1)
    move_to_joint = rospy.Subscriber("/joint_states", JointState, self.move_to_joint, queue_size=1)


    kin, joint_mask = make_kin_utils.make_backpack_kin()
    self.group = hebi.Lookup().get_group_from_names(['*'], ['base_temp', 'X-00476', 'elbow_temp'])
    self.kin = kin
    self.joint_mask = joint_mask
    self.stop = False
    self.at_new = False
    self.rate = rospy.Rate(20)
    self.joint_pose = [0]*3

  # Joint limits based on position
  # Temporary method right now once gravity compensation works
  def threshold(self, joints, efforts):

    if joints[0] > -1.57 and joints[0] < 1.57:
      if efforts[0] > 7 or efforts[0] < -2:
        return True
    else:
      if efforts[0] < -6.5 or efforts[0] > 2:
        return True

    if joints[1] > 0 and joints[1] < 3.14:
      if efforts[1] < -5 or efforts[1] > 3:
        return True
    else:                               
      if efforts[1] > 5 or efforts[1] < -3:
        return True

    if joints[2] > 0 and joints[2] < 3.14:
      if efforts[2] > 2 or efforts[2] < -2:
        return True
    else:
      if efforts[2] < -2.5 or efforts[2] > 2:
        return True

    
    return False
    

  def feedback_callback(self, data):
    # Robot has not collidesd
    if not self.stop:
      group = self.group
      kin = self.kin
      joint_mask = self.joint_mask
      positions = [0]*3

      # Move robot to joint state positions
      positions[0] = data.position[0]
      positions[1] = data.position[1]
      positions[2] = data.position[2]

      # Gravity Compensation values
      gravity= np.matrix([0,0,9.81]).T
      tauG, fkout = make_kin_utils.getGravCompTorques(kin, joint_mask, positions, gravity)
      group_command = hebi.GroupCommand(3)
      group_command.position = positions
      #group_command.effort = tauG
      group.send_command(group_command)

  def fb(self, data):

    kin = self.kin
    joint_mask = self.joint_mask
    group = self.group
    positions = [0]*3

    # Robot collides when exceeds joint threshold
    if (self.threshold(data.position, data.effort)) or self.stop:

      # Unsubscribe to /joint_states topic
      if not self.stop:
        self.joint_pose[0] = data.position[0]
        self.joint_pose[1] = data.position[1]
        self.joint_pose[2] = data.position[2]
        self.stop = True
        
      positions[0] = self.joint_pose[0]
      positions[1] = self.joint_pose[1]
      positions[2] = self.joint_pose[2]

      gravity= np.matrix([0,0,9.81]).T
      tauG, fkout = make_kin_utils.getGravCompTorques(kin, joint_mask, positions, gravity)
      efforts = [0]*3
      group_command = hebi.GroupCommand(3)
      group_command.position = positions
      #group_command.effort = tauG
      group.send_command(group_command)



  def move_to_joint(self, data):
    if self.stop:
      joint_pose = self.joint_pose
      # Adjust joint states to current position
      os.system("rosrun move_backpack move_to_pose.py %e %e %e" % (joint_pose[0], joint_pose[1], joint_pose[2]))

      # Enable replanning and control 
      self.stop = False


if __name__ == '__main__':
  rospy.init_node('right_arm', anonymous=True)
  objecta = RightArm()
  rospy.spin()


