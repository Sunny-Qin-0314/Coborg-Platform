#!/usr/bin/env python

# The template of the code was modified based on https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# This file adds obstacles as boxes to the world in which the backpack arm should avoid contacting. The user should define the obstacle's postion within the world and its size. Multiple boxes could be added, in which they are named in consecutive order. Use the command "rosrun move_backpack add_objects.py 'box x position' 'box y position' 'box z position' 'width' 'length' 'height' " to execute this file 


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String

class AddObject(object):
  
  def __init__(self, x, y, z, width, length, height):
    super(AddObject, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv[0])
    rospy.init_node('add_object',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = group.get_planning_frame()
    attach_object_pub = rospy.Publisher('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, queue_size=20)

    # Load values in object variables
    self.box_name = "box" + str(len(scene.get_attached_objects())+1)
    print "============ Press `Enter` to add " + self.box_name + " to the scene ..."
    raw_input()
    self.robot = robot
    self.scene = scene
    self.attach_obj_pub = attach_object_pub
    self.planning_frame = planning_frame

    self.x = float(x)
    self.y = float(y)
    self.z = float(z)
    self.width = float(width)
    self.height = float(height)
    self.length = float(length)


  # Add box in planning scene such that arm avoids collision  
  # Reference source: https://github.com/ros-planning/moveit/issues/1217
  def add_collision(self):

    # This contains the shapes and poses for the obstacle to be added
    col_obj = moveit_msgs.msg.CollisionObject()
    obj = moveit_msgs.msg.AttachedCollisionObject()
    obj.link_name = "motor1/INPUT_INTERFACE"
    col_obj.header.frame_id = self.planning_frame
  
    #The id of the object is used to identify it.
    col_obj.id = self.box_name
    col_obj.header.stamp = rospy.Time.now()

    #Define a box to add to the world. 
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.0]*3
    primitive.dimensions[0] = self.width
    primitive.dimensions[1] = self.length
    primitive.dimensions[2] = self.height
    
    box_pose = geometry_msgs.msg.Pose()
    box_pose.position.x = self.x
    box_pose.position.y = self.y
    box_pose.position.z = self.z

    col_obj.primitives = [primitive]
    col_obj.primitive_poses = [box_pose]
    col_obj.operation = col_obj.ADD
    obj.object = col_obj

    return obj
  
  # Visualization of the object
  def display(self, msg):
    self.attach_obj_pub.publish(msg)  

def main():
  x = sys.argv[1]
  y = sys.argv[2]
  z = sys.argv[3]
  width = sys.argv[4]
  length = sys.argv[5]
  height = sys.argv[6]

  add_object_client = AddObject(x, y, z, width, length, height)
  rospy.sleep(4.0)
  col_obj = add_object_client.add_collision()
  add_object_client.display(col_obj)  
  

if __name__ == '__main__':
  main()

