#!/usr/bin/env python

# The template of this code was modified based on https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# This file adds a wall into the world. The user defines the height of the wall when running the file. The backpack arm should avoid colliding into the wall when executing its optimal trajectory. Use the command "rosrun move_backpack add_wall.py 'wall height' " to run the file from the terminal 
 
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg
from moveit_commander.conversions import pose_to_list


class AddWall(object):

  def __init__(self, height):
    super(AddWall, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv[0])
    rospy.init_node('add_wall',
                    anonymous=True)

    self.scene = moveit_commander.PlanningSceneInterface()
    self.group = moveit_commander.MoveGroupCommander("right_arm")
    
    # Load values in object variable
    self.height = float(height)
    self.planning_frame = self.group.get_planning_frame()

    # Publish the wall to "/attached_collision_object" so backpack arm can avoid collision
    self.attach_obj_pub = rospy.Publisher('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, queue_size=20)


  # Wait for scene updates
  # Add the actual wall in which collision cannot be ignored
  def add_collision(self):

    # Define the shape and pose of the wall
    col_obj = moveit_msgs.msg.CollisionObject()
    obj = moveit_msgs.msg.AttachedCollisionObject()
    obj.link_name = "motor1/INPUT_INTERFACE"
    col_obj.header.frame_id = self.planning_frame

    #The id of the object is used to identify it.
    col_obj.id = "wall"
    col_obj.header.stamp = rospy.Time.now()

    #Define a box to add to the world. 
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.0]*3
    primitive.dimensions[0] = .5
    primitive.dimensions[1] = 1
    primitive.dimensions[2] = .1

    box_pose = geometry_msgs.msg.Pose()
    box_pose.position.y = .3
    
    # Add a slight height to the defined wall height to ignore collisioin errors when the robot arm just touches the wall
    box_pose.position.z = self.height + 0.02

    col_obj.primitives = [primitive]
    col_obj.primitive_poses = [box_pose]
    col_obj.operation = col_obj.ADD
    obj.object = col_obj

    return obj

  # Visualization of the wall
  def display(self, msg):
    self.attach_obj_pub.publish(msg)

def main():
  print "============ Press `Enter` to add wall to scene ..."
  raw_input()
  height = sys.argv[1]
  wall = AddWall(height)
  rospy.sleep(4.0)
  col_wall = wall.add_collision()
  wall.display(col_wall)


if __name__ == '__main__':
  main()

