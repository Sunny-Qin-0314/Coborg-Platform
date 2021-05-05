#! /usr/bin/env python
import params
from utils import *

import enum
from pathlib import Path

from frankapy import FrankaArm
from frankapy.utils import min_jerk, min_jerk_weight
from autolab_core import RigidTransform, Point

from geometry_msgs.msg import Pose
import math
import numpy as np

def execute(tool):

    # TO DO:
    # Check the table to see if the tool (aruco marker) is there. If not error out.
    # Check params.pegboard to see if there's space on the pegboard to place the tool. If no space error out.
    # Place the tool 
    # update params.pegboard
    # return to home
    
    # add protective boundaries (especially above) to prevent accidental robot collisions

    pegboard_empspots_idx = np.where(np.array(params.pegboard) == 0)[0] #output is tuple, we want first element [0]
    if len(pegboard_empspots_idx) != 0:
        # there is an empty spot
        # place it into the first spot it found empty
        print("Picking up tool {} and placing it on pegboard postion {}".format(tool.name, pegboard_empspots_idx[0]))
        run_place(pegboard_empspots_idx[0], tool) #start aruco tag place function
        params.pegboard[pegboard_empspots_idx[0]] = tool.value
        print("Tool {} successfully placed on pegboard!".format(tool.name))
    else:
        # there is no availble spot
        print("No empty spot to place the tool")

def run_place(tool_index, tool_id):
    '''
    Pick up one tool and place it into the specific pegboard spot

    Inputs:
        tool_index: int, the index of the availble position on the board [0,1,2,3]
        tool_id: int, the tool want to pick up. This will be used to read the correct marker pose topic [1,2,3,4]
                tool1: aruco marker 100
                tool2: aruco marker 153
                tool3: aruco marker 80
                tool4: aruco marker 33
    '''

    aruco_offset = -0.10 # offset of aruco tag to center of tool
    tool_z_height = 0.01  # center of tool height = 1 cm
    pegboard_z_height = 0.265
    intermediate_tool_z_height = 0.30 # offset intermediate z height (tool)
    intermediate_pegboard_z_height = 0.55 # offset intermediate z height (pegboard) 

    print('Starting Robot')
    fa = FrankaArm()
    print('Opening Grippers')
    # Open Gripper
    fa.open_gripper()

    print('Homing Robot')
    # Reset Joints
    fa.reset_joints()

    '''
    Initial pose -> tool pose
    Tools pose on the table are preprocessed to be the correct pose for gripper to move
    '''
    topic_name = "/aruco_multiple/pose"+ str(tool_id.value)
    
    print('Waiting for Tool {} on Table'.format(tool_id.name))
    
    # grab pose of aruco marker tool_id 
    pose = rospy.wait_for_message(topic_name, Pose) 
    
    print('Calculating Franka Transforms')
    rot_matrix = quaternion_rotation_matrix(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    trans_matrix= np.array([pose.position.x, pose.position.y, pose.position.z])
    
    _,_,yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    print(yaw)
    #offset x axis in aruco frame. first we transform from the camera frame to aruco frame
    aruco_translation = rot_matrix@np.array([aruco_offset, 0, 0]) + trans_matrix

    # rotate about y axis 180 deg (change z axis to point down)
    rot_grip_flip = np.array([[-1, 0, 0],[0,-1,0],[0, 0, 1]])
    rot_matrix =  np.array([[np.cos(-yaw), np.sin(-yaw), 0],[-np.sin(-yaw),np.cos(-yaw),0],[0, 0, 1]])
    tool_rotation = rot_matrix@rot_grip_flip  # right hand mulplication

    # transform tool pose to the world frame
    tool_transformed =  params.azure_kinect_to_world_transform * RigidTransform(
        rotation = tool_rotation,
        translation= aruco_translation, #adds offset
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )

    intermediate_robot_pose = tool_transformed.copy()
    intermediate_robot_pose.translation = [tool_transformed.translation[0], tool_transformed.translation[1], intermediate_tool_z_height] #increased z height


    # The goal pose definition : rotation, translation relative to camera frame
    goal_rotation = np.array([[0, 0, 1],
                            [0, -1, 0],
                            [1, 0, 0]])
    
    # transform goal pose to the world frame
    goal_translations = [np.array([0.375, -0.33, 0.50]), np.array([0.375, -0.245, 0.50]), np.array([0.375, -0.16, 0.50]), np.array([0.375, -0.075, 0.50])]
    goal_transformed =  params.azure_kinect_to_world_transform * RigidTransform(
        rotation = goal_rotation,
        translation = goal_translations[tool_index],
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )

    print('Moving to Tool {}'.format(tool_id.name))
    #Move to intermediate robot pose aruco 
    fa.goto_pose(intermediate_robot_pose)

    # use hard coded tool height
    tool_transformed.translation[2] = tool_z_height    

    #Move to tool 
    fa.goto_pose(tool_transformed, 5, force_thresholds=[30, 30, 30, 30, 30, 30])

    #Close Gripper
    fa.goto_gripper(0.03, grasp=True, force=40.0)

    print('Moving to Pegboard Location {}'.format(tool_index))
    # Move to intermediate robot pose aruco 1 (higher than the pegboard)
    intermediate_robot_pose.translation[2] = intermediate_pegboard_z_height # offset intermediate z height franka frame
    fa.goto_pose(intermediate_robot_pose)    

    # Reset joints before going to the next intermediate pose
    fa.reset_joints()

    # move to the new intermediate robot pose on the pegboard
    intermediate_robot_pose_offset = goal_transformed.copy()
    intermediate_robot_pose_offset.translation[2] = intermediate_pegboard_z_height
    fa.goto_pose(intermediate_robot_pose_offset)
    
    # move to the goal pose
    goal_transformed.translation[2] = pegboard_z_height  # goal height in the world frame. This is to fix the z axis to the pegboard dropoff height.
    fa.goto_pose(goal_transformed, 5, force_thresholds=[30, 30, 30, 30, 30, 30])

    print('Opening Grippers')
    fa.open_gripper()

    print('Returning Home')
    # Move to intermediate robot pose, but lower the z height by 10 cm to avoid hitting the camera
    intermediate_robot_pose_offset = goal_transformed.copy()
    intermediate_robot_pose_offset.translation[2] = intermediate_pegboard_z_height - 0.1
    fa.goto_pose(intermediate_robot_pose_offset)


    # Reset Joints
    fa.reset_joints()

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def quaternion_rotation_matrix(x,y,z,w):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = w
    q1 = x
    q2 = y
    q3 = z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix



if __name__ == "__main__": # unit testing code goes here
    class Tool(enum.IntEnum):
        test = 1
    params.validate() # initialize pegboard
    print(params.pegboard)
    execute(Tool.test)
    print(params.pegboard)
 
