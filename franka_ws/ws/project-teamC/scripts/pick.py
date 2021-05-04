#! /usr/bin/env python
import params
import time

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
    # Check params.pegboard to see if the tool the user requested is even on the pegboard. If it's not error out.
    # If it's there, pick it up from its location and hold in handoff location
    # Wait until user pulls the robot arm out of it's impedence bubble
    # Let go of tool
    # Update params.pegboard and return to home position

    pegboard_tool_idx = np.where(np.array(params.pegboard) == tool.value)[0]
    if len(pegboard_tool_idx) != 0:
        # If there is a tool on the pegboard equal to the tool value
        # Pick tool from the first available pegboard spot
        print("Picking up tool {} from pegboard postion {}".format(tool.name, pegboard_tool_idx[0]))
        params.userNoGrab = False
        run_pick(pegboard_tool_idx[0], tool) # start aruco tag place function
        if params.userNoGrab:
            print("Picking failed, tool {} returned to pegboard".format(tool.name))
        else:
            params.pegboard[pegboard_tool_idx[0]] = 0 # pegboard spot now empty
            print("Tool {} successfully picked from pegboard!".format(tool.name))

    else:
        # there is no tool
        print("Tool requested not on pegboard")


def run_pick(tool_index, tool_id):

    # The goal pose definition : rotation, translation relative to camera frame
    goal_rotation = np.array([[0, 0, 1],
                            [0, -1, 0],
                            [1, 0, 0]])
    
    # transform goal pose to the world frame
    goal_translations = [np.array([0.385, -0.33, 0.50]), np.array([0.385, -0.245, 0.50]), np.array([0.385, -0.16, 0.50]), np.array([0.385, -0.075, 0.50])]
    goal_transformed =  params.azure_kinect_to_world_transform * RigidTransform(
        rotation = goal_rotation,
        translation = goal_translations[tool_index],
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )

    home_flipped = RigidTransform(rotation=np.array([
            [-1, 0, 0],
            [0, 1, 0],
            [0, 0, -1],
        ]), translation=np.array([0.3069, 0, 0.4867]),
        from_frame='franka_tool', to_frame='world')

    hand_off = RigidTransform(rotation=np.array([
            [-0.86405261, 0.49902793, 0.06606846],
            [0.4933981, 0.86559895, -0.08530919],
            [-0.09976046, -0.04111357, -0.99416161],
        ]), translation=np.array([0.5182401, -0.21211296, 0.28499551]),
        from_frame='franka_tool', to_frame='world')

    #pegboard_z_height = 0.265
    intermediate_pose_y_dist = 0 # offset pegboard y distance world frame
    intermediate_pose_z_height = 0.55 # offset pegboard z height world frame

    print('Starting Robot')
    fa = FrankaArm()  

    print('Opening Grippers')
    fa.open_gripper()

    print('Homing Robot')
    #Reset Joints
    fa.reset_joints()
   
    print('Moving to Pegboard')
    # move to the Intermediate robot pose x offset infront of the pegboard
    intermediate_robot_pose_1 = goal_transformed.copy()
    intermediate_robot_pose_1.translation[1] = intermediate_pose_y_dist
    fa.goto_pose(intermediate_robot_pose_1)
    
    # move to the goal pose
    #goal_transformed.translation[2] = pegboard_z_height  # goal height in the world frame. This is to fix the z axis to the pegboard dropoff height.
    fa.goto_pose(goal_transformed, 5, force_thresholds=[20, 20, 20, 20, 20, 20])

    print('Closing Grippers')
    fa.close_gripper()

    print('Retrieving Tool')
    # move to the Intermediate robot pose x offset infront of the pegboard
    intermediate_robot_pose_2 = goal_transformed.copy()
    intermediate_robot_pose_2.translation[2] = intermediate_pose_z_height
    fa.goto_pose(intermediate_robot_pose_2)

    print('Returning Home')
    # Reset Joints
    fa.reset_joints()

    # Go to hand off location
    fa.goto_pose(hand_off, ignore_virtual_walls=True)

    # Impedance Control
    # Initalize variables
    distThres = 0.07
    controllerSec = 20
    waitSec = 1

    print("Starting Impedance Control")
    print("Please pickup the tool from the robot")
    pose = fa.get_pose()
    desired_position = pose.translation

    # Testing code for impedance control
    

    #fa.apply_effector_forces_torques(controllerSec,0,0,0, block=False)
    fa.goto_pose(pose, buffer_time = 20, use_impedance=True, dynamic = True, cartesian_impedances=[0,0,200,0,0,0], block=False, ignore_virtual_walls=True)
    beginTime = time.time()
    currTime = time.time()
    diffTime = float(currTime - beginTime)

    while diffTime < controllerSec:
        pose = fa.get_pose()
        position = pose.translation
        error = desired_position[0:2] - position[0:2]
        if np.linalg.norm(error) > distThres:
            fa.stop_skill()
            fa.open_gripper()
            print("Releasing Tool")
            print("Sleeping for {} seconds".format(waitSec))
            time.sleep(waitSec)
            fa.reset_joints()
            return
        currTime = time.time()
        diffTime = float(currTime - beginTime)
    
    # Replace tool here
    print("Returning Tool")
    fa.stop_skill()
    fa.reset_joints()
    pegboard_z_height = 0.265
    intermediate_tool_z_height = 0.30 # offset intermediate z height (tool)
    intermediate_pegboard_z_height = 0.55 # offset intermediate z height (pegboard) 

    # move to the new intermediate robot pose on the pegboard
    intermediate_robot_pose_offset = goal_transformed.copy()
    intermediate_robot_pose_offset.translation[2] = intermediate_pegboard_z_height
    fa.goto_pose(intermediate_robot_pose_offset)
    
    # move to the goal pose
    goal_transformed.translation[2] = pegboard_z_height  # goal height in the world frame. This is to fix the z axis to the pegboard dropoff height.
    fa.goto_pose(goal_transformed, 5, force_thresholds=[20, 20, 20, 20, 20, 20])

    print('Opening Grippers')
    fa.open_gripper()

    print('Returning Home')
    # Move to intermediate robot pose, but lower the z height by 10 cm to avoid hitting the camera
    intermediate_robot_pose_offset = goal_transformed.copy()
    intermediate_robot_pose_offset.translation[2] = intermediate_pegboard_z_height - 0.1
    fa.goto_pose(intermediate_robot_pose_offset)

   # Reset Joints
    fa.reset_joints()

    params.userNoGrab = True #if the user successfully retrieved the tool, return true
    return




if __name__ == "__main__": #unit testing code goes here
    class Tool(enum.IntEnum):
        test = 1

    params.validate() # initialize pegboard
    print(params.pegboard)
    execute(Tool.test)
    print(params.pegboard)