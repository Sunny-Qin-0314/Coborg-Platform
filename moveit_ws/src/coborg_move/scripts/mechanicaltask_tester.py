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


# Simulation Variables
monte_carlo_runs = 2000
np_seed = 0
perimeter_percentage = 0.1
buffer_space_run_ratio = 0.25
stable_space_run_ratio = 0.1
# Task Space Definitions
task_space_rad_max_ft = 3
task_space_rad_min_ft = 1.5 # Consider reducing this to account for offset displacement?  Or just add it as a factor later?
task_space_horizontal_max_deg = 30
task_space_horizontal_min_deg = -30
task_space_vertical_max_deg = 10
task_space_vertical_min_deg = -90
# Buffer Space Definitions
buffer_space_rad_diff_ft = 3/12
buffer_space_horizontal_angle_diff_deg = 5
buffer_space_vertical_angle_diff_deg = 5
# Stabilization Space Definitions
stable_space_horizontal_max_deg = 20
stable_space_horizontal_min_deg = -90
stable_space_x_max_ft = 6/12
stable_space_x_min_ft = -6/12
stable_space_y_max_ft = 6/12
stable_space_y_min_ft = -6/12
stable_space_z_max_ft = 6/12
stable_space_z_min_ft = -6/12
# Additionally there is a visual space, which includes the task
# space and buffer space and additionally room for spread
#  between the hands (midpoint in task space, one hand out)
# MoveIt Variables
group_name = "coborg_arm"
if group_name is "coborg_arm": 
    compact_configuration_rad = np.array([-1.91986, -2.37365, -2.51327])
elif group_name is "dof_4_arm":
    compact_configuration_rad = np.array([0.0, -1.91986, -2.37365, -2.51327])
elif group_name is "dof_5_config_z_arm":
    compact_configuration_rad = np.array([0.0, -1.9266, 0.0, -2.0307, -2.3778])
elif group_name is "dof_5_config_y_arm":
    compact_configuration_rad = np.array([0.0, -1.9266, 0.0, -2.0307, -2.3778])
elif group_name is "dof_5_config_y_arm":
    compact_configuration_rad = np.array([0.0, -1.9266, 0.0, -2.0307, -2.3778])

chest_to_t265_transform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[-0.1651,0.2159,0.3048,1]])
t265_to_world_transform=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0.2794,-0.4999,-0.67305,1]])
# t265_to_world*chest_to_origin*(p_chest)
chest_to_origin_transform = np.matmul(t265_to_world_transform, chest_to_t265_transform)

stable_waypoints_num = 20


# Check goal point with inverse kinematics
def IK_runner(goal_point, origin_point = False, stable_waypoints = False):
    # This function confirms whether goal_point, a point in
    # Cartesian space is accessible by MoveIt's inverse kinematics
    # IK_outcome is 0 if IK failed and 1 if it succeeded
    # goal_point's origin and axes are described at the top of this file
    # goal_point is a (3,1) numpy array with x, y, z positions in feet
    # origin_point is either a boolean False or a (3,) numpy array with x,y,z starting positions in feet.
    # If it is a boolean False, then this means that the "compact position" joint goal should be used
    # instead of some x, y, z cartesian point
    # stable_waypoints is a boolean 
    
    ft_to_m = 0.3048
    #Send the arm to its initial position
    if origin_point:
        # If a Cartesian origin point was given, start there
        pose_goal = geometry_msgs.msg.Pose()
        origin_point_transformed = chest_to_origin_transform @ np.array([origin_point, 1])
        pose_goal.position.x = origin_point_transformed[0] * ft_to_m
        pose_goal.position.y = origin_point_transformed[1] * ft_to_m
        pose_goal.position.z = origin_point_transformed[2] * ft_to_m
        # group.set_pose_target(pose_goal)
        # Feng Xiang
        group.set_position_target([pose_goal.position.x,pose_goal.position.y,pose_goal.position.z], group.get_end_effector_link())
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else
        group.go(compact_configuration_rad, wait=True)
        group.stop()
    # MoveIt trajectory planning
    if stable_waypoints:
        waypoints = []

        wpose = group.get_current_pose().pose
        start_pose = np.array([wpose.position.x, wpose.position.y, wpose.position.z])
        start_pose_transformed = chest_to_origin_transform @ np.array([start_pose,1])
        goal_point_transformed = chest_to_origin_transform @ np.array([goal_point,1])
        x_step = (goal_point_transformed[0] - start_pose_transformed[0])/stable_waypoints_num
        y_step = (goal_point_transformed[1] - start_pose_transformed[1])/stable_waypoints_num
        z_step = (goal_point_transformed[2] - start_pose_transformed[2])/stable_waypoints_num
        for ii = range(stable_waypoints_num):
            wpose.position.x += x_step
            wpose.position.y += y_step
            wpose.position.z += z_step
            waypoints.append(copy.deepcopy(wpose))
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (IK_outcome, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        else:
            IK_outcome = group.plan()
            group.stop()
            group.clear_pose_targets()

    return IK_outcome


print("end")