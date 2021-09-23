#!/usr/bin/env python2

# Types of arms:
# coborg_arm
# dof_4_arm
# dof_5_config_z_arm
# dof_5_config_y_arm

# coborg_arm : preset joints
# home - -1.91986, -2.37365, -2.51327

# dof_4_arm : preset joints
# home - 0.0, -1.91986, -2.37365, -2.51327

# dof_5_config_z_arm : preset joints
# home - 0.0, -1.9266, 0.0, -2.0307, -2.3778

# dof_5_config_y_arm : preset joints
# home - 0.0, -1.9266, 0.0, -2.0307, -2.3778

# Usage:
# roslaunch <arm type> demo.launch
# rosrun moveit_tutorials move_group_python_interface_tutorial.py

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

# Coordinate definitions
# The origin is at the center of the user's chest, the midpoint between
# the rotation points of the user's shoulders
# The x-axis is positive in the anterior direction
# The y-axis is positive in the medial direction
# The z-axis is positive in the superior direction
# Anatomical planes and axes: https://tinyurl.com/4nu5j344
# Horizontal angles are rotated around the z-axis with the x-axis being 0
# Vertical angles are rotated around the y-axis with the x-axis being 0


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
        origin_point_transformed = np.matmul(chest_to_origin_transform, np.array([origin_point, 1]))
        pose_goal.position.x = origin_point_transformed[0] * ft_to_m
        pose_goal.position.y = origin_point_transformed[1] * ft_to_m
        pose_goal.position.z = origin_point_transformed[2] * ft_to_m
        # group.set_pose_target(pose_goal)
        # Feng Xiang
        group.set_position_target([pose_goal.position.x,pose_goal.position.y,pose_goal.position.z], group.get_end_effector_link())
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        group.go(compact_configuration_rad, wait=True)
        group.stop()
    # MoveIt trajectory planning
    if stable_waypoints:
        waypoints = []

        wpose = group.get_current_pose().pose
        start_pose = np.array([wpose.position.x, wpose.position.y, wpose.position.z])
        start_pose_transformed = np.matmul(chest_to_origin_transform, np.array([start_pose,1]))
        goal_point_transformed = np.matmul(chest_to_origin_transform, np.array([goal_point,1]))
        x_step = (goal_point_transformed[0] - start_pose_transformed[0])/stable_waypoints_num
        y_step = (goal_point_transformed[1] - start_pose_transformed[1])/stable_waypoints_num
        z_step = (goal_point_transformed[2] - start_pose_transformed[2])/stable_waypoints_num
        for ii in range(stable_waypoints_num):
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


# ROS/Move-It Initializations
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander(group_name)
group.set_planner_id("RRTConnect")
group.set_planning_time(1.0)
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# Run task space checks
# Check the task space
np.random.seed(np_seed)
# Create task space boundaries
# rad_min, rad_max
# horizontal_min, horizontal_max
# vertical_min, vertical_max
task_space_boundaries = np.array([[task_space_rad_min_ft, task_space_rad_max_ft],[task_space_horizontal_min_deg, task_space_horizontal_max_deg],[task_space_vertical_min_deg, task_space_vertical_max_deg]])
# Decide which points will lie on particular boundaries
perimeter_choices = np.random.choice([-1,0,1], size = (task_space_boundaries.shape[0], monte_carlo_runs), p = [perimeter_percentage/2, 1-perimeter_percentage, perimeter_percentage/2])
# Create array to hold task space target goals in spherical coordinates
task_space_goals_spherical = np.zeros(perimeter_choices.shape)
# Add "min" perimeter values
perimeter_choices_worker = np.copy(perimeter_choices)
perimeter_choices_worker[perimeter_choices > 0] = 0
task_space_goals_spherical = task_space_goals_spherical + np.matmul(np.diag(task_space_boundaries[:,0]), (perimeter_choices_worker * -1))
# Add "max" perimeter values
perimeter_choices_worker = np.copy(perimeter_choices)
perimeter_choices_worker[perimeter_choices < 0] = 0
task_space_goals_spherical = task_space_goals_spherical + np.matmul(np.diag(task_space_boundaries[:,1]), perimeter_choices_worker)
# Add random perimeter values
perimeter_choices_worker = np.copy(perimeter_choices)
perimeter_choices_worker = np.abs(np.abs(perimeter_choices_worker) - 1)
task_space_goals_spherical = task_space_goals_spherical + np.add(np.matmul(np.diag(task_space_boundaries[:,1] - task_space_boundaries[:,0]), np.random.rand(perimeter_choices_worker.shape[0], perimeter_choices_worker.shape[1])), np.repeat(task_space_boundaries[:,0].reshape(-1,1), perimeter_choices_worker.shape[1], axis = 1)) * perimeter_choices_worker
# Convert from spherical coordinates to cartesian coordinates
task_space_goals_cartesian = np.zeros(task_space_goals_spherical.shape)
task_space_goals_cartesian[0,:] = task_space_goals_spherical[0,:] * np.cos(task_space_goals_spherical[1,:]*2*np.pi/360) * np.cos(task_space_goals_spherical[2,:]*2*np.pi/360)
task_space_goals_cartesian[1,:] = task_space_goals_spherical[0,:] * np.sin(task_space_goals_spherical[1,:]*2*np.pi/360) * np.cos(task_space_goals_spherical[2,:]*2*np.pi/360)
task_space_goals_cartesian[2,:] = task_space_goals_spherical[0,:] * np.cos(task_space_goals_spherical[1,:]*2*np.pi/360) * np.sin(task_space_goals_spherical[2,:]*2*np.pi/360) * -1
# Check each point for IK success
task_space_success = []
for run_num in range(perimeter_choices.shape[1]):
    task_space_success.append(IK_runner(task_space_goals_cartesian[:,run_num]))
# Plot task space results, colored by success
fig = plt.figure()
ax = plt.axes(projection='3d')
norm = Normalize()
norm.autoscale(task_space_success)
colormap = cm.RdYlGn
ax.scatter3D(task_space_goals_cartesian[0,:], task_space_goals_cartesian[1,:], task_space_goals_cartesian[2,:], color = colormap(norm(task_space_success)), alpha = .8, lw = 1, s = 5)
ax.set_xlim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
ax.set_ylim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
ax.set_zlim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
ax.set_xlabel('$X$', fontsize=15, rotation=150)
ax.set_ylabel('$Y$', fontsize=15)
ax.set_zlabel('$Z$', fontsize=15, rotation=60)
ax.set_title('Success of Task Space Target Positions')
ax.view_init(elev=5, azim=110)
#fig.savefig(sys.argv[1] + "/Graphs" + "/tf" + "_" + "End_Effector_Position_in_Base_Frame_Colored_with_Time" + ".png", dpi = 300, bbox_inches = 'tight')


# Check the buffer space
# Create buffer space boundaries
# rad_min, rad_max
# horizontal_min, horizontal_max
# vertical_min, vertical_max
# Along axis 2 are the different boundaries for each thin buffer space volume,
# which is essentially a "thick" surface on top of each task space face
buffer_space_diffs = np.array([buffer_space_rad_diff_ft, buffer_space_horizontal_angle_diff_deg, buffer_space_vertical_angle_diff_deg])
buffer_space_boundaries = np.tile(task_space_boundaries, (1, 1, task_space_boundaries.flatten().shape[0]))
buffer_space_boundaries = np.dstack((task_space_boundaries, task_space_boundaries, task_space_boundaries, task_space_boundaries, task_space_boundaries, task_space_boundaries))
for ii in range(buffer_space_boundaries.shape[2]):
    if ii%2 == 0:
        # We are in the "min" column
        buffer_space_boundaries[ii//2,0,ii] = task_space_boundaries[ii//2,0] - buffer_space_diffs[ii//2]
        buffer_space_boundaries[ii//2,1,ii] = task_space_boundaries[ii//2,0]
    else:
        # We are in the "max" column
        buffer_space_boundaries[ii//2,0,ii] = task_space_boundaries[ii//2,1] 
        buffer_space_boundaries[ii//2,1,ii] = task_space_boundaries[ii//2,1] + buffer_space_diffs[ii//2]
# Decide which points will lie on particular boundaries
buffer_perimeter_choices = np.random.choice([-1,0,1], size = (buffer_space_boundaries.shape[0], round(monte_carlo_runs * buffer_space_run_ratio), buffer_space_boundaries.shape[2]), p = [perimeter_percentage/2, 1-perimeter_percentage, perimeter_percentage/2])
# Create array to hold buffer space target goals in spherical coordinates
buffer_space_goals_spherical = np.zeros(buffer_perimeter_choices.shape)
# Add "min" perimeter values
buffer_perimeter_choices_worker = np.copy(buffer_perimeter_choices)
buffer_perimeter_choices_worker[buffer_perimeter_choices > 0] = 0
for ii in range(buffer_perimeter_choices_worker.shape[2]):
    buffer_space_goals_spherical[:,:,ii] = buffer_space_goals_spherical[:,:,ii] + np.matmul(np.diag(buffer_space_boundaries[:,0,ii]), (buffer_perimeter_choices_worker[:,:,ii] * -1))
# Add "max" perimeter values
buffer_perimeter_choices_worker = np.copy(buffer_perimeter_choices)
buffer_perimeter_choices_worker[buffer_perimeter_choices < 0] = 0
for ii in range(buffer_perimeter_choices_worker.shape[2]):
    buffer_space_goals_spherical[:,:,ii] = buffer_space_goals_spherical[:,:,ii] + np.matmul(np.diag(buffer_space_boundaries[:,1,ii]), buffer_perimeter_choices_worker[:,:,ii])
# Add random perimeter values
buffer_perimeter_choices_worker = np.copy(buffer_perimeter_choices)
buffer_perimeter_choices_worker = np.abs(np.abs(buffer_perimeter_choices_worker) - 1)
for ii in range(buffer_perimeter_choices_worker.shape[2]):
    buffer_space_goals_spherical[:,:,ii] = buffer_space_goals_spherical[:,:,ii] + np.add(np.matmul(np.diag(buffer_space_boundaries[:,1,ii] - buffer_space_boundaries[:,0,ii]), np.random.rand(buffer_perimeter_choices_worker.shape[0], buffer_perimeter_choices_worker.shape[1])), np.repeat(buffer_space_boundaries[:,0,ii].reshape(-1,1), buffer_perimeter_choices_worker.shape[1], axis = 1)) * buffer_perimeter_choices_worker[:,:,ii]
# Convert from spherical coordinates to cartesian coordinates
buffer_space_goals_cartesian = np.zeros(buffer_space_goals_spherical.shape)
buffer_space_goals_cartesian[0,:,:] = buffer_space_goals_spherical[0,:,:] * np.cos(buffer_space_goals_spherical[1,:,:]*2*np.pi/360) * np.cos(buffer_space_goals_spherical[2,:,:]*2*np.pi/360)
buffer_space_goals_cartesian[1,:,:] = buffer_space_goals_spherical[0,:,:] * np.sin(buffer_space_goals_spherical[1,:,:]*2*np.pi/360) * np.cos(buffer_space_goals_spherical[2,:,:]*2*np.pi/360)
buffer_space_goals_cartesian[2,:,:] = buffer_space_goals_spherical[0,:,:] * np.cos(buffer_space_goals_spherical[1,:,:]*2*np.pi/360) * np.sin(buffer_space_goals_spherical[2,:,:]*2*np.pi/360) * -1
# Check each point for IK success
buffer_space_success = []
for run_num in range(buffer_perimeter_choices.shape[1]):
    for buffer_surface_num in range(buffer_perimeter_choices.shape[2]):
        buffer_space_success.append(IK_runner(buffer_space_goals_cartesian[:, run_num, buffer_surface_num]))
# Plot buffer space results, colored for success
fig = plt.figure()
ax = plt.axes(projection='3d')
norm = Normalize()
norm.autoscale(buffer_space_success)
colormap = cm.RdYlGn
ax.scatter3D(buffer_space_goals_cartesian[0,:,:], buffer_space_goals_cartesian[1,:,:], buffer_space_goals_cartesian[2,:,:], color = colormap(norm(buffer_space_success)), alpha = .8, lw = 1, s = 5)
ax.set_xlim(np.min(buffer_space_goals_cartesian)*1.1, np.max(buffer_space_goals_cartesian)*1.1)
ax.set_ylim(np.min(buffer_space_goals_cartesian)*1.1, np.max(buffer_space_goals_cartesian)*1.1)
ax.set_zlim(np.min(buffer_space_goals_cartesian)*1.1, np.max(buffer_space_goals_cartesian)*1.1)
ax.set_xlabel('$X$', fontsize=15, rotation=150)
ax.set_ylabel('$Y$', fontsize=15)
ax.set_zlabel('$Z$', fontsize=15, rotation=60)
ax.set_title('Success of Buffer Space Target Positions')
ax.view_init(elev=5, azim=110)
#fig.savefig(sys.argv[1] + "/Graphs" + "/tf" + "_" + "End_Effector_Position_in_Base_Frame_Colored_with_Time" + ".png", dpi = 300, bbox_inches = 'tight')


# Check the stabilization task space
# Create a series of homogeneous matrices with which to transform the task space points
stable_space_rot_diffs = np.array([stable_space_horizontal_max_deg, 0, stable_space_horizontal_min_deg])
stable_space_trans_diffs = np.array([[stable_space_x_max_ft, stable_space_x_min_ft],[stable_space_y_max_ft, stable_space_y_min_ft],[stable_space_z_max_ft, stable_space_z_min_ft]])
H_matrices = np.dstack((np.identity(4),np.identity(4),np.identity(4),np.identity(4),np.identity(4),np.identity(4)))
H_matrices = np.dstack((H_matrices, H_matrices, H_matrices))
for ii in range(H_matrices.shape[2]):
    H_matrices[0:3,0:3,ii] = R.from_euler('z', stable_space_rot_diffs[ii//(H_matrices.shape[2]//stable_space_rot_diffs.shape[0])], degrees=True).as_matrix()
    translation = np.zeros(3)
    translation[ii%3] = stable_space_trans_diffs[ii%3, ii%2]
    H_matrices[0:3,3,ii] = translation
for ii in range(stable_space_rot_diffs.shape[0]):
    H_matrices_worker = np.identity(4)
    H_matrices_worker[0:3,0:3] = R.from_euler('z', stable_space_rot_diffs[ii], degrees=True).as_matrix()
    H_matrices = np.dstack((H_matrices, H_matrices_worker))
# Extract a subset of the goal points to transform
task_space_goals_cartesian_subset = np.copy(task_space_goals_cartesian[:,::round(1/stable_space_run_ratio)])
stable_task_space_goals_cartesian = np.zeros((4, H_matrices.shape[2] * task_space_goals_cartesian_subset.shape[1]))
# Transform and check for IK success
stable_task_space_success = []
for H_num in range(H_matrices.shape[2]):
    for goal_num in range(task_space_goals_cartesian_subset.shape[1]):
        stable_task_space_goals_cartesian[:, goal_num * H_matrices.shape[2] + H_num] = np.matmul(H_matrices[:,:,H_num], np.append(task_space_goals_cartesian_subset[:,goal_num], [1], 0))
        stable_task_space_success.append(IK_runner(stable_task_space_goals_cartesian[:, goal_num * H_matrices.shape[2] + H_num], origin_point = task_space_goals_cartesian_subset[0:3, goal_num], stable_waypoints = True))
# Plot task space stability results, colored for success
fig = plt.figure()
ax = plt.axes(projection='3d')
norm = Normalize()
norm.autoscale(stable_task_space_success)
colormap = cm.RdYlGn
ax.scatter3D(stable_task_space_goals_cartesian[0,:], stable_task_space_goals_cartesian[1,:], stable_task_space_goals_cartesian[2,:], color = colormap(norm(stable_task_space_success)), alpha = .8, lw = 1, s = 5)
ax.set_xlim(np.min(stable_task_space_goals_cartesian)*1.1, np.max(stable_task_space_goals_cartesian)*1.1)
ax.set_ylim(np.min(stable_task_space_goals_cartesian)*1.1, np.max(stable_task_space_goals_cartesian)*1.1)
ax.set_zlim(np.min(stable_task_space_goals_cartesian)*1.1, np.max(stable_task_space_goals_cartesian)*1.1)
ax.set_xlabel('$X$', fontsize=15, rotation=150)
ax.set_ylabel('$Y$', fontsize=15)
ax.set_zlabel('$Z$', fontsize=15, rotation=60)
ax.set_title('Success of Stable Task Space Target Positions')
ax.view_init(elev=5, azim=110)
#fig.savefig(sys.argv[1] + "/Graphs" + "/tf" + "_" + "End_Effector_Position_in_Base_Frame_Colored_with_Time" + ".png", dpi = 300, bbox_inches = 'tight')


# Check the stabilization buffer space
# Extract a subset of the goal points to transform
buffer_space_goals_cartesian_subset = np.copy(buffer_space_goals_cartesian.reshape(3,-1)[:,::round(1/stable_space_run_ratio)])
stable_buffer_space_goals_cartesian = np.zeros((4, H_matrices.shape[2] * buffer_space_goals_cartesian_subset.shape[1]))
# Transform and check each point for IK success
stable_buffer_space_success = []
for H_num in range(H_matrices.shape[2]):
    for goal_num in range(buffer_space_goals_cartesian_subset.shape[1]):
        stable_buffer_space_goals_cartesian[:, goal_num * H_matrices.shape[2] + H_num] = np.matmul(H_matrices[:,:,H_num], np.append(buffer_space_goals_cartesian_subset[:,goal_num], [1], 0))
        stable_buffer_space_success.append(IK_runner(stable_buffer_space_goals_cartesian[:, goal_num * H_matrices.shape[2] + H_num], origin_point = buffer_space_goals_cartesian_subset[0:3, goal_num], stable_waypoints = True))
# Plot buffer space stability results, colored for success
fig = plt.figure()
ax = plt.axes(projection='3d')
norm = Normalize()
norm.autoscale(stable_buffer_space_success)
colormap = cm.RdYlGn
ax.scatter3D(stable_buffer_space_goals_cartesian[0,:], stable_buffer_space_goals_cartesian[1,:], stable_buffer_space_goals_cartesian[2,:], color = colormap(norm(stable_buffer_space_success)), alpha = .8, lw = 1, s = 5)
ax.set_xlim(np.min(stable_buffer_space_goals_cartesian)*1.1, np.max(stable_buffer_space_goals_cartesian)*1.1)
ax.set_ylim(np.min(stable_buffer_space_goals_cartesian)*1.1, np.max(stable_buffer_space_goals_cartesian)*1.1)
ax.set_zlim(np.min(stable_buffer_space_goals_cartesian)*1.1, np.max(stable_buffer_space_goals_cartesian)*1.1)
ax.set_xlabel('$X$', fontsize=15, rotation=150)
ax.set_ylabel('$Y$', fontsize=15)
ax.set_zlabel('$Z$', fontsize=15, rotation=60)
ax.set_title('Success of Stable Buffer Space Target Positions')
ax.view_init(elev=5, azim=110)
#fig.savefig(sys.argv[1] + "/Graphs" + "/tf" + "_" + "End_Effector_Position_in_Base_Frame_Colored_with_Time" + ".png", dpi = 300, bbox_inches = 'tight')


# Show all plots.
plt.show(block=False)
plt.pause(0.001)
input("hit [enter] to close all plots and end the program.")
plt.close('all')