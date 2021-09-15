---
# Jekyll 'Front Matter' goes here. Most are set by default, and should NOT be
# overwritten except in special circumstances. 
# You should set the date the article was last updated like this:
date: 2021-09-14 # YYYY-MM-DD
# This will be displayed at the bottom of the article
# You should set the article's title:
title: Actuated Manipulation System Setup Readme
# The 'title' is automatically displayed at the top of the page
# and used in other parts of the site.
---

## Overview

In this readme, we will do the following things for the actuated manipulation system:

- Install and set up actuated manipulation system
- Run randomPoseGenerator for first SVD performance
- Run voicePoseGenerator for SVD encore performance



## Setting up actuated manipulation system

### Requirements

- Ubuntu: 18.04

- Ubuntu: librealsense2 (v2.42.0.0-realsense0.4059)

- ROS: Melodic (v1.4.1-0bionic.20210304.173654)

- ROS: Realsense2-camera (v2.2.22-1bionic.20210219.07850)

- ROS: Realsense2-description (v2.2.22-1bionic.20210219.071513 500)


### Steps
1. #### Download the repo from the following link:

[CoBorg-Platform GitHub Link](https://github.com/Sunny-Qin-0314/Coborg-Platform/tree/devel_arm64)

Run the following commands to clone the repository and go to the appropriate branch:

```
git clone https://github.com/Sunny-Qin-0314/Coborg-Platform.git
cd CoBorg-Platform
git checkout devel_arm64
cd catkin_ws
```

2. #### Installing realsense-ros and librealsense:

To install realsense-ros, download the repo from:

[realsense-ros GitHub link](https://github.com/IntelRealSense/realsense-ros)

Following Method 2 of the authors' installation process to install the realsense-ros node from the catkin_ws ./src folder.

librealsense is a linux package that needs to be installed separate from the realsense-ros node. To install librealsense on Ubuntu, follow the authors' instructions from the librealsense repo:

[librealsense Linux installation GitHub webpage](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

To run RealSense Viewer, run the following command from terminal:

```realsense-viewer```

To check the RealSense devices connected to the local machine, run the following command from terminal:

```rs-enumerate-devices```

NOTE: As of the creation of this README file, Intel has discontinued sale and support of their Realsense cameras. You ought to remove any Realsense repositories from your sources.list file. Refer to [how to remove malformed line from sources list](https://askubuntu.com/questions/78951/how-do-i-remove-a-malformed-line-from-my-sources-list) for details on how to remove repositories from update list on Ubuntu.

3. #### Build:
Delete any ./build or ./devel folders if present in the catkin_ws folder. Install ros dependencies using the ```rosdep``` command.

```
cd CoBorg-Platform/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make darknet_ros
catkin_make darknet_ros_3d
catkin_make install
source devel/setup.bash
```

NOTE: Will need to catkin_make the darknet_ros node separately first before running catkin_make for the reset of the nodes. This will avoid any unexpected errors from occurring.

# How to Run

There are various "modes" to run the actuated manipulation system, and are worth noting here:

1. Manual mode
2. Preset target mode (first SVD)
3. Integrated, voice-activated mode (SVD encore)

## Manual Mode

Manual mode involves the user manually quering goal poses to the robot URDF through the ROS visualization tool, RViz. Run the following commands to activate manual mode:

```
roslaunch coborg_arm demo.launch
roslaunch coborg_move find_hebi_moveit_planner.launch
```

Querying a goal pose through RViz is as easy as drag-and-dropping the queried end-effector pose (colored in orange through RViz). You can also set multiple preset positions through Rviz, selecting "Plan" and selecting "Execute" once a successful plan is created.

## Preset Target Mode

Preset target mode contains multiple preset target goal poses that are out and in-front of the front of the robot arm. This code was compile and run during the first SVD event and was used to command the robot arm to move through various intermediate positions and goal positions on the bord in front of the robot. Run the following commands to activate preset target mode (this will require multiple terminal windows to be open):

```
source devel/setup.bash
roslaunch coborg_move demo_hebi_realsense_tf.launch
```

In another terminal window, run the following commands:

```
source devel/setup.bash
roslaunch coborg_move tf_moveit_goalsetNode.launch
```

In another terminal window, run the following commands:

```
source devel/setup.bash
roslaunch coborg_move randomGoalPoseGenerator.launch
```

Before running the next node, manually set the robot arm to its home/tucked position through manually querying that pose through RViz. In another terminal window, run the following commands:

```
source devel/setup.bash
roslaunch coborg_move find_hebi_moveit_planner.launch
```

To operate this actuated manipulation mode, there are two rosparams that must be configured: svdTarget and manipulation_state. The svdTarget rosparam sets the goal target pose for the robot arm. The manipulation_state rosparam sets the state triggers of the robot arm. 

The options for the svdTarget rosparam are:

- target1
- target2
- target3

To set the value for the svdTarget rosparam, run the following command structure (ensure that you have performed source devel/setup.bash in the terminal):

```
rosparam set /randomGoalPoseGenerator/svdTarget <value>
```

The options for the manipulation_state rosparam are:

1. home
2. ready
3. push-out
4. out-disengage
5. push-up
6. out-disengage

To set the value for the manipulation_state rosparam, run the follow command structure (ensure that you have performed source devel/setup.bash in the terminal):

```
rosparam set /tf_moveit_goalsetNode/manipulation_state <value>
```

The general flow of the robot arm during this mode is to first start at the home position by setting the manipulation_state rosparam to "home". Once robot arm is set to the "home" preset position (i.e. compact/tucked position), the user will set the manipulation_state to "ready". The ready state will partly extend the robot arm outwards to prepare it for outward goal poses. Once the robot arm is set to the "ready" preset position, the user can set the svdTarget position. When the svdTarget position is set, the user can command the robot to to move to the target position through setting the manipulation_state to "push-out" (assuming that all target positions are out and in front of the robot arm). Once the robot arm has reached to the target position or pushed against the target part, the user can command the robot arm to retract back by changing the manipulation_state to "out-disengage". At the end of this sequence, the robot arm will contract to its home/tucked position, and will be ready for the next command.

The command structure of the flow is shown below:

```
# set robot arm to home to begin
rosparam set /tf_moveit_goalsetNode/manipulation_state home

# set robot arm to ready
rosparam set /tf_moveit_goalsetNode/manipulation_state ready

# set the target position of the robot arm (ex. svd target 01)
rosparam set /randomGoalPoseGenerator/svdTarget target1

# send the robot arm to the goal pose
rosparam set /tf_moveit_goalsetNode/manipulation_state push-out

# bring the robot arm back to its home position from the goal position
rosparam set /tf_moveit_goalsetNode/manipulation_state out-disengage
```


# Miscellaneous Topics

## Converting HRDF to URDF
Note: HRDF file is created using HEBI's 3D CAD tool. 

In the terminal, navigate to the hebi_description source folder. You will need to run the generate_pipeline.bash tool to convert HRDF to XACRO and SDF.

'''
./scripts/generate_pipeline.bash <HRDF file> <space deliminted desired names of actuator motors>
'''

The SDF file will be saved in the models/ folder. The URDF file will be saved in the urdf/kits/ folder. 
  
NOTE: the bash script assumes that there is a gripper at the end of the linkage arm. Would need to do some manual configuration of the XACRO file to clean up the code and add additional components to the URDF model.

NOTE: when observing the kinematic chain of motors and linkages as described in the xacro file, keep in mind that reporting twist angles are origin-based from the previous motor / joint angle, rather than a global angle origin. 

## References
- [ROS MoveIt Tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)

