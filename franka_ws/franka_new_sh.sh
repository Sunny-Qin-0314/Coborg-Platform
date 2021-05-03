#!/bin/bash
# Sourcing & enter virtual environment
cd ~/Coborg-Platform/franka_ws
source ./envs/franka/bin/activate
source ./ws/frankapy-public/catkin_ws/devel/setup.bash
source ./ws/robot-autonomy-labs/lab3/cv_bridge_ws/devel/setup.bash --extend
source ./ws/aruco_ros_multi/devel/setup.bash
