#!/bin/bash
# Sourcing & enter virtual environment
cd ~/Coborg-Platform/franka_ws
source ./envs/franka/bin/activate
source ./ws/frankapy-public/catkin_ws/devel/setup.bash
source ./ws/robot-autonomy-labs/lab3/cv_bridge_ws/devel/setup.bash --extend
#source ./ws/aruco_ros_multi/devel/setup.bash

# Open firefox on control PC
nohup ssh -X student@iam-bashful "firefox -new-tab https://172.16.0.2/desk/" >/dev/null 2>&1 &

read -p "Press ENTER when joints are unlocked (Lights are WHITE)..."
read -p "Press ENTER when E-STOP is un-engaged (Lights are BLUE)..."
bash ws/frankapy-public/bash_scripts/start_control_pc.sh -i iam-bashful >/dev/null 2>&1

bash ws/project-teamC/bash_scripts/start_azure_kinect.sh -i iam-bashful >/dev/null 2>&1
source ./ws/aruco_ros_multi/devel/setup.bash
roslaunch aruco_ros_multi aruco.launch &
