#!/bin/bash
# Sourcing & enter virtual environment
sudo apt-get install python3-tk python3-empy
pip3 install rospkg numpy
cd ~/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source ~/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_ws/devel/setup.bash --extend
cd /opt/ros/melodic/lib/python2.7/dist-packages
sudo mv cv_bridge cv_bridge_2.7

cd ~/Coborg-Platform/franka_ws
source ./envs/franka/bin/activate
cd ws/frankapy-public
pip install -e .
./bash_scripts/make_catkin.sh
source ./catkin_ws/devel/setup.bash
cd ../..
cd ws/perception
pip install -e .
cd ../..


# Open firefox on control PC
nohup ssh -X student@iam-bashful "firefox -new-tab https://172.16.0.2/desk/" >/dev/null 2>&1 &

read -p "Press ENTER when joints are unlocked (Lights are WHITE)..."
read -p "Press ENTER when E-STOP is un-engaged (Lights are BLUE)..."
bash ws/frankapy-public/bash_scripts/start_control_pc.sh -i iam-bashful >/dev/null 2>&1

bash ws/project-teamC/bash_scripts/start_azure_kinect.sh -i iam-bashful >/dev/null 2>&1
source ./ws/aruco_ros_multi/devel/setup.bash
roslaunch aruco_ros_multi aruco.launch &

