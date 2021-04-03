#!/bin/bash
# Sourcing & enter virtual environment
cd ~/Coborg-Platform/franka_ws
source ./envs/franka/bin/activate
cd ws/frankapy-public
pip install -e .
./bash_scripts/make_catkin.sh
source ./catkin_ws/devel/setup.bash
cd ../..

# Open firefox on control PC
nohup ssh -X student@iam-bashful "firefox -new-tab https://172.16.0.2/desk/" >/dev/null 2>&1 &

read -p "Press ENTER when joints are unlocked (Lights are WHITE)..."
read -p "Press ENTER when E-STOP is un-engaged (Lights are BLUE)..."
bash ws/frankapy-public/bash_scripts/start_control_pc.sh -i iam-bashful >/dev/null 2>&1
