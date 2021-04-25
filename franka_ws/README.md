franka_ws is a catkin workspace for the robot autonomy project
======

First Time Install (for iam-bashful):
------

Ensure that the franka robot is on, and that you are connected to it via an ethernet port. Follow the instructions found in the `/tutorial` folder `Franka-Lab1.pdf`

These commands are relative to this path:\
cd to `Coborg-Platform/franka_ws`

Run this script to catkin_make and pip install all dependencies then start the robot control (does not include camera install dependencies yet Gerry):\
`source franka_setup.sh`


Running the Project (for iam-bashful):
------

Once the initial setup completes you no longer need to make the environments and can run the robot interface using this script:\
`source franka_startup.sh`

Now follow these steps to run the program:

1. cd to `franka_ws/ws/project-TeamC`(this step should be added to the franka_startup script Gerry):\
run `source ../aruco_ros_multi/devel/setup.bash`\
run `roslaunch aruco_ros_multi aruco.launch`

2. Open a new terminal instance in the franka virtual environment with the correct catkin directories sourced:\
cd to `Coborg-Platform/franka_ws`\
run `source franka_new_sh.sh`

3. run the main script in the franka virtual environment shell:
cd to `franka_ws/ws/project-TeamC/scripts`\
run `python main.py`