# Prequisities

* ROS Melodic
* MoveIt
* realsense2_camera

# Configuration

Import the folders in the 'motion demo' folder to the catkin_ws src folder. Verify you have the prerequisities installed on your system. On the catkin_ws folder, type:

~~~
catkin_make
~~~

# How to Run

~~~
roslaunch realsense2_camera rs_d400_and_t265.launch
roslaunch coborg_arm demo.launch
~~~

To perform a command-line approach to tie the frames of the realsense cameras and coborg toger, type:

~~~
rosrun tf static_transform_publisher 0 0 0 0 0 0 \t265_odom_frame \world 100
~~~