# Prequisities

* ROS Melodic
* MoveIt
* realsense2_camera

# Configuration

Import the folders in the 'motion demo' folder to an existing catkin_ws src folder. 

Starting at the high level catkin_ws directory, install dependencies of the packages:

~~~
source devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
~~~


# How to Run

~~~
roslaunch realsense2_camera rs_d400_and_t265.launch
roslaunch coborg_arm demo.launch
~~~

To perform a command-line approach to tie the frames of the realsense cameras and coborg toger, type:

~~~
rosrun tf static_transform_publisher 0.35 -0.5 -0.5 0 0 0 \t265_link \world 100
~~~

On the RViz program, select Add -> By display type. Add TF. On the RViz window, go to Global Options -> Fixed Frame and change to t265_odom_frame


Optional: To see T265 vector frame and D435i depth map

Add the t265->odom->sample->Odometry frame. 

Add the d400->depth->color->points->PointCloud2. 

