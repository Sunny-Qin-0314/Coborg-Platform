# Prequisities

* ubuntu 18.04 (Bionic Beaver)
* ros-melodic-desktop-full v1.4.1-0bionic.20210304.173654
* ros-melodic-moveit v1.0.7-1bionic.20210304. 174558


* ros-melodic-realsense2-camera v2.2.22-1bionic.20210219.07850
* ros-melodic-realsense2-description v2.2.22-1bionic.20210219.071513 500
* librealsense2 v2.42.0-0~realsense0.4059
* librealsense2-dkms v1.3.16-0ubuntu1
* librealsense2-utils v2.42.0-0~realsense0.4059
* librealsense2-dev v2.42.0-0~realsense0.4059
* librealsense2-dbg v2.42.0-0~realsense0.4059

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

