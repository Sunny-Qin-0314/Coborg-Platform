## within src/darknet_ros/darnet_ros/

Note: check whether the darknet has /data folder, if not, you may need to re-download the repo from darknet_ros, and change the config, launch files as mentioned below

Note: make sure you have weights file in the /yolo_network_config

/yolo_network_config:

0. download the weights and cfg from /download.sh
1. add .cfg and .weights (hand detection model weights and configs)
2. within /cfg, run "dos2unix cross-hands.cfg" (convert it to unix format)

/config:

1. modify "ros.yaml" with correct camera topic
2. create "yolov3-hand.yaml" to configure the model files

/launch:

1. modify "darknet_ros.launch" with correct yaml file (yolov3-hand.yaml)



## within src/gb_visual_detection_3d/darknet_ros_3d

Note: download the repo and checkout to "melodic" branch!!
Note: download msg repo and checkout to "melodic" branch before catkin_make!!
Note: if "catkin_make" doesn't work, try "catkin_make -j1"

/config:
1. modify "darknet_3d.yaml" with correct camera depth-registed pointcloud topic and "interested_classes" ("hand")

/launch:
1. modify "darknet_ros_3d.launch" with correct yaml file (yolov3-hand.yaml)



## to run on GPU: needs more than 4GB GPU memory

### version: cuda 10.2 + cudnn 7.6.5 (not support cudnn8!!!!!!!!!!!!!)

### install cuda 10.2:
Note: if there is any usr/local/cuda directory, remove it before re-install
Note: no need to set up driver, when install cuda, it will set up driver automatically
Note: the libcudnn7 is installed in the /usr/local/cuda-10.2, if libcudnn exists in /cuda-10.2, copy to /usr/local/cuda/lib64, also copy cudnn.h to /cuda/include as well.

https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130

check cuda version:
nvcc -V


### install cudnn separately:
Goto page https://developer.nvidia.com/rdp/cudnn-download
Download all three .deb: runtime/developer/code sample

$ sudo dpkg -i libcudnn7_7.6.5.32–1+cuda10.1_amd64.deb (the runtime library),

$ sudo dpkg -i libcudnn7-dev_7.6.5.32–1+cuda10.1_amd64.deb (the developer library), and

$ sudo dpkg -i libcudnn7-doc_7.6.5.32–1+cuda10.1_amd64.deb (the code samples).

!! copy cudnn.h (in usr/include) to (usr/local/cuda/include)
!! copy libcudnn* (in usr/lib/x86_...) to (/usr/local/cuda/lib64/)

### modify makefile 
1. change Makefile in /darnet_ros/darknet
   GPU = 1
   CUDNN =1
   OPENCV = 1
   -gencode (75)

2. run "make" in /darknet_ros/darknet

3. change CmakeList.txt in /darknet_ros/darknet_ros
   -gencode = 75

4. run "catkin_make -DCMAKE_BUILD_TYPE=Release" in /catkin_ws

GPU is ready!

## Modify the rs_d400_and_t265.launch
Add rs_rgbd.launch into rs_d400_and_t265.launch

Note: install rgbd_launch package first

$ sudo apt-get install ros-melodic-rgbd-launch

modify the tf based on the urdf for d435 and t265
xyz="0.009 0.021 0.027" rpy="0.000 -0.018 0.005"

To use it, move it into your /opt/ros/melodic/share/realsense2_camera/launch

## Run the program

1. roslaunch realsense2_camera rs_d400_and_t265.launch 
   (modified d400 and t265 launch file (combined rs_rgbd.launch and rs_d400_and_t265.launch)) or use rs_rgbd.launch with D435i only

2. catkin_make   

   if not working, try "catkin_make -j1"

   source devel/setup.bash

3. roslaunch darknet_ros_3d darknet_ros_3d.launch
   (make sure configure file and launch file have already been modified with correct image topics and YOLO weights, configs)

4. rosrun goal_getter goal_getter
   (publish 3D position to /goal topic)
