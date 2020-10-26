sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-aruco
sudo apt-get install ros-kinetic-aruco-detect
sudo apt-get install ros-kinetic-aruco-ros
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-kinetic-usb-cam-hardware
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
sudo apt-get update
sudo apt-get install ros-kinetic-ddynamic_reconfigure
pip install --user hebi-py
pip3 install --user hebi-py
catkin_init_workspace
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

source devel/setup.bash
chmod +x src/aruco_detect_ros/src/getposes.py
chmod +x src/nn_detect_hand/src/get_hand_poses.py
cd src/nn_detect_hand/src/include/
source start.sh
cd ../../../..