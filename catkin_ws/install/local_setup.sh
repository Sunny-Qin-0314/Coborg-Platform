#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD:franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/local_setup.sh
: ${_CATKIN_SETUP_DIR:=/home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel}
=======
: ${_CATKIN_SETUP_DIR:=/home/yuqing/Desktop/Coborg-Platform/catkin_ws/install}
>>>>>>> 0ad6f6b835c25920092c265e79cc9ef3ac4c9b72:catkin_ws/install/local_setup.sh
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
