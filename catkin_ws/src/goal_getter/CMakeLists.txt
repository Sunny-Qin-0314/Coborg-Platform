cmake_minimum_required(VERSION 3.0.2)
project(goal_getter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gb_visual_detection_3d_msgs
  message_generation
  roscpp
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   goal_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES follow_line
#  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_getter src/goal_getter.cpp)
add_dependencies(goal_getter ${catkin_EXPORTED_TARGETS})
add_dependencies(goal_getter goal_getter_generate_messages_cpp)
target_link_libraries(goal_getter ${catkin_LIBRARIES})
