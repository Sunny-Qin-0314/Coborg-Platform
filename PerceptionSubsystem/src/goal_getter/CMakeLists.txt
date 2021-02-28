cmake_minimum_required(VERSION 2.8.3)
project(goal_getter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  darknet_ros_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   goal_3d.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_getter src/goal_getter.cpp)
add_dependencies(goal_getter ${catkin_EXPORTED_TARGETS})
add_dependencies(goal_getter goal_getter_generate_messages_cpp)
add_dependencies(goal_getter darknet_ros_msgs_generate_messages_cpp)
target_link_libraries(goal_getter ${catkin_LIBRARIES} )
