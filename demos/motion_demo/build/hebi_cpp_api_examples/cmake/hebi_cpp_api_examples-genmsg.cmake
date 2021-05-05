# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hebi_cpp_api_examples: 20 messages, 4 services")

set(MSG_I_FLAGS "-Ihebi_cpp_api_examples:/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg;-Ihebi_cpp_api_examples:/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Icontrol_msgs:/opt/ros/melodic/share/control_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hebi_cpp_api_examples_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" "actionlib_msgs/GoalID:hebi_cpp_api_examples/ArmMotionFeedback:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" "actionlib_msgs/GoalID:hebi_cpp_api_examples/BaseMotionActionGoal:hebi_cpp_api_examples/BaseMotionResult:actionlib_msgs/GoalStatus:hebi_cpp_api_examples/BaseMotionFeedback:hebi_cpp_api_examples/BaseMotionActionFeedback:hebi_cpp_api_examples/BaseMotionGoal:std_msgs/Header:hebi_cpp_api_examples/BaseMotionActionResult"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:hebi_cpp_api_examples/BaseMotionFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" "actionlib_msgs/GoalID:hebi_cpp_api_examples/ArmMotionGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" "actionlib_msgs/GoalID:hebi_cpp_api_examples/BaseMotionGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" "actionlib_msgs/GoalID:hebi_cpp_api_examples/ArmMotionResult:actionlib_msgs/GoalStatus:hebi_cpp_api_examples/ArmMotionGoal:hebi_cpp_api_examples/ArmMotionFeedback:hebi_cpp_api_examples/ArmMotionActionFeedback:hebi_cpp_api_examples/ArmMotionActionGoal:std_msgs/Header:hebi_cpp_api_examples/ArmMotionActionResult"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:hebi_cpp_api_examples/BaseMotionResult:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" ""
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" "hebi_cpp_api_examples/ArmMotionResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_custom_target(_hebi_cpp_api_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hebi_cpp_api_examples" "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Services
_generate_srv_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_cpp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Module File
_generate_module_cpp(hebi_cpp_api_examples
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hebi_cpp_api_examples_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hebi_cpp_api_examples_generate_messages hebi_cpp_api_examples_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_cpp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hebi_cpp_api_examples_gencpp)
add_dependencies(hebi_cpp_api_examples_gencpp hebi_cpp_api_examples_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hebi_cpp_api_examples_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Services
_generate_srv_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_eus(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Module File
_generate_module_eus(hebi_cpp_api_examples
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hebi_cpp_api_examples_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hebi_cpp_api_examples_generate_messages hebi_cpp_api_examples_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_eus _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hebi_cpp_api_examples_geneus)
add_dependencies(hebi_cpp_api_examples_geneus hebi_cpp_api_examples_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hebi_cpp_api_examples_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Services
_generate_srv_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_lisp(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Module File
_generate_module_lisp(hebi_cpp_api_examples
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hebi_cpp_api_examples_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hebi_cpp_api_examples_generate_messages hebi_cpp_api_examples_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_lisp _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hebi_cpp_api_examples_genlisp)
add_dependencies(hebi_cpp_api_examples_genlisp hebi_cpp_api_examples_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hebi_cpp_api_examples_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Services
_generate_srv_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_nodejs(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Module File
_generate_module_nodejs(hebi_cpp_api_examples
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hebi_cpp_api_examples_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hebi_cpp_api_examples_generate_messages hebi_cpp_api_examples_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hebi_cpp_api_examples_gennodejs)
add_dependencies(hebi_cpp_api_examples_gennodejs hebi_cpp_api_examples_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hebi_cpp_api_examples_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_msg_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Services
_generate_srv_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)
_generate_srv_py(hebi_cpp_api_examples
  "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
)

### Generating Module File
_generate_module_py(hebi_cpp_api_examples
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hebi_cpp_api_examples_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hebi_cpp_api_examples_generate_messages hebi_cpp_api_examples_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg" NAME_WE)
add_dependencies(hebi_cpp_api_examples_generate_messages_py _hebi_cpp_api_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hebi_cpp_api_examples_genpy)
add_dependencies(hebi_cpp_api_examples_genpy hebi_cpp_api_examples_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hebi_cpp_api_examples_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hebi_cpp_api_examples
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET control_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp control_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hebi_cpp_api_examples
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET control_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus control_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(hebi_cpp_api_examples_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hebi_cpp_api_examples
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET control_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp control_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(hebi_cpp_api_examples_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hebi_cpp_api_examples
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET control_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs control_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(hebi_cpp_api_examples_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hebi_cpp_api_examples
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET control_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py control_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(hebi_cpp_api_examples_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
