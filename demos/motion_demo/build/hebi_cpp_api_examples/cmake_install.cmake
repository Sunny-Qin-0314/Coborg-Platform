# Install script for directory: /home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/coborg/Coborg-Platform/demos/motion_demo/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/action" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/action/ArmMotion.action"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/action/BaseMotion.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/msg" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionAction.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionGoal.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionResult.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionActionFeedback.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionGoal.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionResult.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/ArmMotionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/msg" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionAction.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionGoal.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionResult.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionActionFeedback.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionGoal.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionResult.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/hebi_cpp_api_examples/msg/BaseMotionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/msg" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/EndPath.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/OffsetPlayback.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/Playback.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/SaveWaypoint.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/StartPath.msg"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/msg/TargetWaypoints.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/srv" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetCommandLifetime.srv"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetFeedbackFrequency.srv"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetGains.srv"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/srv/SetIKSeed.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/cmake" TYPE FILE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/build/hebi_cpp_api_examples/catkin_generated/installspace/hebi_cpp_api_examples-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/include/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/roseus/ros/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/common-lisp/ros/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/share/gennodejs/ros/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/python2.7/dist-packages/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/python2.7/dist-packages/hebi_cpp_api_examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/build/hebi_cpp_api_examples/catkin_generated/installspace/hebi_cpp_api_examples.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/cmake" TYPE FILE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/build/hebi_cpp_api_examples/catkin_generated/installspace/hebi_cpp_api_examples-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/cmake" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/build/hebi_cpp_api_examples/catkin_generated/installspace/hebi_cpp_api_examplesConfig.cmake"
    "/home/coborg/Coborg-Platform/demos/motion_demo/build/hebi_cpp_api_examples/catkin_generated/installspace/hebi_cpp_api_examplesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples" TYPE FILE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples" TYPE DIRECTORY FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/config"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/launch"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples/integrations" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/src/integrations/clearpath_jackal/jackal_arm.launch"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/src/integrations/clearpath_jackal/jackal_base_station.launch"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/src/integrations/clearpath_jackal/jackal_to_hebi.urdf"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/src/integrations/clearpath_jackal/rviz_config.rviz"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/scripts" TYPE PROGRAM FILES "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/src/integrations/clearpath_jackal/jackal_arm_teleop.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hebi_cpp_api_examples" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/launch/daisy.launch"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/data/hexapod/gains18.xml"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/data/hexapod/gains3.xml"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/data/hexapod/hex_config.xml"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/data/hexapod/left.hrdf"
    "/home/coborg/Coborg-Platform/demos/motion_demo/src/hebi_cpp_api_examples/data/hexapod/right.hrdf"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/arm_moveit_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_moveit_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/arm_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/arm_teach_repeat_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/arm_teach_repeat_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/diff_drive_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/diff_drive_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/group_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/group_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/gripper_moveit_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_moveit_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/gripper_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/gripper_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/mecanum_base_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/mecanum_base_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/omni_base_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_controller_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/omni_base_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/omni_base_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/rosie_teach_repeat_controller_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/hexapod_control")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_control")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples" TYPE EXECUTABLE FILES "/home/coborg/Coborg-Platform/demos/motion_demo/devel/lib/hebi_cpp_api_examples/hexapod_controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hebi_cpp_api_examples/hexapod_controller")
    endif()
  endif()
endif()

