# Install script for directory: /home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/msg" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/srv" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/action" TYPE FILE FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/action/ExecuteSkill.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/msg" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/build/franka-interface-msgs/catkin_generated/installspace/franka_interface_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/include/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/roseus/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/common-lisp/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/share/gennodejs/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/lib/python2.7/dist-packages/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/devel/lib/python2.7/dist-packages/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/build/franka-interface-msgs/catkin_generated/installspace/franka_interface_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/build/franka-interface-msgs/catkin_generated/installspace/franka_interface_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/build/franka-interface-msgs/catkin_generated/installspace/franka_interface_msgsConfig.cmake"
    "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/build/franka-interface-msgs/catkin_generated/installspace/franka_interface_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs" TYPE FILE FILES "/home/coborg/Coborg-Platform/franka_ws/ws/frankapy-public/catkin_ws/src/franka-interface-msgs/package.xml")
endif()

