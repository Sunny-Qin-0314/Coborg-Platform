execute_process(COMMAND "/home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/build/vision_opencv/image_geometry/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/build/vision_opencv/image_geometry/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
