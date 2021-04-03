arduino_ws is a catkin workspace for the sensors and motors lab
======

Arduino:
------
In "Coborg-Platform/arduino_ws/src/arduino_files" is the main.ino for the arduino.

Point your arduino IDE sketch folder to "Coborg-Platform/arduino_ws/src/arduino_files" to be able to compile and upload main.ino. This allows it to see the libraries folder which contains the PID and ros_lib libraries.

GUI:
------
To run the ROS package for the gui, follow these steps:\
`cd Coborg-Platform/arduino_ws`\
`catkin_make install`\
`source devel/setup.bash`

Now we have to recompile the message library for the (jank) rosserial package to work:\
`cd Coborg-Platform/arduino_ws/src/arduino_files/libraries`\
`rm -rf ros_lib`\
`rosrun rosserial_arduino make_libraries.py .` <- that period is important. include it in the command.

You should be ready to launch the gui. plug in the arduino to a usb port and run:\
`sudo chmod a+rw /dev/ttyACM0`\
`roslaunch cmu_motor_lab demo.launch`
 
The demo.launch file runs these components:
* joint_state publisher + GUI sliders
* robot_state publisher
* rosserial python to communicate with the arduino

rqt with a custom perspective for this project:
* rqt_plot of the custom CMU message to read the sensor inputs
* RVIZ
	* The rqt_rviz plugin may not automatically load the urdf.rviz perspective file.\
	To do this manually, click on "file/open config" tab in the RVIZ section and load\
	the file located in "cmu_motor_lab/rviz/urdf.rviz".\
	You may have to pull out the USB plug to allow the "open file" prompt to pop up.
	
* If for some reason rqt does not load it's custom perspective, click on perspective/import\
from the top drop down list and load the file "cmu_motor_lab/rqt/motor_lab.perspective".

Additional information:
------
rosserial install info:
[http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

Note: under "src/rosserial/rosserial_arduino/msg" is the custom message type "CMU.msg"
Ensure that this file is present when you run catkin_make to allow rosserial to compile that message type.

Common ROS errors:
------
[ERROR] [1614183628.727014]: Tried to publish before configured, topic id 125

[ERROR] [1614183338.789682]: Error opening serial: [Errno 13] could not open port /dev/ttyACM0: Permission denied: '/dev/ttyACM0' 
	-run: `sudo chmod a+rw /dev/ttyACM0` to allow read/write access to the USB port

Common Arduino compile errors:
In file included from Coborg-Platform/arduino_ws/src/arduino_files/libraries/ros_lib/std_msgs/Time.h:7:0,
                 from Coborg-Platform/arduino_ws/src/arduino_files/libraries/ros_lib/ros/node_handle.h:40,
                 from Coborg-Platform/arduino_ws/src/arduino_files/libraries/ros_lib/ros.h:38,
                 from Coborg-Platform/arduino_ws/src/arduino_files/archive/adc_test/adc_test.ino:10:
Coborg-Platform/arduino_ws/src/arduino_files/libraries/ros_lib/ros/msg.h:40:10: fatal error: cstring: No such file or directory
 #include <cstring>

Basically the `rosrun rosserial_arduino make_libraries.py .` command installs a broken ros_lib library (I know. Why ROS, Why...). Copy the contents from ros_lib_backup into ros_lib and you will be able to compile the arduino code (overwrite).
