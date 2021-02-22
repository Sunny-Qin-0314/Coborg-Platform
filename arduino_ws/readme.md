This is a catkin workspace for the sensors and motors lab.

In arduino_ws/src/arduino_files is the main.ino for running the sensors and motors lab.

Point your arduino IDE to look at the libraries folder at arduino_ws/src/arduino_files/libraries to be able to compile and upload main.ino

to run the ROS package for the gui, follow these steps:

cd to arduino_ws
run: catkin_make install
run: source devel/setup.bash

now you should be able to run packages under this directory. Now you should be able to run the launch file that compiles and runs these components:

rqt with a custom perspective for this project:
	-RVIZ with a URDF loaded
	-Joint_State GUI
	-rosserial python to communicate with the arduino
		-don't forget to run: sudo chmod a+rw /dev/ttyACM0 to 			allow read/write access to the USB port

 	-rqt_plot of the custom CMU message to read the sensor inputs

rosserial install info:
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Note: under src/rosserial/rosserial_arduino/msg is the custom message type CMU.msg

Ensure that this file is present when you run catkin_make to allow rosserial to compile that message type.

