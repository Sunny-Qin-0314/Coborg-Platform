# HEBI MoveIt Configurations

(Note -- this has only been tested on Melodic)

This repository provides MoveIt configurations for HEBI standard arm kits.  These are designed for and have been tested with the HEBI MoveIt node in the `hebi_cpp_api_examples` package ( https://github.com/HebiRobotics/hebi_cpp_api_ros_examples/ ).

## Running the HEBI MoveIt examples

You must have the following repositories or packages in your catkin workspace:

- `hebi_cpp_api_examples` ( https://github.com/HebiRobotics/hebi_cpp_api_ros_examples/ ).
- `hebi_description` ( https://github.com/HebiRobotics/hebi_description/ ).
- `hebi_moveit_configs` ( https://github.com/HebiRobotics/hebi_moveit_configs/ ).

After running `catkin make`, first modify the parameters in the `moveit_arm_node<configuration>.launch` file to match your robot (e.g., module names, desired gains, etc).  In particular, ensure that the modules you are trying to control have matching name and family properties to those in the launch file (these properties can be set using the Scope tool found at http://docs.hebi.us ).  Then run:

```
roslaunch hebi_cpp_api_ros_examples moveit_arm_node<configuration>.launch
```

## Creating a MoveIt configration for a custom robot from HEBI components

For a custom system created from HEBI components, you can follow the same steps we used for creating these configurations to ensure the resulting configuration will work with the HEBI MoveIt node.  First, create a .xacro file for your custom robot (using the components/examples/documentation in https://github.com/HebiRobotics/hebi_description ), and then follow the instructions below.

Note -- this closely follows the [official MoveIt tutorial](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html), and this should be referenced for more detailed information.

### Launching the setup assistant

Before running this, ensure you have sourced the devel/setup.sh file of a workspace where you have run `catkin_make` with the `hebi_description` package.  Then run the first launch command below from that directory.


```
roslaunch moveit_setup_assistant setup_assistant.launch
```

### Loading your robot

1. Press the button for creating a new config
1. Browse to find your XACRO (or URDF) file.  If using a xacro, add the `--xacro-ns` flag in the "optional xacro arguments" box. 
1. press the "load files" button

### Self collisions

1. Move to the self collisions tab.  We moved up the sampling density to about halfway, but a lower sampling density will probably work fine.
1. Press "generate collision matrix".  We find the "matrix view" of these results more intuitive.

## Virtual Joints

1. Move to the virtual joints tab.  MoveIt config recommends adding a virtual joint from the base to the world.  (We were not sure if this is necessary, given that we already have a world element, but we followed the MoveIt tutorial here):
1. Click on Add Virtual Joint
1. Set the name as "virtual_joint"
1. Set the child link as "world" and the parent frame name as "world"
1. Set the joint type as "fixed"
1. Click "save"

### Planning Groups

1. Move to the planning groups tab
1. Click "add group"
1. Enter "group name" as "hebi_arm"
1. Choose `kdl_kinematics_plugin/KDLKinematicsPlugin` as the kinematics solver. Note: this can be tweaked as desired
1. Let "Kin. Search Resolution" and "Kin. Search Timeout" stay at their default values.
1. Click "add joints".  Ctrl-click to select all of the actuator joints in the left pane (e.g., "HEBI/base/X8_9")
1. Click on the right arrow in the center to add the selected joints to the right panel
1. Press "save"

### Gripper

If you have a gripper, you can add the gripper as a planning group, too:

1. Click "add group"
1. Enter "group name" as "hand"
1. Leave kinematic solver as "none", and let "Kin. Search Resolution" and "Kin. Search Timeout" stay at their default values.
1. Click "add links"
1. Shift-click to select all of the links in the left pane that begin with "end_effector" _except for_ `end_effector/INPUT_INTERFACE` (or whichever links are part of your end effector)
1. Click on the right arrow in the center to add the selected links to the right panel
1. Press "save"

### Robot Poses

1. Move to the robot poses tab
1. Click "Add Pose".
1. Choose a name for the pose, and set joint angles here.  We recommend adding a "home" pose in a convenient (e.g., elbow-up) configuration.

If you have a gripper, you can add gripper poses here the same way.  We recommend adding an "open" and "closed" pose for the gripper.

### End Effectors

**Only follow these steps if you have a gripper**

1. Move to the end effectors tab
1. Choose `hand` as the "End Effector Name"
1. Choose `hand` as the "End Effector Group"
1. Choose `end_effector/INPUT_INTERFACE` as the "parent link" (this serves as the output of the last link on the arm)
1. Leave "parent group" blank

### ROS Controllers

1. Move to the ROS Controllers tab
1. Click the "auto add followjointstrajectory controllers for each planning group" at the top of the page.
1. **If creating an arm with a gripper**, delete the `hand_controller` controller that was created.

### 3D Perception

1. Go to the 3D sensing/perception tab, and select "none"

### Author Information

1. Go to the Author Information tab, and enter your name and email (required by catkin for publishing packages)

### Saving Configuration

1. Go to the "Configuration Files" tab, and select the desired output directory.  Note that the files will be placed directly in the selected directory, not in a new subdirectory, so choose a directory matching convention such as `myrobotname_moveit_config`.
1. Click "Generate Package"
1. Click "exit setup assistant"
1. **If creating an arm with a gripper**, you will need to manually add the following entry to the end of the `config/ros_controllers.yaml` file, under the `controller_list` parameter.  You may have to change the joint name if you have created a different end effector.
```
  - name: hand_controller
    action_ns: gripper_action
    type: GripperCommand
    default: true
    joints:
      - end_effector/input_l_finger
```
1. Save/commit your files after testing them.


