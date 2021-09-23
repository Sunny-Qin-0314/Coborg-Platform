#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include "Eigen/Eigen"


#include <algorithm> //based on HEBI C++ API 3.3.0 documentation
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/Int16.h"
#include <string.h>


// declare target pose to send to RViz
geometry_msgs::Pose target_pose1;

// declare tf listener node to be global to be used in callback and main function
tf::TransformListener* listPoint;

// sets interval for robot arm to periodically move
bool moveReady = false;

// global time variables to set durations
ros::Time beginTime;
ros::Time currTime;
ros::Duration durVar;

// subscribe to rosparam for manipulation state triggers
std::string maniState;
//std::string "camera_link" = "camera_link";


// stabilization and impedance control global variables
std::vector<double> hebiJointAngles(3);
std::vector<double> hebiJointAngVelocities(3);
Eigen::Vector3d current_hebi_joints;
Eigen::Isometry3d end_effector_state;

// MoveIt global variables (optional) explained later in the code
bool success;
moveit::planning_interface::MoveItErrorCode moveitSuccess;
moveit::planning_interface::MoveGroupInterface::Plan* myPlanPoint;
moveit::planning_interface::MoveGroupInterface* movePoint;
robot_state::RobotStatePtr* kin_model;

std::vector<std::string> presetPositionNames;
std::vector<std::vector<double>> presetJointValues;

//declare preset joint positions
// home: -110, -136, -144 [deg]
// home: -1.91986, -2.37365, -2.51327 [rad]

// ready: -54, -126, -87 [deg]
// ready: -0.942478, -2.19911, -1.51844 [rad]


