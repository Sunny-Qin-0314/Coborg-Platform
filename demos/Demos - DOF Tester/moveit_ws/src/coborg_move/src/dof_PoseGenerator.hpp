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
#include <math.h>

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

geometry_msgs::Pose goalPose;
std::string defOrigin;
std::int8_t sequenceCount = 0;

ros::Time beginTime;
ros::Time currTime;
ros::Duration durVar;

std::string maniState;
std::string vision_reference_frame = "d400_link";
std::string svdTargetVal = "target1";
std::string visionTargetStart = "visiongoal";
std::string visionTargetEnd = "goalset";

float uniWaitSec = 1.0;

tf::StampedTransform* globalTransform;

// 3d pose default positions (should be universal per robot arm)
Eigen::Vector3d homePose(-0.0144186, -0.0856238, 0.116201);
Eigen::Vector3d readyPushOut(0.353297, 0.11838, 0.342266);
Eigen::Vector3d dummyPushOut(0.626008, 0.174859, 0.438969);
Eigen::Vector3d dummyPushUp(0.0294834, 0.190455, 1.06668);

// list of target locations based on SVD day
Eigen::Vector3d svdPushOut01(0.895-0.2704, -0.345+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut02(0.895-0.2704, -0.495+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut03(0.895-0.2704, -0.495+0.5209, -0.405+0.70005);

