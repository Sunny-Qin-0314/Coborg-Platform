#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include "Eigen/Eigen"

#include <algorithm> //based on HEBI C++ API 3.3.0 documentation
#include <cmath>

#include <gb_visual_detection_3d_msgs/goal_msg.h>

tf::StampedTransform* prevTransform;
tf::Listener* globalListener;
gb_visual_detection_3d_msgs::goal_msg* goal; 

geometry_msgs::Pose poseMotionDetection(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
    
    tf::StampedTransform transform;
    geometry_msgs::Pose target_pose;

    try
    {
        // TODO: acquire the transform once
        // FORNOW: update transform parameters at every callback interval
        //listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
        globalListener->lookupTransform("/t265_odom", "/end_link/INPUT_INTERFACE",ros::Time(0), transform);

        // FORNOW: only goal position is updated b/c 3DoF robot arm cannot solve 6DoF goal every time
        target_pose.position.x = -(transform.getOrigin().getX()-prevTransform->getOrigin().getX()) + pose_msg->position.x;
        target_pose.position.y = -(transform.getOrigin().getY()-prevTransform->getOrigin().getY()) + pose_msg->position.y;
        target_pose.position.z = -(transform.getOrigin().getZ()-prevTransform->getOrigin().getZ()) + pose_msg->position.z;

        ROS_INFO("Transforms are: x: %f, y: %f: z: %f", target_pose.position.x,target_pose.position.y, target_pose.position.z);

        // robot arm can now move to updated goal pose
        return target_pose;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return *pose_msg;
    }
}

void get_goalpose_callback(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg)
{
    goal = goal_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "resolved_rate_planner");
    ros::NodeHandle node;

    tf::TransformListener listener;
    globalListener = &listener

    // Establish initial tf
    tf::Transform tempTrans;
    globalListener->waitForTransform("/t265_odom", "/end_link/INPUT_INTERFACE", ros::Time(0), ros::Duration(3.0), tempTrans);
    prevTransform = &tempTrans;

    ros::Subscriber goal_pos_sub = node.subscribe("/goal",1,get_goalpose_callback);
    

}