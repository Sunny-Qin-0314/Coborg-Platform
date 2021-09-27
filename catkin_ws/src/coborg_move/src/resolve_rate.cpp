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

#include "std_msgs/Bool.h"
#include <gb_visual_detection_3d_msgs/goal_msg.h>

tf::StampedTransform* prevTransform;
tf::Listener* globalListener;
gb_visual_detection_3d_msgs::goal_msg* goal; 
bool enable_rr;

geometry_msgs::Pose poseMotionDetection(const geometry_msgs::Pose::ConstPtr& pose_msg)
{    
    tf::StampedTransform transform;
    geometry_msgs::Pose target_pose;

    try
    {
        // TODO: acquire the transform once
        // FORNOW: update transform parameters at every callback interval
        //listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
        globalListener->lookupTransform("/t265_odom", "/motor1_link/INPUT_INTERFACE",ros::Time(0), transform);

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

void goal_callback(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg)
{
    tf::StampedTransform transform;

    bool goal_got = false
    while(!goal_got)
    {
        try
        {
            // TODO: acquire the transform once
            // FORNOW: update transform parameters at every callback interval
            //listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
            globalListener->waitForTransform("/d400_link", "/t265_odom", ros::Time(0), ros::Duration(3.0), transform);

            // FORNOW: only goal position is updated b/c 3DoF robot arm cannot solve 6DoF goal every time
            goal.position.x = -transform.getOrigin().getX() + goal_msg->position.x;
            goal.position.y = -transform.getOrigin().getY() + goal_msg->position.y;
            goal.position.z = -transform.getOrigin().getZ() + goal_msg->position.z;

            ROS_INFO("Transforms are: x: %f, y: %f: z: %f", goal.position.x,goal.position.y, goal.position.z);

            // robot arm can now move to updated goal pose
            goal_got = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            goal_got = false;
        }
    }
}

void enable_rr_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
    enable_rr = bool_msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "resolved_rate_planner");
    ros::NodeHandle node;

    // Establish initial values
    tf::TransformListener listener;
    globalListener = &listener;

    tf::Transform tempTrans;
    globalListener->waitForTransform("/t265_odom", "/end_link/INPUT_INTERFACE", ros::Time(0), ros::Duration(3.0), tempTrans);
    prevTransform = &tempTrans;

    enable_rr = false;

    // Get goal position once it's published by vision nodes (same goal move_it picks up)
    ros::Subscriber goal_pos_sub = node.subscribe("/goal",1,goal_callback);
    // Topic to trigger resolved rate to begin
    ros::Subscriber tigger_rr_sub = node.subscribe("/enable_rr",1,enable_rr_callback);


    // inititalize HEBI API
    std::vector<std::string> families;
    families = {"X5-9","X5-9","X5-4"};
    std::vector<std::string> names;
    names = {"base_1", "elbow_2", "wrist_3"};

    // connect to HEBI joints on network through UDP connection
    std::shared_ptr<hebi::Group> group;
    for (int num_tries = 0; num_tries < 3; num_tries++) {
      hebi::Lookup lookup;
      group = lookup.getGroupFromNames(families, names);
      if (group) {
        //print hebi modules to terminal
        auto entry_list = lookup.getEntryList();

        for(size_t hebi_iter = 0; hebi_iter < entry_list->size(); ++ hebi_iter)
        {
            std::cout << (*entry_list)[hebi_iter].family_ << " | " << (*entry_list)[hebi_iter].name_ << std::endl;
        }
        break;
      }
      ROS_WARN("Could not find group actuators, trying again...");
      ros::Duration(1.0).sleep();
    }
    //code stolen from hebi_cpp_api_examples/src/basic/group_node.cpp

    // error out if HEBI joints are not found on the network
    if (!group) {
      ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return -1;
    }
    //print names of each HEBI item in the group
    std::cout << "Total Number of HEBI Members in group: " << group->size() << std::endl;

    hebi::GroupCommand groupCommand(group->size());

    hebi::GroupFeedback group_feedback(group->size());
    Eigen::VectorXd thetas(group->size());

    float dt = 0.01;
    Eigen::Identity W(3,3);

    ros::Rate rate(20.0);
    while(ros::ok())
    {
        if(enable_rr)
        {
            //xg = goal
            gb_visual_detection_3d_msgs::goal_msg xg = *goal;

            //theta = Get joint state
            if (group->getNextFeedback(group_feedback))
            {
                thetas = group_feedback.getPosition(); 
            }
            
            //x0 = forward_kinematics(theta)
            //[x,y,z of the end effector]

            //J = compute_jacobian(theta)
            //[2d matrix of joint angles ]

            //thetadot = inv(W)*J.T*inv(J*inv(W)*J.T)*(xg-x0)
            
            //theta = theta + dt*thetadot
            
            //command_angles(theta)
            groupCommand.setPosition(thetas);
            group->sendCommand(groupCommand);

        }
        ros::spinOnce();
        rate.sleep();
    }
}