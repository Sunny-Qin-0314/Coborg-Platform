#include "ros/ros.h"
#include <sstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <coborg_move/CartesianTrajectory.h>

// Define global variables
float compact_origin_Cartesian[3];
std::vector<int> task_space_headers;
std::vector<std::vector<float>> task_space_goals;
std::vector<std::vector<float>> task_space_origins;
std::vector<float*> buffer_space_goals = std::vector<float*>();
std::vector<float*> buffer_space_origins = std::vector<float*>();
std::vector<float*> task_stable_space_goals = std::vector<float*>();
std::vector<float*> task_stable_space_origins = std::vector<float*>();
std::vector<float*> buffer_stable_space_goals = std::vector<float*>();
std::vector<float*> buffer_stable_space_origins = std::vector<float*>();

// Subscriber Callbacks
void task_space_callback(const coborg_move::CartesianTrajectory::ConstPtr& msg)
{
    std::vector<float> goal_point;
    goal_point.resize(7);
    std::vector<float> origin_point;
    origin_point.resize(7);
    task_space_headers.push_back(msg->header.seq);
    if (msg->header.seq == 0) {
        task_space_origins.push_back(origin_point);
        goal_point[0] = msg->goal.position.x;
        goal_point[1] = msg->goal.position.y;
        goal_point[2] = msg->goal.position.z;
        goal_point[3] = msg->goal.orientation.x;
        goal_point[4] = msg->goal.orientation.y;
        goal_point[5] = msg->goal.orientation.z;
        goal_point[6] = msg->goal.orientation.w;
        task_space_goals.push_back(goal_point);
    }
    else if (msg->header.seq == 1) {
        origin_point[0] = msg->origin.position.x;
        origin_point[1] = msg->origin.position.y;
        origin_point[2] = msg->origin.position.z;
        origin_point[3] = msg->origin.orientation.x;
        origin_point[4] = msg->origin.orientation.y;
        origin_point[5] = msg->origin.orientation.z;
        origin_point[6] = msg->origin.orientation.w;
        task_space_origins.push_back(origin_point);
        goal_point[0] = msg->goal.position.x;
        goal_point[1] = msg->goal.position.y;
        goal_point[2] = msg->goal.position.z;
        goal_point[3] = msg->goal.orientation.x;
        goal_point[4] = msg->goal.orientation.y;
        goal_point[5] = msg->goal.orientation.z;
        goal_point[6] = msg->goal.orientation.w;
        task_space_goals.push_back(goal_point);
    }
}

// void buffer_space_callback(const moveit_msgs::CartesianTrajectory::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// void task_stable_space_callback(const moveit_msgs::CartesianTrajectory::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// void buffer_stable_space_callback(const moveit_msgs::CartesianTrajectory::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "Mechanical_Task_Space_MoveIt");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialize publishers and subscribers
    ros::Publisher task_success_pub = n.advertise<geometry_msgs::Pose>("task_success", 1000);
    //   ros::Publisher buffer_success_pub = n.advertise<geometry_msgs::Pose>("buffer_success", 1000);
    //   ros::Publisher task_stable_success_pub = n.advertise<geometry_msgs::Pose>("task_stable_success", 1000);
    //   ros::Publisher buffer_stable_success_pub = n.advertise<geometry_msgs::Pose>("buffer_stable_success", 1000);
    ros::Subscriber task_space_sub = n.subscribe("task_space", 1000, task_space_callback);
    //   ros::Subscriber buffer_space_sub = n.subscribe("buffer_space", 1000, buffer_space_callback);
    //   ros::Subscriber task_stable_space_sub = n.subscribe("task_stable_space", 1000, task_stable_space_callback);
    //   ros::Subscriber buffer_stable_space_sub = n.subscribe("buffer_stable_space", 1000, buffer_stable_space_callback);

    // Types of arms:
    // coborg_arm
    // dof_4_arm
    // dof_5_config_z_arm
    // dof_5_config_y_arm
    static const std::string PLANNING_GROUP = "coborg_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::string end_effector;
    end_effector = move_group.getEndEffectorLink();

    // Set origin point based on arm type
    if (PLANNING_GROUP == "coborg_arm") {
        std::vector<double> compact_configuration_rad;
        compact_configuration_rad[0] = -1.91986;
        compact_configuration_rad[1] = -2.37365;
        compact_configuration_rad[2] = -2.51327;
    }
    else if (PLANNING_GROUP == "dof_4_arm") {
        std::vector<double> compact_configuration_rad;
        compact_configuration_rad[0] = 0.0;
        compact_configuration_rad[1] - 1.91986;
        compact_configuration_rad[2] = -2.37365;
        compact_configuration_rad[3] = -2.51327;
    }
    else if (PLANNING_GROUP == "dof_5_config_z_arm") {
        std::vector<double> compact_configuration_rad;
        compact_configuration_rad[0] = 0.0;
        compact_configuration_rad[1] = -1.9266;
        compact_configuration_rad[2] = 0.0;
        compact_configuration_rad[3] = -2.0307;
        compact_configuration_rad[4] = -2.3778;
    }
    else if (PLANNING_GROUP == "dof_5_config_y_arm") {
        std::vector<double> compact_configuration_rad;
        compact_configuration_rad[0] = 0.0;
        compact_configuration_rad[1] = -1.9266;
        compact_configuration_rad[2] = 0.0;
        compact_configuration_rad[3] = -2.0307;
        compact_configuration_rad[4] = -2.3778;
    }

    // Adjust MoveIt default settings
    // Set goal tolerances
    move_group.setGoalOrientationTolerance(0.52);
    move_group.setGoalPositionTolerance(0.05);
    // Increase planning time
    move_group.setPlanningTime(10.0);
    // Change planner
    move_group.setPlannerId("RRTConnect");
    // Send the arm to its origin position
    move_group.setJointValueTarget(compact_configuration_rad);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    // Loop, checking for points and running MoveIt
    while (ros::ok())
    {

        if (std::size(task_space_goals) > 0) {
            // Grab the oldest point to check
            int* header_ptr = task_space_headers.front();
            float* goal_ptr = task_space_goals.front();
            float* origin_ptr = task_space_origins.front();
            move_group.clearPoseTargets();
            // Plan from compact to goal point
            if (*header_ptr == 0) {
                // Set the goal pose
                geometry_msgs::PoseStamped goal_pose;
                goal_pose.position.x = *goal_ptr[0];
                goal_pose.position.y = *goal_ptr[1];
                goal_pose.position.z = *goal_ptr[2];
                goal_pose.orientation.x = *goal_ptr[3];
                goal_pose.orientation.y = *goal_ptr[4];
                goal_pose.orientation.z = *goal_ptr[5];
                goal_pose.orientation.w = *goal_ptr[6];
                move_group.setPoseTarget(goal_pose);
                // Plan the path
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (success == 1) {
                    goal_pose.header.seq = 1;
                    task_success_pub.publish(goal_pose);
                }
                else if (success == 0) {
                    goal_pose.header.seq = 0;
                    task_success_pub.publish(goal_pose);
                }
            }
            // Plan from origin to goal point
            else if (*header_ptr == 1) {
                geometry_msgs::PoseStamped origin_pose;
                origin_pose.position.x = *origin_ptr[0];
                origin_pose.position.y = *origin_ptr[1];
                origin_pose.position.z = *origin_ptr[2];
                origin_pose.orientation.x = *origin_ptr[3];
                origin_pose.orientation.y = *origin_ptr[4];
                origin_pose.orientation.z = *origin_ptr[5];
                origin_pose.orientation.w = *origin_ptr[6];
                geometry_msgs::PoseStamped goal_pose;
                goal_pose.position.x = *goal_ptr[0];
                goal_pose.position.y = *goal_ptr[1];
                goal_pose.position.z = *goal_ptr[2];
                goal_pose.orientation.x = *goal_ptr[3];
                goal_pose.orientation.y = *goal_ptr[4];
                goal_pose.orientation.z = *goal_ptr[5];
                goal_pose.orientation.w = *goal_ptr[6];

                std::vector<geometry_msgs::PoseStamped> waypoints;
                waypoints.push_back(origin_pose);
                waypoints.push_back(goal_pose);
                moveit_msgs::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                if (fraction > 0.99) {
                    goal_pose.header.seq = 1;
                    task_success_pub.publish(goal_pose);
                }
                else if (fraction <= 0.99) {
                    goal_pose.header.seq = 0;
                    task_success_pub.publish(goal_pose);
                }
            }
            // Erase point from vector
            task_space_headers.erase(task_space_headers.begin());
            task_space_goals.erase(task_space_goals.begin());
            task_space_origins.erase(task_space_origins.begin());
        }
        ros::spinOnce();
    }
    return 1;
}