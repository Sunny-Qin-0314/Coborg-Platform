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

geometry_msgs::Pose target_pose1;

tf::TransformListener* listPoint;

moveit::planning_interface::MoveGroupInterface* movePoint;

bool moveReady = true;
bool success;

moveit::planning_interface::MoveGroupInterface::Plan* myPlanPoint;

void poseTransformCallback(const geometry_msgs::Pose::ConstPtr& posemsg)
{
    
    ROS_INFO("callback received!!!");

    //get transform between rodL and world frames
    tf::StampedTransform transform;
    try
    {
        listPoint->lookupTransform("world", "rodL",ros::Time(0), transform);
        ROS_INFO("now this is pod racing!");

        target_pose1.position.x = transform.getOrigin().getX() + posemsg->position.x;
        target_pose1.position.y = transform.getOrigin().getY() + posemsg->position.y;
        target_pose1.position.z = transform.getOrigin().getZ() + posemsg->position.z;

        ROS_INFO("Transforms are: x: %f, y: %f: z: %f", target_pose1.position.x,target_pose1.position.y, target_pose1.position.z);

        moveReady = true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // ROS_INFO("Transform rotation is: x: %f, y: %f, z: %f, w: %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    // ROS_INFO("Transform translation is: x: %f, y: %f: z: %f", transform.getOrigin().getX(),transform.getOrigin().getY(), transform.getOrigin().getZ());
    // ROS_INFO("Frame ID: %s",transform.frame_id_.c_str());
    

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tf_moveit_goalsetNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node;

    tf::TransformListener listener;
    listPoint = &listener;

    ROS_INFO("Loading robot model");
    static const std::string PLANNING_GROUP = "coborg_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    movePoint = &move_group;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    ROS_INFO("Declaring joint_model_group");
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Printing Basic Information Now:");
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));

    move_group.setPlanningTime(10.0);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();
    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

    // positon and orientation initialized relative to the rodL frame
    
    target_pose1.position.x = 1.0;
    target_pose1.position.y = -0.07659;
    target_pose1.position.z = -0.03536;
    
    // base point 1 (theoretically feasible for robot arm)
    // target_pose1.position.x = 0.6124;
    // target_pose1.position.y = -0.07659;
    // target_pose1.position.z = -0.03536;
    // target_pose1.orientation.x = 0.00019349;
    // target_pose1.orientation.y = -0.18569;
    // target_pose1.orientation.z = 0.60322;
    // target_pose1.orientation.w = 0.77566;
    // target_pose1.orientation.w = 1.0;

    // convert pose to eigen
    Eigen::Isometry3d target_joints;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    myPlanPoint = &my_plan;

    // publish to desired_pose rostopic
    ros::Publisher desired_pos_pub = node.advertise<geometry_msgs::Pose>("desired_pose", 1000);

    // subscribe to desired pose rostopic
    ros::Subscriber sent_msg_sub = node.subscribe("desired_pose", 1000, poseTransformCallback);

    desired_pos_pub.publish(target_pose1);

    movePoint->setGoalTolerance(0.001);

    moveit::core::RobotState start_state(*move_group.getCurrentState());
    
    ros::Rate rate(10.0);

    while(ros::ok())
    {

        tf::poseMsgToEigen(target_pose1,target_joints);
        // current_state = move_group.getCurrentState();
        move_group.setStartState(start_state);
        
        
        // convert Eigen to joint states

        movePoint->setApproximateJointValueTarget(target_pose1,"end_link/INPUT_INTERFACE"); //considered a workaround solution
        // movePoint->setPoseTarget(target_pose1,"end_link/INPUT_INTERFACE");
        // movePoint->setPositionTarget(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
        // movePoint->setJointValueTarget(target_pose1, "end_link/INPUT_INTERFACE");

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        success = (movePoint->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        // Visualizing plans
        // ^^^^^^^^^^^^^^^^^
        // We can also visualize the plan as a line with markers in RViz.
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
        visual_tools.publishAxisLabeled(target_pose1, "pose1");
        visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(myPlanPoint->trajectory_, joint_model_group);
        visual_tools.trigger();
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); 

        move_group.move();
        moveReady = false;

        ros::Duration(2.0).sleep();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

   