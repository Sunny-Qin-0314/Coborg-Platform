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

geometry_msgs::Pose target_pose1;

tf::TransformListener* listPoint;

moveit::planning_interface::MoveGroupInterface* movePoint;

bool moveReady = false;
bool success;
moveit::planning_interface::MoveItErrorCode moveitSuccess;

moveit::planning_interface::MoveGroupInterface::Plan* myPlanPoint;

ros::Time beginTime;
ros::Time currTime;
ros::Duration durVar;

std::string maniState;

robot_state::RobotStatePtr* kin_model;
std::vector<double> hebiJoints(3);
Eigen::Vector3d current_hebi_joints;
Eigen::Isometry3d end_effector_state;

void poseTransformCallback(const geometry_msgs::Pose::ConstPtr& posemsg)
{
    
    ROS_INFO("callback received!!!");

    //get transform between rodL and world frames
    tf::StampedTransform transform;
    try
    {
        listPoint->lookupTransform("world", "camera_link",ros::Time(0), transform);
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

void hebiJointsCallback(const geometry_msgs::Vector3::ConstPtr& hebimsg)
{
    //compute foward kinematics of robot
    // ROS_INFO("hebi callback received");
    hebiJoints.at(0) = hebimsg->x;
    hebiJoints.at(1) = hebimsg->y;
    hebiJoints.at(2) = hebimsg->z;



}



// TODO: create random position callback
// x: 0.6 - 1.0
// y: 0.0 - -0.35
// z: 0.0 - -0.2



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
    kin_model = &robot_state;

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

    move_group.setPlanningTime(2.0);

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
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

    // positon and orientation initialized relative to the rodL frame
    
    // target_pose1.position.x = 1.0;
    // target_pose1.position.y = -0.07659;
    // target_pose1.position.z = -0.03536;
    
    // base point 1 (theoretically feasible for robot arm)
    // target_pose1.position.x = 0.6124;
    // target_pose1.position.y = -0.07659;
    // target_pose1.position.z = -0.03536;
    // target_pose1.orientation.x = 0.00019349;
    // target_pose1.orientation.y = -0.18569;
    // target_pose1.orientation.z = 0.60322;
    // target_pose1.orientation.w = 0.77566;
    // target_pose1.orientation.w = 1.0;

    // target_pose1.position.x = 0.7014;
    // target_pose1.position.y = -0.3670;
    // target_pose1.position.z = -0.0814;

    // convert pose to eigen
    Eigen::Isometry3d target_joints;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    myPlanPoint = &my_plan;

    // publish to desired_pose rostopic
    ros::Publisher desired_efforts_pub = node.advertise<geometry_msgs::Vector3>("desired_hebi_efforts", 10);

    // subscribe to desired pose rostopic
    ros::Subscriber sent_msg_sub = node.subscribe("desired_pose", 1000, poseTransformCallback);
    ros::Subscriber hebi_joints_sub = node.subscribe("hebi_joints", 10, hebiJointsCallback);

    // subscribe to vision system true rostopic to output random goal pose position

    
    // set joint constraints
    // moveit_msgs::JointConstraint jcm;
    // jcm.joint_name = "motor2/X5_9";
    // jcm.position = -3.14/2.0;
    // jcm.tolerance_above = 0.05;
    // jcm.tolerance_below = 3.0/2.0;
    // jcm.weight = 1.0;

    // moveit_msgs:: Constraints test_constraints;
    // test_constraints.joint_constraints.push_back(jcm);
    // move_group.setPathConstraints(test_constraints);

    movePoint->setGoalTolerance(0.001);

    moveit::core::RobotState start_state(*move_group.getCurrentState());
    
    ros::Rate rate(10.0);

    beginTime = ros::Time::now();

    // stabilization variables
    geometry_msgs::Pose target_push_pose2;
    float xErr;
    float yErr;
    float zErr;

    float impGain = 5;

    Eigen::VectorXd endLinkEffort(6);

    Eigen::MatrixXd wristJacobian;

    Eigen::VectorXd desired_torques;






    while(ros::ok())
    {

        ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);
        ROS_INFO("The state of manipulation is: %s", maniState.c_str());
        currTime = ros::Time::now();
        durVar = currTime - beginTime;

        if (moveReady)
        {


            tf::poseMsgToEigen(target_pose1,target_joints);
            // current_state = move_group.getCurrentState();
            // move_group.setStartState(start_state);
            
            
            // convert Eigen to joint states

            movePoint->setJointValueTarget(target_pose1,"end_link/INPUT_INTERFACE"); //considered a workaround solution
            
            // movePoint->setPoseTarget(target_pose1,"end_link/INPUT_INTERFACE");
            // movePoint->setPositionTarget(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
            // movePoint->setJointValueTarget(target_pose1, "end_link/INPUT_INTERFACE");

            // Now, we call the planner to compute the plan and visualize it.
            // Note that we are just planning, not asking move_group
            // to actually move the robot.
            // success = (movePoint->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            moveitSuccess = move_group.plan(my_plan);

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", moveitSuccess ? "" : "FAILED");

            // Visualizing plans
            // ^^^^^^^^^^^^^^^^^
            // We can also visualize the plan as a line with markers in RViz.
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
            visual_tools.publishAxisLabeled(target_pose1, "pose1");
            visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishTrajectoryLine(myPlanPoint->trajectory_, joint_model_group);
            visual_tools.trigger();
            // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); 

            // move_group.move();
            move_group.execute(my_plan);
            moveReady = false;
        }
        else if (strcmp(maniState.c_str(),"stabilize") == 0)
        {
            target_push_pose2 = target_pose1;
            target_push_pose2.position.z += 0.05;
            ROS_INFO("State is now in stabilization mode");
            // compute fwd kinematics from hebi joints
            robot_state->setJointGroupPositions(joint_model_group, hebiJoints);
            end_effector_state = robot_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
            ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
            ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

            // end effector translation is x,y,z state
            // error is desired - current
            xErr = target_push_pose2.position.x - end_effector_state.translation().x();
            yErr = target_push_pose2.position.y - end_effector_state.translation().y();
            zErr = target_push_pose2.position.z - end_effector_state.translation().z();

            // set effort from gains and errors
            endLinkEffort[0] = impGain*xErr; // x directional force
            endLinkEffort[1] = impGain*yErr; // y directional force
            endLinkEffort[2] = impGain*zErr; // z directional force
            endLinkEffort[3] = 0;
            endLinkEffort[4] = 0;
            endLinkEffort[5] = 0;

            // perform jacobian transform from end effector force to joint torques
            current_hebi_joints[0] = hebiJoints.at(0);
            current_hebi_joints[1] = hebiJoints.at(1);
            current_hebi_joints[2] = hebiJoints.at(2);

            robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), current_hebi_joints, wristJacobian);

            ROS_INFO_STREAM("wristJacobian: \n" << wristJacobian << "\n");

            desired_torques = wristJacobian.transpose()*endLinkEffort;

            





        }



        ros::Duration(2.0).sleep();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

   