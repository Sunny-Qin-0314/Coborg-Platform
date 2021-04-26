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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <string.h>

// declare motor joints as global vairables
double motor1_joint;
double motor2_joint;
double motor3_joint;

// start up variables
// set start up procedure to prevent max torque at start up phenomena
bool boolFirstTime = true;
double startup_sec = 0.1;

// set global variable to publish joint angles
geometry_msgs::Twist publishState;
geometry_msgs::Vector3 torqueVect;

// subscribe to rosparam for state triggers
std::string maniState;

ros::Time currImp;

bool impValue = false;


// convert joint angles from MoveIt node to HEBI joint angles
void hebiOutputCallback(const sensor_msgs::JointState::ConstPtr& msg){
  // ROS_INFO("motor1: %f | motor2: %f | motor3: %f", msg->position[0],msg->position[1],msg->position[2]);
  motor1_joint = msg->position[0] + 0.1; // offset determined empirically for level arm out at 0 radians 
  motor2_joint = msg->position[1];
  motor3_joint = msg->position[2];
}

void effortCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  impValue = true;
  currImp = ros::Time::now();
  ROS_INFO_STREAM("Received Torque Command : \n" << *msg << "\n");
  torqueVect = *msg;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "find_hebi_moveit_planner");
    ros::NodeHandle node;

    //Goals
    //Subscribe to /move_group/fake_controller_states rostopic
    //output the topic to the screen

    ros::Subscriber fake_joint_states_sub = node.subscribe("/move_group/fake_controller_joint_states",1,hebiOutputCallback);
    ros::Publisher hebi_joints_pub = node.advertise<geometry_msgs::Twist>("hebi_joints", 1);

    ros::Subscriber desired_torques_sub = node.subscribe("/desired_hebi_efforts", 1, effortCallback);

    
    //TODO: create method to auto find the families and names
    //FORNOW: assume there is 1 family and 3 names
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

    // set HEBI command variables
    hebi::GroupCommand groupCommandBegin(group->size());
    hebi::GroupCommand groupCommand(group->size());
    hebi::GroupCommand groupCommandStabilize(group->size());

    // set positions based on motor joints angles
    // VectorXd will get passed into HEBI joint command
    Eigen::VectorXd positions(group->size());
    Eigen::VectorXd torques(group->size());

    //define feedback variables
    hebi::GroupFeedback group_feedback(group->size());
    Eigen::VectorXd feedbackPos(group->size());
    Eigen::VectorXd feedbackVel(group->size());
    Eigen::VectorXd feedbackTor(group->size());
    feedbackPos.setZero();



    // (impedance control) declare varaibles to be using for force control state
    group->setFeedbackFrequencyHz(50);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();
    ros::Time beginImp = ros::Time::now();
    double durr; 
    Eigen:: VectorXd startupVelocity(group->size());
    startupVelocity.setOnes();
    const double stiffness = 1.0;

    ros::Rate loop_rate(20.0);


    while(ros::ok()) {
      beginImp = ros::Time::now();
      

      // compute duration variable
      durr = (float) (curr- begin).toSec();

      // acquire current state of the manipulation system
      ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);

      // if the motors sendback feedback information
      if (group->getNextFeedback(group_feedback))
      {
        feedbackPos = group_feedback.getPosition();
        feedbackTor = group_feedback.getEffort();
        feedbackVel = group_feedback.getVelocity();
        // std::cout << "Position feedback: " << std::endl << feedbackPos << std::endl;
        publishState.linear.x = (float) feedbackPos(0); // base motor
        publishState.linear.y = (float) feedbackPos(1); // elbow motor
        publishState.linear.z = (float) feedbackPos(2); // wrist motor
        publishState.angular.x = (float) feedbackVel(0);
        publishState.angular.y = (float) feedbackVel(1);
        publishState.angular.z = (float) feedbackVel(2);
        torques[0] = (float) feedbackTor(0);
        torques[1] = (float) feedbackTor(1);
        torques[2] = (float) feedbackTor(2);
        groupCommandStabilize.setEffort(torques);
        group->sendCommand(groupCommandStabilize);

        hebi_joints_pub.publish(publishState);


        // intiliaze if HEBI motors are starting up for the first time
        if (boolFirstTime)
        {

          // TODO: load xml gain files that has the velocity and effort limits on them
          // FORNOW: hardcode velocity numbers and push those velocities to the joints for a period of time
          curr = ros::Time::now();
          startupVelocity[0] = 0.2*durr*(motor1_joint - publishState.linear.x);
          startupVelocity[1] = 0.2*durr*(motor2_joint - publishState.linear.y);
          startupVelocity[2] = 0.2*durr*(motor3_joint - publishState.linear.z);

          groupCommandBegin.setVelocity(startupVelocity);
          group->sendCommand(groupCommandBegin);

          if (durr > startup_sec)
          {
            boolFirstTime = false;
          }
        }
        else if (strcmp(maniState.c_str(),"impedance") == 0 && impValue == true)
        {
          // position control
          // positions[0] = motor1_joint;
          // positions[1] = motor2_joint;
          // positions[2] = motor3_joint;
          // groupCommand.setPosition(positions);
          // // group->sendCommand(groupCommand);
          // group->sendCommandWithAcknowledgement(groupCommand);
          curr = ros::Time::now();
          ROS_INFO_STREAM("HEBI Time to Receive Callback: " << (float) (curr- currImp).toSec() << "\n");

          // Torques to instantenously hold up the robot arm
          // torques[0] = 3.05;
          // torques[1] = -0.29;
          // torques[2] = 0.15;
          
          torques[0] = torqueVect.x;
          torques[1] = torqueVect.y;
          torques[2] = torqueVect.z;

          groupCommandStabilize.setEffort(torques);
          group->sendCommand(groupCommandStabilize);
          // curr = ros::Time::now();
          // ROS_INFO_STREAM("Impedance Duration: " << (float) (curr- begin).toSec() << "\n");


          

        }
        else
        {
          curr = ros::Time::now();
          // position control
          positions[0] = motor1_joint;
          positions[1] = motor2_joint;
          positions[2] = motor3_joint;

          groupCommand.setPosition(positions);
          // group->sendCommand(groupCommand);
          group->sendCommand(groupCommand);
          // ROS_INFO_STREAM((float) durr);
          impValue = false;


        }


        
      }







      // if (strcmp(maniState.c_str(),"motion") == 0)
      // {
      //   //implement own code to publish message
      //   curr = ros::Time::now();
      //   durr = (curr- begin).toSec();
      //   //console debugging command
      //   //ROS_INFO("What year is it!!??\n");

      //   positions[0] = motor1_joint;
      //   positions[1] = motor2_joint;
      //   positions[2] = motor3_joint;
      //   groupCommand.setPosition(positions);

      //   // group->sendCommand(groupCommand);
      //   group->sendCommandWithAcknowledgement(groupCommand);
      // }
      // else if (strcmp(maniState.c_str(),"stabilize") == 0)
      // {
      //   hebi_joints_pub.publish(publishState);
      // }


      ros::spinOnce();
      loop_rate.sleep();
   }

    return 0;
}
