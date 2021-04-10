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

double motor1_joint;
double motor2_joint;
double motor3_joint;

bool boolFirstTime = true;
double startup_sec = 1.5;

geometry_msgs::Vector3 publishState;

std::string maniState;

void hebiOutputCallback(const sensor_msgs::JointState::ConstPtr& msg){
  ROS_INFO("message heard!");
  ROS_INFO("motor1: %f | motor2: %f | motor3: %f", msg->position[0],msg->position[1],msg->position[2]);
  motor1_joint = msg->position[0] + 0.1; // offset determined empirically for level arm out at 0 radians 
  motor2_joint = msg->position[1];
  motor3_joint = msg->position[2];
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "find_hebi_moveit_planner");
    ros::NodeHandle node;

    //Goals
    //Subscribe to /move_group/fake_controller_states rostopic
    //output the topic to the screen

    ros::Subscriber fake_joint_states_sub = node.subscribe("/move_group/fake_controller_joint_states",1000,hebiOutputCallback);
    ros::Publisher hebi_joints_pub = node.advertise<geometry_msgs::Vector3>("hebi_joints", 10);

    
    //TODO: create method to auto find the families and names
    //FORNOW: assume there is 1 family and 3 names
    std::vector<std::string> families;
    families = {"X5-9","X5-9","X5-4"};

    std::vector<std::string> names;
    names = {"base_1", "elbow_2", "wrist_3"};

    //code stolen from hebi_cpp_api_examples/src/basic/group_node.cpp
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

    if (!group) {
      ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return -1;
    }

    //print names of each HEBI item in the group
    std::cout << "Total Number of HEBI Members in group: " << group->size() << std::endl;

    Eigen::VectorXd positions(group->size());
    positions[0] = motor1_joint;
    positions[1] = motor2_joint;
    positions[2] = motor3_joint;

    hebi::GroupCommand groupCommandBegin(group->size());
    hebi::GroupCommand groupCommand(group->size());
    // groupCommand.setPosition(positions);

    //define feedback
    hebi::GroupFeedback group_feedback(group->size());

    Eigen::VectorXd feedbackPos(group->size());
    feedbackPos.setZero();


    group->setFeedbackFrequencyHz(50);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();
    double durr; 
    
    Eigen:: VectorXd efforts(group->size());
    efforts.setOnes();

    const double stiffness = 1.0;

    ros::Rate loop_rate(30);


    while(ros::ok()) {

      //implement own code to publish message
      
      durr = (float) (curr- begin).toSec();
      //console debugging command
      //ROS_INFO("What year is it!!??\n");

      ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);

      // get feedback from hebi


      if (group->getNextFeedback(group_feedback))
      {
        group->getNextFeedback(group_feedback);
        feedbackPos = group_feedback.getPosition();
        publishState.x = (float) feedbackPos(0);
        publishState.y = (float) feedbackPos(1);
        publishState.z = (float) feedbackPos(2);

        if (boolFirstTime)
        {

          // TODO: load xml gain files that has the velocity and effort limits on them
          // FORNOW: hardcode velocity numbers and push those velocities to the joints for a period of time
          curr = ros::Time::now();
          efforts[0] = 0.1*(motor1_joint - publishState.x);
          efforts[1] = 0.1*(motor2_joint - publishState.y);
          efforts[2] = 0.1*(motor3_joint - publishState.z);

          groupCommandBegin.setVelocity(efforts);
          group->sendCommand(groupCommandBegin);

          if (durr > startup_sec)
          {
            boolFirstTime = false;
          }
        }
        else
        {
          // position control
          positions[0] = motor1_joint;
          positions[1] = motor2_joint;
          positions[2] = motor3_joint;
          groupCommand.setPosition(positions);
          // group->sendCommand(groupCommand);
          group->sendCommandWithAcknowledgement(groupCommand);
        }



        
        std::cout << "Position feedback: " << std::endl << feedbackPos << std::endl;
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
