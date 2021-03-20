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
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "find_hebi_modules");
    ros::NodeHandle node;

    //Construct robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //maintain state of the robot
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("coborg_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    //get current state of joint values
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // // perform forward kinematics to get state of the end effector
    // kinematic_state->setToRandomPositions(joint_model_group);
    // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
    // ROS_INFO_STREAM("Forward Translation: \n" << end_effector_state.translation() << "\n");
    // ROS_INFO_STREAM("Forward Rotation: \n" << end_effector_state.rotation() << "\n");

    //perform inverse kinematics based on end_effector_position
    Eigen::Vector3d pos = Eigen::Vector3d(0.491584, -0.162383, 0.441646);   
    Eigen::Vector3d curr_pos = Eigen::Vector3d(0.6924, 0.024512, 0.413701);
    Eigen::Vector3d changevect = pos - curr_pos;
    changevect.normalize();

    Eigen::Quaterniond quatRot;
    quatRot.setFromTwoVectors(Eigen::Vector3d(0,-1,0),changevect);
    Eigen::Matrix3d quatRotMat;
    quatRotMat = quatRot.toRotationMatrix();
    ROS_INFO_STREAM("Quaternion Rotation: \n" << quatRotMat << "\n");



    //FORNOW: hard coded rotationmatrix
    Eigen::MatrixXd rot(3,3);
    rot(0,0) = 0.306492;
    rot(0,1) = 0.950377;
    rot(0,2) = -0.0533535;
    rot(1,0) = -0.918691;
    rot(1,1) = 0.310013;
    rot(1,2) = 0.244744;
    rot(2,0) = 0.249139;
    rot(2,1) = -0.0259966;
    rot(2,2) = 0.968119;

    Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();
    end_effector_state.translation() = pos;
    end_effector_state.linear() = rot;

    ROS_INFO_STREAM("Forward Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Forward Rotation: \n" << end_effector_state.rotation() << "\n");


    std::size_t attempts = 10;
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (found_ik)
    {
      kinematic_state -> copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Inverse Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }else
    {
      ROS_INFO("Did not find IK solution");
    }

    // // Compute Jacobian
    // Eigen::Vector3d reference_point_position(0.66693, 0.074561, 0.36542);
    // Eigen::MatrixXd jacobian;
    // kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
    // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

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


    ros::Rate loop_rate(5);


    Eigen::VectorXd positions(group->size());
    //set robot arm to be straight out
    positions.setZero();

    // positions[0] = joint_values[0];
    // positions[1] = joint_values[1];
    // positions[2] = joint_values[2];

    
    hebi::GroupCommand groupCommand(group->size());
    groupCommand.setPosition(positions);

    //define feedback
    hebi::GroupFeedback group_feedback(group->size());

    Eigen::VectorXd feedbackPos(group->size());
    feedbackPos.setZero();



    group->setFeedbackFrequencyHz(50);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();
    double durr; 

    while (ros::ok())
    {
        curr = ros::Time::now();
        durr = (curr- begin).toSec();
        //console debugging command
        //ROS_INFO("What year is it!!??\n");

        if (!group->getNextFeedback(group_feedback))
        {
          continue;
        }
        group->getNextFeedback(group_feedback);

        //Eigen::VectorXd feedbackPos = group_feedback.getPosition();
        //std::cout << "Position feedback: " << std::endl << feedbackPos << std::endl;

        //stream constant joint angle vector to coborg
        // if(durr > 5.0)
        // {
        //     positions[0] = joint_values[0];
        //     positions[1] = joint_values[1];
        //     positions[2] = joint_values[2];
        //     groupCommand.setPosition(positions);
        //     ROS_INFO("Running position change");
        // }
        // group->sendCommand(groupCommand);
        // group->sendCommandWithAcknowledgement(groupCommand);



        ros::spinOnce(); //handle ROS events
    }

    return 0;
}
