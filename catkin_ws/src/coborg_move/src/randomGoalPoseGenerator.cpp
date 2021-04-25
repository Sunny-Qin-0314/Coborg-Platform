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
#include <goal_getter/goal_msg.h>

geometry_msgs::Pose goalPose;
std::string defOrigin;
std::int8_t sequenceCount = 0;

ros::Time beginTime;
ros::Time currTime;
ros::Duration durVar;

std::string maniState;






// hard coded position poses

Eigen::Vector3d homePose(0.26595, -0.4133, -0.556078);
Eigen::Vector3d readyPushOut(0.71119, -0.346, -0.05444);
Eigen::Vector3d dummyPushUp(0.24396, -0.273652, 0.28507);
Eigen::Vector3d dummyPushOut(0.895, -0.345, -0.255);

void goalPushOutCallback(const goal_getter::goal_msg::ConstPtr& posemsg)
{
    dummyPushOut(0) = posemsg->x;
    dummyPushOut(1) = posemsg->y;
    dummyPushOut(2) = posemsg->z;
}

void randomPoseFunc()
{
    // // relative to d400_link / camera_link frame
    // float xRangeMin = 0.6;
    // float xRangeMax = 0.9;
    // float yRangeMin = -0.4;
    // float yRangeMax = -0.15;
    // float zRangeMin = -0.25;
    // float zRangeMax = -0.15;

    // relative to rodL frame
    float xRangeMin = 0.6;
    float xRangeMax = 0.9;
    float yRangeMin = -0.3;
    float yRangeMax = -0.1;
    float zRangeMin = -0.2;
    float zRangeMax = -0.05;

    float randomX = ((float) rand()) / (float) RAND_MAX;
    float randomY = ((float) rand()) / (float) RAND_MAX;
    float randomZ = ((float) rand()) / (float) RAND_MAX;
    goalPose.position.x = randomX*(xRangeMax - xRangeMin) + xRangeMin;
    goalPose.position.y = randomY*(yRangeMax - yRangeMin) + yRangeMin;
    goalPose.position.z = randomZ*(zRangeMax - zRangeMin) + zRangeMin;
}


void homePoseFunc(std::int8_t& seqCount)
{

    // if preset is "arm out": set to joint values
    goalPose.position.x = homePose(0);
    goalPose.position.y = homePose(1);
    goalPose.position.z = homePose(2);

    seqCount = 0;



}

void readyPoseFunc()
{
    goalPose.position.x = readyPushOut(0);
    goalPose.position.y = readyPushOut(1);
    goalPose.position.z = readyPushOut(2);  
}


void upPoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;

    Eigen::MatrixXd seqMat(3,countMax);


    // ready push out 10cm out
    seqMat(0,0) = dummyPushUp(0);
    seqMat(1,0) = dummyPushUp(1);
    seqMat(2,0) = dummyPushUp(2) - 0.1;

    // push into -5cm into
    seqMat(0,1) = dummyPushUp(0);
    seqMat(1,1) = dummyPushUp(1);
    seqMat(2,1) = dummyPushUp(2) + 0.05;

    if (seqCount < countMax)
    {
        goalPose.position.x = seqMat(0, seqCount);
        goalPose.position.y = seqMat(1, seqCount);
        goalPose.position.z = seqMat(2, seqCount);
        seqCount += 1;

    }
    else
    {
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
    }

}

void upDisengagePoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;
    Eigen::MatrixXd seqMat(3, countMax);

    // ready push out 10cm out
    seqMat(0,0) = dummyPushUp(0);
    seqMat(1,0) = dummyPushUp(1);
    seqMat(2,0) = dummyPushUp(2) - 0.15;

    seqMat(0,1) = readyPushOut(0);
    seqMat(1,1) = readyPushOut(1);
    seqMat(2,1) = readyPushOut(2);

    if (seqCount > 0)
    {
        seqCount -= 1;
        goalPose.position.x = seqMat(0, 1 - seqCount);
        goalPose.position.y = seqMat(1, 1 - seqCount);
        goalPose.position.z = seqMat(2, 1 - seqCount);

    }
    else
    {
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
    }

}

void outPoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;

    Eigen::MatrixXd seqMat(3,countMax);


    // ready push out 10cm out
    seqMat(0,0) = dummyPushOut(0) - 0.1;
    seqMat(1,0) = dummyPushOut(1);
    seqMat(2,0) = dummyPushOut(2);

    // push into -5cm into
    seqMat(0,1) = dummyPushOut(0) + 0.01;
    seqMat(1,1) = dummyPushOut(1);
    seqMat(2,1) = dummyPushOut(2);

    if (seqCount < countMax)
    {
        goalPose.position.x = seqMat(0, seqCount);
        goalPose.position.y = seqMat(1, seqCount);
        goalPose.position.z = seqMat(2, seqCount);
        seqCount += 1;

    }
    else
    {
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
    }

}

void outDisengagePoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;
    Eigen::MatrixXd seqMat(3, countMax);

    // ready push out 10cm out
    seqMat(0,0) = dummyPushOut(0) - 0.15;
    seqMat(1,0) = dummyPushOut(1);
    seqMat(2,0) = dummyPushOut(2);

    seqMat(0,1) = readyPushOut(0);
    seqMat(1,1) = readyPushOut(1);
    seqMat(2,1) = readyPushOut(2);

    if (seqCount > 0)
    {
        seqCount -= 1;
        goalPose.position.x = seqMat(0, 1 - seqCount);
        goalPose.position.y = seqMat(1, 1 - seqCount);
        goalPose.position.z = seqMat(2, 1 - seqCount);

    }
    else
    {
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
    }

}

// strcmp(maniState.c_str(),"motion") == 0 && 
// if (strcmp(maniState.c_str(),"stabilize") == 0)



int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_goalPose_generator");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node;

    ros::Publisher desired_pos_pub = node.advertise<geometry_msgs::Pose>("desired_pose", 10);

    // subscribe to Yuqing's goal getter node
    // ros::Subscriber desired_goal_sub = node.subscribe("goal", 10, goalPushOutCallback);
    ros::Duration(1.0).sleep();
    boost::shared_ptr<goal_getter::goal_msg const> goalpose = ros::topic::waitForMessage<goal_getter::goal_msg>("/goal");
    // oal_getter::goal_msg goalpose = ros::topic::waitForMessage("/goal");
    dummyPushOut[0] = goalpose->x;
    dummyPushOut[1] = goalpose->y;
    dummyPushOut[2] = goalpose->z;

    
    ROS_INFO_STREAM("Goal Position Reading: \n" << *goalpose << "\n");
    

    desired_pos_pub.publish(goalPose);


    ros::Rate rate(10.0);

    beginTime = ros::Time::now();

    while(ros::ok())
    {
        // ROS_INFO("testing!!!!");
        ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);
        currTime = ros::Time::now();
        durVar = currTime - beginTime;

        if (strcmp(maniState.c_str(),"random") == 0 && (float) durVar.toSec() > 2.0)
        {
            randomPoseFunc();
            // target_pose1.position.x = 1.0;
            // target_pose1.position.y = -0.07659;
            // target_pose1.position.z = -0.03536;

            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"home") == 0 && (float) durVar.toSec() > 2.0)
        {
            homePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        } 
        else if (strcmp(maniState.c_str(),"ready") == 0 && (float) durVar.toSec() > 2.0)
        {
            readyPoseFunc();
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();   
        }
        else if (strcmp(maniState.c_str(),"push-up") == 0 && (float) durVar.toSec() > 3.0)
        {
            upPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"up-disengage") == 0 && (float) durVar.toSec() > 3.0)
        {
            upDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"push-out") == 0 && (float) durVar.toSec() > 3.0)
        {
            outPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"out-disengage") == 0 && (float) durVar.toSec() > 3.0)
        {
            outDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }

        // ROS_INFO("%d",sequenceCount);



    }


return 0;
}
