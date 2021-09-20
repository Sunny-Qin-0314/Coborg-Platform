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
#include <gb_visual_detection_3d_msgs/goal_msg.h>


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

// hardcoded three target positions
// TODO: change positions to be relative to the world position
Eigen::Vector3d svdPushOut01(0.895-0.2704, -0.345+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut02(0.895-0.2704, -0.495+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut03(0.895-0.2704, -0.495+0.5209, -0.405+0.70005);



// hard coded position poses
// presets are created relative to the urdf world frame
// Eigen::Vector3d intermediateHomePose(0.638892, -0.504, -0.420819);
// Eigen::Vector3d homePose(0.2569, -0.60749, -0.5831);
// Eigen::Vector3d readyPushOut(0.7022, -0.3670, -0.08145);
// Eigen::Vector3d dummyPushUp(0.30457, -0.3261, 0.3151);
// Eigen::Vector3d dummyPushOut(0.895, -0.345, -0.255);

// Eigen::Vector3d dummyPushOut = svdPushOut01;

Eigen::Vector3d homePose(-0.0144186, -0.0856238, 0.116201);
Eigen::Vector3d readyPushOut(0.353297, 0.11838, 0.342266);
Eigen::Vector3d dummyPushOut(0.626008, 0.174859, 0.438969);
Eigen::Vector3d dummyPushUp(0.0294834, 0.190455, 1.06668);

tf::StampedTransform* globalTransform;

Eigen::Vector3d presetPositionUpdate(const tf::StampedTransform *transformPtr, const Eigen::Vector3d& presetVect)
{
    // update home preset pose
    Eigen::Vector3d tempVect;

    tempVect(0) = -transformPtr->getOrigin().getX() + presetVect(0);
    tempVect(1) = -transformPtr->getOrigin().getY() + presetVect(1);
    tempVect(2) = -transformPtr->getOrigin().getZ() + presetVect(2);

    return tempVect;
}


void randomPoseFunc()
{
    if (strcmp(svdTargetVal.c_str(),"d400_link") == 0 || strcmp(svdTargetVal.c_str(),"camera_link") == 0)
    {
        // relative to d400_link / camera_link frame at default static transform from t265_link
        // float xRangeMin = 0.6;
        // float xRangeMax = 0.9;
        // float yRangeMin = -0.4;
        // float yRangeMax = -0.15;
        // float zRangeMin = -0.25;
        // float zRangeMax = -0.15;
        float xRangeMin = 0.6;
        float xRangeMax = 0.9;
        float yRangeMin = 0.0;
        float yRangeMax = 0.3;
        float zRangeMin = 0.1;
        float zRangeMax = 0.4;

        float randomX = ((float) rand()) / (float) RAND_MAX;
        float randomY = ((float) rand()) / (float) RAND_MAX;
        float randomZ = ((float) rand()) / (float) RAND_MAX;
        goalPose.position.x = randomX*(xRangeMax - xRangeMin) + xRangeMin;
        goalPose.position.y = randomY*(yRangeMax - yRangeMin) + yRangeMin;
        goalPose.position.z = randomZ*(zRangeMax - zRangeMin) + zRangeMin;
    }
    else if (strcmp(svdTargetVal.c_str(),"rodL") == 0)
    {
        // relative to rodL frame
        // float xRangeMin = 0.6;
        // float xRangeMax = 0.9;
        // float yRangeMin = -0.3;
        // float yRangeMax = -0.1;
        // float zRangeMin = -0.2;
        // float zRangeMax = -0.05;
        float xRangeMin = 0.6;
        float xRangeMax = 0.9;
        float yRangeMin = 0.0;
        float yRangeMax = 0.3;
        float zRangeMin = 0.1;
        float zRangeMax = 0.4;

        float randomX = ((float) rand()) / (float) RAND_MAX;
        float randomY = ((float) rand()) / (float) RAND_MAX;
        float randomZ = ((float) rand()) / (float) RAND_MAX;
        goalPose.position.x = randomX*(xRangeMax - xRangeMin) + xRangeMin;
        goalPose.position.y = randomY*(yRangeMax - yRangeMin) + yRangeMin;
        goalPose.position.z = randomZ*(zRangeMax - zRangeMin) + zRangeMin;
    }
    else
    {
        ROS_INFO("Origin frame not preconfigured for random pose generation");
    }

}


void homePoseFunc(std::int8_t& seqCount)
{

    goalPose.position.x = presetPositionUpdate(globalTransform, homePose)(0);
    goalPose.position.y = presetPositionUpdate(globalTransform, homePose)(1);
    goalPose.position.z = presetPositionUpdate(globalTransform, homePose)(2);

    seqCount = 0;



}

void readyPoseFunc()
{
    goalPose.position.x = presetPositionUpdate(globalTransform, readyPushOut)(0);
    goalPose.position.y = presetPositionUpdate(globalTransform, readyPushOut)(1);
    goalPose.position.z = presetPositionUpdate(globalTransform, readyPushOut)(2);
}


void upPoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;

    Eigen::MatrixXd seqMat(3,countMax);
    Eigen::Vector3d offsetBack(0,0,-0.1);
    Eigen::Vector3d offsetForward(0,0,0.05);


    // ready push out 10cm out
    seqMat(0,0) = presetPositionUpdate(globalTransform, dummyPushUp)(0);
    seqMat(1,0) = presetPositionUpdate(globalTransform, dummyPushUp)(1);
    seqMat(2,0) = presetPositionUpdate(globalTransform, dummyPushUp+offsetBack)(2);

    // push into -5cm into
    seqMat(0,1) = presetPositionUpdate(globalTransform, dummyPushUp)(0);
    seqMat(1,1) = presetPositionUpdate(globalTransform, dummyPushUp)(1);
    seqMat(2,1) = presetPositionUpdate(globalTransform, dummyPushUp+offsetForward)(2);

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
    std::int8_t countMax = 3;
    Eigen::MatrixXd seqMat(3, countMax);

    Eigen::Vector3d OffsetBack(0,0,-0.15);

    // ready push out 15cm out
    seqMat(0,0) = presetPositionUpdate(globalTransform, dummyPushUp)(0);
    seqMat(1,0) = presetPositionUpdate(globalTransform, dummyPushUp)(1);
    seqMat(2,0) = presetPositionUpdate(globalTransform, dummyPushUp + OffsetBack)(2);

    seqMat(0,1) = presetPositionUpdate(globalTransform, readyPushOut)(0);
    seqMat(1,1) = presetPositionUpdate(globalTransform, readyPushOut)(1);
    seqMat(2,1) = presetPositionUpdate(globalTransform, readyPushOut)(2);

    seqMat(0,2) = presetPositionUpdate(globalTransform, homePose)(0);
    seqMat(1,2) = presetPositionUpdate(globalTransform, homePose)(1);
    seqMat(2,2) = presetPositionUpdate(globalTransform, homePose)(2);

    std::string endParamSet = "home";
    bool paramSet = false;

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
        paramSet = true;
    }

    if (paramSet && seqCount <= 0)
    {
        ros::param::set("tf_moveit_goalsetNode/manipulation_state", endParamSet);

        paramSet = false;
    }

}

void outPoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;

    Eigen::MatrixXd seqMat(3,countMax);
    Eigen::Vector3d OffsetBack(-0.15,0,0);
    Eigen::Vector3d OffsetForward(0.1, 0, 0);

    if (strcmp(svdTargetVal.c_str(), visionTargetEnd.c_str()) == 0)
    {
        // ready push out 15cm out
        seqMat(0,0) = dummyPushOut(0) + OffsetBack(0);
        seqMat(1,0) = dummyPushOut(1);
        seqMat(2,0) = dummyPushOut(2);

        // push into -5cm into
        seqMat(0,1) = dummyPushOut(0);
        seqMat(1,1) = dummyPushOut(1);
        seqMat(2,1) = dummyPushOut(2);
    }
    else
    {
        // ready push out 15cm out
        seqMat(0,0) = presetPositionUpdate(globalTransform, dummyPushOut + OffsetBack)(0);
        seqMat(1,0) = presetPositionUpdate(globalTransform, dummyPushOut)(1);
        seqMat(2,0) = presetPositionUpdate(globalTransform, dummyPushOut)(2);

        // push into -5cm into
        seqMat(0,1) = presetPositionUpdate(globalTransform, dummyPushOut)(0);
        seqMat(1,1) = presetPositionUpdate(globalTransform, dummyPushOut)(1);
        seqMat(2,1) = presetPositionUpdate(globalTransform, dummyPushOut)(2);        
    }


    if (seqCount < countMax)
    {
        // ROS_INFO("Ready to push");
        goalPose.position.x = seqMat(0, seqCount);
        goalPose.position.y = seqMat(1, seqCount);
        goalPose.position.z = seqMat(2, seqCount);
        seqCount += 1;

    }
    else
    {
        ROS_INFO("Pushing Now");
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
    }

}

void outDisengagePoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 3;
    Eigen::MatrixXd seqMat(3, countMax);
    Eigen::Vector3d OffsetBack(-0.20,0,0);

    // ready push out 15cm out
    seqMat(0,0) = presetPositionUpdate(globalTransform, dummyPushOut + OffsetBack)(0);
    seqMat(1,0) = presetPositionUpdate(globalTransform, dummyPushOut)(1);
    seqMat(2,0) = presetPositionUpdate(globalTransform, dummyPushOut)(2);

    // seqMat(0,1) = presetPositionUpdate(globalTransform, readyPushOut)(0);
    // seqMat(1,1) = presetPositionUpdate(globalTransform, readyPushOut)(1);
    // seqMat(2,1) = presetPositionUpdate(globalTransform, readyPushOut)(2);

    // // intermediate home pose used to speed up IK solving to home position
    // // NOTE: does not work
    // seqMat(0,1) = intermediateHomePose(0);
    // seqMat(1,1) = intermediateHomePose(1);
    // seqMat(2,1) = intermediateHomePose(2);

    seqMat(0,1) = presetPositionUpdate(globalTransform, homePose)(0);
    seqMat(1,1) = presetPositionUpdate(globalTransform, homePose)(1);
    seqMat(2,1) = presetPositionUpdate(globalTransform, homePose)(2);

    std::string endParamSet = "home";
    bool paramSet = false;

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
        paramSet = true;
    }

    if (paramSet && seqCount <= 0)
    {
        ros::param::set("tf_moveit_goalsetNode/manipulation_state", endParamSet);

        paramSet = false;
    }

}

Eigen::Vector3d svdTargetFunc(std::string& svdTargetVal)
{

    if (strcmp(svdTargetVal.c_str(),"target1") == 0)
    {
        // ROS_INFO("Position 01");
        // dummyPushOut = svdPushOut01;
        return svdPushOut01;
    }
    else if (strcmp(svdTargetVal.c_str(),"target2") == 0)
    {
        // ROS_INFO("Position 02");
        // dummyPushOut = svdPushOut02;
        return svdPushOut02;
    }
    else if(strcmp(svdTargetVal.c_str(),"target3") == 0)
    {
        // ROS_INFO("Position 03");
        // dummyPushOut = svdPushOut03;
        return svdPushOut03;
    }
    else if(strcmp(svdTargetVal.c_str(), visionTargetStart.c_str()) == 0)
    {
        std::cout << svdTargetVal.c_str() << std::endl;
        Eigen::Vector3d goalSetPose;
        ROS_INFO("Waiting for message from goal rostopic");
        // goal is relative to camera frame
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose = ros::topic::waitForMessage<gb_visual_detection_3d_msgs::goal_msg>("/goal");

        ROS_INFO("Position Received");

        goalSetPose(0) = goalpose->x;
        goalSetPose(1) = goalpose->y;
        goalSetPose(2) = goalpose->z;

        ROS_INFO_STREAM("Goal Position Reading: \n" << *goalpose << "\n");
        
        ros::param::set("randomGoalPoseGenerator/svdTarget", visionTargetEnd);


        return goalSetPose;

    }
    else
    {
        return dummyPushOut;
    }

}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "random_goalPose_generator");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node;

    ros::Time begin_vision_time = ros::Time::now();
    ros::Publisher desired_pos_pub = node.advertise<geometry_msgs::Pose>("desired_pose", 1);

    // // Feng Xiang code for vision SVD
    // // code begin
    // // subscribe to Yuqing's goal getter node
    // // ros::Subscriber desired_goal_sub = node.subscribe("goal", 10, goalPushOutCallback);
    // ros::Duration(1.0).sleep();
    // boost::shared_ptr<goal_getter::goal_msg const> goalpose = ros::topic::waitForMessage<goal_getter::goal_msg>("/goal");
    // // oal_getter::goal_msg goalpose = ros::topic::waitForMessage("/goal");
    // // dummyPushOut[0] = goalpose->x;
    // // dummyPushOut[1] = goalpose->y;
    // // dummyPushOut[2] = goalpose->z;

    // ROS_INFO_STREAM("Goal Position Reading: \n" << *goalpose << "\n");

    // // compute dot product
    // // 1 * {surface normal_x} / sqrt(norm(x^2 + y^2 + z^2))
    // Eigen::Vector3d measuredNormal;
    // Eigen::Vector3d groundTruthNormal(1, 0, 0);

    // measuredNormal[0] = goalpose->normal_x;
    // measuredNormal[1] = goalpose->normal_y;
    // measuredNormal[2] = goalpose->normal_z;

    // // std::cout << "Measured Normal Vector: " << std::endl;
    // // std::cout << measuredNormal << std::endl;

    // float angle = groundTruthNormal.dot(measuredNormal);
    // angle = acos (angle);

    // std::cout << "Angle difference between ground truth and measured normals: " << std::endl;
    // std::cout << angle * (180/3.14159) << " degrees" << std::endl;

    // ros::Time end_vision_time = ros::Time::now();
    // std::cout << "Time: " << end_vision_time.sec - begin_vision_time.sec << " seconds" << std::endl;


    // desired_pos_pub.publish(goalPose);
    // svdPushOut01(0) = goalpose->x;
    // svdPushOut01(1) = goalpose->y;
    // svdPushOut01(2) = goalpose->z;

    // // code end

    // update transform between urdf world and d400/camera_link frame
    tf::TransformListener listener;
    tf::StampedTransform transform;

    globalTransform = &transform;
    



    ros::Rate rate(20.0);

    beginTime = ros::Time::now();

    while(ros::ok())
    {
        ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);

        ros::param::get("randomGoalPoseGenerator/svdTarget", svdTargetVal);
        ros::param::get("randomGoalPoseGenerator/originState", vision_reference_frame);
        try{
            listener.lookupTransform("world", vision_reference_frame.c_str(), ros::Time(0), transform);
            // ROS_INFO("%s", vision_reference_frame.c_str());
            // presetPositionUpdate(transform, homePose);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
        // presetPositionUpdate(transform);
        dummyPushOut = svdTargetFunc(svdTargetVal);

        // if ((float) durVar.toSec() > 0.25)
        // {
        //     ROS_INFO_STREAM("Current Pose Target: " << svdTargetVal.c_str() << "\n");
        //     ROS_INFO_STREAM("Current Manipulation State: " << maniState.c_str() << "\n");
        // }


        currTime = ros::Time::now();
        durVar = currTime - beginTime;
        if (strcmp(maniState.c_str(),"random") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            randomPoseFunc();
            // // preset pose declared for debugging
            // target_pose1.position.x = 1.0;
            // target_pose1.position.y = -0.07659;
            // target_pose1.position.z = -0.03536;

            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"home") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            homePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"ready") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            readyPoseFunc();
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"push-up") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            upPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"up-disengage") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            upDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"push-out") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            outPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }
        else if (strcmp(maniState.c_str(),"out-disengage") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            outDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
        }


    }


return 0;
}