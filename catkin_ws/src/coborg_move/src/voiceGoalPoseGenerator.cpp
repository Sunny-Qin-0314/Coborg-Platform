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
#include "std_msgs/Int32.h"
#include <string.h>
#include <gb_visual_detection_3d_msgs/goal_msg.h>

ros::Publisher desired_pos_pub;

// subscribe to state_output rostopic
ros::Subscriber state_output_sub;

// publish to state_input rostopic
ros::Publisher state_input_pub;


geometry_msgs::Pose goalPose;
std::string defOrigin;
std::string currTask = "standby";
std::int8_t sequenceCount = 0;

std::string upTask = "pushup";
std::string upDisengageTask = "updisengage";
std::string outTask = "pushout";
std::string outDisengageTask = "outdisengage";
std::string homeTask = "home";
std::string readyTask = "ready";

ros::Time beginTime;
ros::Time currTime;
ros::Duration durVar;

std::string maniState;
// std::string "camera_link" = "camera_link";
std::string svdTargetVal = "target1";
std::string visionTargetStart = "visiongoal";
std::string visionTargetEnd = "goalset";

float uniWaitSec = 1.5;

// hardcoded three target positions
// TODO: change positions to be relative to the world position
Eigen::Vector3d svdPushOut01(0.895-0.2704, -0.345+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut02(0.895-0.2704, -0.495+0.5209, -0.255+0.70005);
Eigen::Vector3d svdPushOut03(0.895-0.2704, -0.495+0.5209, -0.405+0.70005);
Eigen::Vector3d svdPushOut04(0.895-0.2704-0.1, -0.345+0.5209+0.11, -0.255+0.70005);

Eigen::Vector3d svdPushOut05(0.895-0.2704-0.1, -0.495+0.5209, -0.255+0.70005+0.35);
Eigen::Vector3d svdIntermediatePushOut05(0.302419, 0.115716, 0.718497);
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

tf::TransformListener* globalListener;
tf::StampedTransform* globalTransform;
ros::Publisher* state_input_ptr;

std::string voiceGoTo = "voiceGoal";
std::string voiceGoBack = "voiceHome";
std_msgs::Int32 commandReceived;
std_msgs::Int32 robotExecuting;
std_msgs::Int32 robotDone;
bool commandDone = true;


Eigen::Vector3d presetPositionUpdate(const Eigen::Vector3d& presetVect)
{
    // update home preset pose
    Eigen::Vector3d tempVect;
    tf::StampedTransform transform;

    // try{
    //     globalListener->waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
    //     globalListener->lookupTransform("/world", "/camera_link", ros::Time(0), transform);
    //     // presetPositionUpdate(transform, homePose);
    //     tempVect(0) = -transform.getOrigin().getX() + presetVect(0);
    //     tempVect(1) = -transform.getOrigin().getY() + presetVect(1);
    //     tempVect(2) = -transform.getOrigin().getZ() + presetVect(2);
        
    //     // ROS_INFO("Transform rotation is: x: %f, y: %f, z: %f, w: %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    //     // ROS_INFO("Transform translation is: x: %f, y: %f: z: %f", transform.getOrigin().getX(),transform.getOrigin().getY(), transform.getOrigin().getZ());

    //     return tempVect;

    // }
    // catch (tf::TransformException ex){
    //     ROS_ERROR("%s", ex.what());

    // }

    tempVect(0) = 0.242 + presetVect(0);
    tempVect(1) = -0.069 + presetVect(1);
    tempVect(2) = -0.610 + presetVect(2);

    return tempVect;



    
}


void randomPoseFunc()
{
    if (strcmp(svdTargetVal.c_str(),"d400_link") == 0 || strcmp(svdTargetVal.c_str(),"cam1_link") == 0)
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

    goalPose.position.x = presetPositionUpdate(homePose)(0);
    goalPose.position.y = presetPositionUpdate(homePose)(1);
    goalPose.position.z = presetPositionUpdate(homePose)(2);

    seqCount = 0;



}

void readyPoseFunc()
{
    goalPose.position.x = presetPositionUpdate(readyPushOut)(0);
    goalPose.position.y = presetPositionUpdate(readyPushOut)(1);
    goalPose.position.z = presetPositionUpdate(readyPushOut)(2);

}


void upPoseFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 2;
    if (strcmp(currTask.c_str(),upTask.c_str()) != 0)
    {
        seqCount = 0;
    }

    Eigen::MatrixXd seqMat(3,countMax);
    Eigen::Vector3d offsetBack(0,0,-0.15);
    Eigen::Vector3d offsetForward(0,0,0.05);


    // ready push out 10cm out
    seqMat(0,0) = presetPositionUpdate(dummyPushUp+offsetBack)(0);
    seqMat(1,0) = presetPositionUpdate(dummyPushUp+offsetBack)(1);
    seqMat(2,0) = presetPositionUpdate(dummyPushUp+offsetBack)(2);

    // push into -5cm into
    seqMat(0,1) = presetPositionUpdate(dummyPushUp)(0);
    seqMat(1,1) = presetPositionUpdate(dummyPushUp)(1);
    seqMat(2,1) = presetPositionUpdate(dummyPushUp)(2);

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
    if (strcmp(currTask.c_str(),upDisengageTask.c_str()) != 0)
    {
        seqCount = 0;
    }
    Eigen::MatrixXd seqMat(3, countMax);

    Eigen::Vector3d OffsetBack(0,0,-0.15);
    seqCount = countMax;

    // ready push out 15cm out
    seqMat(0,0) = presetPositionUpdate(dummyPushUp + OffsetBack)(0);
    seqMat(1,0) = presetPositionUpdate(dummyPushUp + OffsetBack)(1);
    seqMat(2,0) = presetPositionUpdate(dummyPushUp + OffsetBack)(2);

    seqMat(0,1) = presetPositionUpdate(readyPushOut)(0);
    seqMat(1,1) = presetPositionUpdate(readyPushOut)(1);
    seqMat(2,1) = presetPositionUpdate(readyPushOut)(2);

    seqMat(0,2) = presetPositionUpdate(homePose)(0);
    seqMat(1,2) = presetPositionUpdate(homePose)(1);
    seqMat(2,2) = presetPositionUpdate(homePose)(2);

    std::string endParamSet = "home";
    bool paramSet = false;

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
    if (strcmp(currTask.c_str(),outTask.c_str()) != 0)
    {
        seqCount = 0;
    }

    Eigen::MatrixXd seqMat(3,countMax);
    Eigen::Vector3d OffsetBack(-0.15,0,0);
    Eigen::Vector3d OffsetForward(0.05, 0, 0);

    // special case of top extreme intermediate pose


    if (strcmp(svdTargetVal.c_str(), visionTargetEnd.c_str()) == 0)
    {
        // ready push out 15cm out
        seqMat(0,0) = dummyPushOut(0) + OffsetBack(0);
        seqMat(1,0) = dummyPushOut(1);
        seqMat(2,0) = dummyPushOut(2);

        // push into -5cm into
        seqMat(0,1) = dummyPushOut(0) + OffsetForward(0);
        seqMat(1,1) = dummyPushOut(1);
        seqMat(2,1) = dummyPushOut(2);
    }
    else
    {
        // ready push out 15cm out
        seqMat(0,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(0);
        seqMat(1,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(1);
        seqMat(2,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(2);

        // push into -5cm into
        seqMat(0,1) = presetPositionUpdate(dummyPushOut)(0);
        seqMat(1,1) = presetPositionUpdate(dummyPushOut)(1);
        seqMat(2,1) = presetPositionUpdate(dummyPushOut)(2);        
    }

    if (strcmp(svdTargetVal.c_str(),"target5") == 0)
    {
        seqMat(0,0) = presetPositionUpdate(svdIntermediatePushOut05)(0);
        seqMat(1,0) = presetPositionUpdate(svdIntermediatePushOut05)(1);
        seqMat(2,0) = presetPositionUpdate(svdIntermediatePushOut05)(2);
    }

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
    std::int8_t countMax = 3;
    if (strcmp(currTask.c_str(),outDisengageTask.c_str()) != 0)
    {
        seqCount = 0;
    }
    Eigen::MatrixXd seqMat(3, countMax);
    Eigen::Vector3d OffsetBack(-0.15,0,0);

    // ready push out 15cm out
    seqMat(0,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(0);
    seqMat(1,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(1);
    seqMat(2,0) = presetPositionUpdate(dummyPushOut + OffsetBack)(2);

    // // intermediate home pose used to speed up IK solving to home position
    // // NOTE: does not work
    // seqMat(0,1) = intermediateHomePose(0);
    // seqMat(1,1) = intermediateHomePose(1);
    // seqMat(2,1) = intermediateHomePose(2);

    seqMat(0,1) = presetPositionUpdate(readyPushOut)(0);
    seqMat(1,1) = presetPositionUpdate(readyPushOut)(1);
    seqMat(2,1) = presetPositionUpdate(readyPushOut)(2);

    seqMat(0,2) = presetPositionUpdate(homePose)(0);
    seqMat(1,2) = presetPositionUpdate(homePose)(1);
    seqMat(2,2) = presetPositionUpdate(homePose)(2);

    if (strcmp(svdTargetVal.c_str(),"target5") == 0)
    {
        seqMat(0,0) = presetPositionUpdate(svdIntermediatePushOut05)(0);
        seqMat(1,0) = presetPositionUpdate(svdIntermediatePushOut05)(1);
        seqMat(2,0) = presetPositionUpdate(svdIntermediatePushOut05)(2);
    }

    std::string endParamSet = "home";
    bool paramSet = false;

    if (seqCount == 0 && !paramSet)
    {
        state_input_ptr->publish(commandReceived);
        state_input_ptr->publish(robotExecuting);
        commandDone = false;
    }

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
        paramSet = true;
        if (!commandDone)
        {
            state_input_ptr->publish(robotDone);
            commandDone = true;
        }
        ros::param::set("tf_moveit_goalsetNode/manipulation_state", endParamSet);
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
        // dummyPushOut = svdPushOut01;
        return svdPushOut01;
    }
    else if (strcmp(svdTargetVal.c_str(),"target2") == 0)
    {
        // dummyPushOut = svdPushOut02;
        return svdPushOut02;
    }
    else if(strcmp(svdTargetVal.c_str(),"target3") == 0)
    {
        // dummyPushOut = svdPushOut03;
        return svdPushOut03;
    }
    else if(strcmp(svdTargetVal.c_str(),"target4") == 0)
    {
        return svdPushOut04;
    }
    else if(strcmp(svdTargetVal.c_str(),"target5") == 0)
    {
        return svdPushOut05;
    }
    else if(strcmp(svdTargetVal.c_str(), visionTargetStart.c_str()) == 0)
    {
        ros::Time begin_vision_time = ros::Time::now();
        std::cout << svdTargetVal.c_str() << std::endl;
        Eigen::Vector3d goalSetPose;
        // goal is relative to camera frame

        ros::Duration(1.0).sleep();
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose = ros::topic::waitForMessage<gb_visual_detection_3d_msgs::goal_msg>("/goal");

        ROS_INFO("Position Received");

        goalSetPose(0) = goalpose->x;
        goalSetPose(1) = goalpose->y;
        goalSetPose(2) = goalpose->z;

        ROS_INFO_STREAM("Goal Position Reading: \n" << *goalpose << "\n");

        Eigen::Vector3d measuredNormal;
        Eigen::Vector3d groundTruthNormal(1, 0, 0);

        measuredNormal[0] = goalpose->normal_x;
        measuredNormal[1] = goalpose->normal_y;
        measuredNormal[2] = goalpose->normal_z;

        // std::cout << "Measured Normal Vector: " << std::endl;
        // std::cout << measuredNormal << std::endl;

        float angle = groundTruthNormal.dot(measuredNormal);
        angle = acos (angle);

        std::cout << "Angle difference between ground truth and measured normals: " << std::endl;
        std::cout << angle * (180/3.14159) << " degrees" << std::endl;

        ros::Time end_vision_time = ros::Time::now();
        std::cout << "Time: " << end_vision_time.sec - begin_vision_time.sec << " seconds" << std::endl;

        
        ros::param::set("voiceGoalPoseGenerator/svdTarget", visionTargetEnd);


        return goalSetPose;

    }
    else
    {
        return dummyPushOut;
    }

}

// publish 1 when receiving any command
// publish 2 when executing
// publish 3 when done

// outputVal = 2 ->
// publish into state input 1
// publish into state input 2
// grab point from hands, go to home, go to ready, go to target -15cm
// go to target
// publish into state_input std_msgs::Int32 = 3

// outputVal = 3 ->
// publish into state input 1
// publish into state input 2
// go to target -15cm, go to ready, go to home
// pubiish into state_input std::msgs::Int32 = 3




void state_output_callback(const std_msgs::Int32::ConstPtr& outputVal)
{

    if (outputVal->data == 2)
    {
        // go to goal
        ros::param::set("tf_moveit_goalsetNode/manipulation_state", voiceGoTo);
        ros::param::set("voiceGoalPoseGenerator/svdTarget", visionTargetStart);
        state_input_ptr->publish(commandReceived);

    }
    else if (outputVal->data == 3)
    {
        // go to home
        ros::param::set("tf_moveit_goalsetNode/manipulation_state", voiceGoBack);
        ros::param::set("voiceGoalPoseGenerator/svdTarget", visionTargetStart);
        state_input_ptr->publish(commandReceived);

    }
}

void voiceGoalFunc(std::int8_t& seqCount)
{
    std::int8_t countMax = 4;
    if (strcmp(currTask.c_str(),outTask.c_str()) != 0)
    {
        seqCount = 0;
    }

    Eigen::MatrixXd seqMat(3,countMax);
    Eigen::Vector3d OffsetBack(-0.15,0,0);
    Eigen::Vector3d OffsetForward(0.02, 0, 0);

    // go to home
    // publish 2 into state_input
    // go to ready
    // go to position -15cm
    // go to position
    // publish 3 into state input
    if (strcmp(svdTargetVal.c_str(), visionTargetStart.c_str()) == 0)
    {
        dummyPushOut = svdTargetFunc(svdTargetVal);
        ros::param::set("voiceGoalPoseGenerator/svdTarget", visionTargetEnd);
    }

    if (seqCount == 0)
    {
        state_input_ptr->publish(robotExecuting);
        commandDone = false;
        // ROS_INFO("Robot is moving");
    }


    if (strcmp(svdTargetVal.c_str(), visionTargetEnd.c_str()) == 0)
    {
        // go to home
        seqMat(0,0) = presetPositionUpdate(homePose)(0);
        seqMat(1,0) = presetPositionUpdate(homePose)(1);
        seqMat(2,0) = presetPositionUpdate(homePose)(2);

        // go to ready
        seqMat(0,1) = presetPositionUpdate(readyPushOut)(0);
        seqMat(1,1) = presetPositionUpdate(readyPushOut)(1);
        seqMat(2,1) = presetPositionUpdate(readyPushOut)(2);


        // ready push out 15cm out
        seqMat(0,2) = dummyPushOut(0) + OffsetBack(0);
        seqMat(1,2) = dummyPushOut(1);
        seqMat(2,2) = dummyPushOut(2);
        // push into -5cm into
        seqMat(0,3) = dummyPushOut(0) + OffsetForward(0);
        seqMat(1,3) = dummyPushOut(1);
        seqMat(2,3) = dummyPushOut(2);
    }


    if (seqCount < countMax)
    {
        // ROS_INFO("Ready to push");
        ROS_INFO("%d", seqCount);
        goalPose.position.x = seqMat(0, seqCount);
        goalPose.position.y = seqMat(1, seqCount);
        goalPose.position.z = seqMat(2, seqCount);
        seqCount += 1;
        


    }
    else
    {
        // ROS_INFO("Pushing Now");
        goalPose.position.x = seqMat(0, countMax-1);
        goalPose.position.y = seqMat(1, countMax-1);
        goalPose.position.z = seqMat(2, countMax-1);
        if (!commandDone)
        {
            state_input_ptr->publish(robotDone);
            commandDone = true;
        }
    }

}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "random_goalPose_generator");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::NodeHandle node;

    ros::Time begin_vision_time = ros::Time::now();
    desired_pos_pub = node.advertise<geometry_msgs::Pose>("desired_pose", 1);

    // subscribe to state_output rostopic
    state_output_sub = node.subscribe("state_output", 1, state_output_callback);

    // publish to state_input rostopic
    state_input_pub = node.advertise<std_msgs::Int32>("state_input", 1);
    state_input_ptr = &state_input_pub;

    // update transform between urdf world and d400/camera_link frame
    tf::TransformListener listener;
    tf::StampedTransform transform;


    globalListener = &listener;


    commandReceived.data = 1;
    robotExecuting.data = 2;
    robotDone.data = 3;



    ros::Rate rate(20.0);

    beginTime = ros::Time::now();

    while(ros::ok())
    {
        ros::param::get("tf_moveit_goalsetNode/manipulation_state",maniState);

        ros::param::get("voiceGoalPoseGenerator/svdTarget", svdTargetVal);
        // ros::param::get("voiceGoalPoseGenerator/originState", "camera_link");



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
            currTask = homeTask;
        }
        else if (strcmp(maniState.c_str(),"ready") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            readyPoseFunc();
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = readyTask;
        }
        else if (strcmp(maniState.c_str(),"push-up") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            upPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = upTask;
        }
        else if (strcmp(maniState.c_str(),"up-disengage") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            upDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = upDisengageTask;
        }
        else if (strcmp(maniState.c_str(),"push-out") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            outPoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = outTask;
        }
        else if (strcmp(maniState.c_str(),"out-disengage") == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            outDisengagePoseFunc(sequenceCount);
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = outDisengageTask;
        }
        else if (strcmp(maniState.c_str(), voiceGoTo.c_str()) == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            // ROS_INFO("Voice Function Going to Be Called Now");
            voiceGoalFunc(sequenceCount);
            if (goalPose.position.x < 0.001)
            {
                continue;
            }
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = outTask;
        }
        else if (strcmp(maniState.c_str(), voiceGoBack.c_str()) == 0 && (float) durVar.toSec() > uniWaitSec)
        {
            outDisengagePoseFunc(sequenceCount);
            if (goalPose.position.x < 0.001)
            {
                continue;
            }
            desired_pos_pub.publish(goalPose);
            beginTime = ros::Time::now();
            currTask = outDisengageTask;            
        }

        ros::spinOnce();
        rate.sleep();

    }


return 0;
}