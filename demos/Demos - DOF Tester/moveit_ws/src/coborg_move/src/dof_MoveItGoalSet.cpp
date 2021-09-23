#include "dof_MoveItGoalSet.hpp"


// triggers when camera_link goal pose is published
void poseTransformCallback(const geometry_msgs::Pose::ConstPtr& posemsg)
{
    // ros::param::get("voiceGoalPoseGenerator/originState", "camera_link");

    // TODO: convert goal pose from camera_link to URDF world frame
    // FORNOW: pose converted from URDF rodL to URDF world frame
    tf::StampedTransform transform;
    // try
    // {

    //     // TODO: acquire the transform once
    //     // FORNOW: update transform parameters at every callback interval
    //     // listPoint->waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
    //     // listPoint->lookupTransform("/world", "/camera_link",ros::Time(0), transform);

    //     // FORNOW: only goal position is updated b/c 3DoF robot arm cannot solve 6DoF goal every time
    //     target_pose1.position.x = -0.242 + posemsg->position.x;
    //     target_pose1.position.y = 0.069 + posemsg->position.y;
    //     target_pose1.position.z = 0.610 + posemsg->position.z;

    //     ROS_INFO("Transforms are: x: %f, y: %f: z: %f", target_pose1.position.x,target_pose1.position.y, target_pose1.position.z);

    //     // robot arm can now move to updated goal pose
    //     moveReady = true;
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    // }

    target_pose1.position.x = -0.242 + posemsg->position.x;
    target_pose1.position.y = 0.069 + posemsg->position.y;
    target_pose1.position.z = 0.610 + posemsg->position.z;
    moveReady = true;

    // // debugging commands
    // ROS_INFO("callback received!!!");
    // ROS_INFO("now this is pod racing!");
    // ROS_INFO("Transform rotation is: x: %f, y: %f, z: %f, w: %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    // ROS_INFO("Transform translation is: x: %f, y: %f: z: %f", transform.getOrigin().getX(),transform.getOrigin().getY(), transform.getOrigin().getZ());
    // ROS_INFO("Frame ID: %s",transform.frame_id_.c_str());
}


// subscribes current hebi joint angles rostopic
void hebiJointsCallback(const geometry_msgs::Twist::ConstPtr& hebimsg)
{
    //compute foward kinematics of robot
    hebiJointAngles.at(0) = hebimsg->linear.x;
    hebiJointAngles.at(1) = hebimsg->linear.y;
    hebiJointAngles.at(2) = hebimsg->linear.z;
    hebiJointAngVelocities.at(0) = hebimsg->angular.x;
    hebiJointAngVelocities.at(1) = hebimsg->angular.y;
    hebiJointAngVelocities.at(2) = hebimsg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_moveit_goalsetNode");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle node;

    // declare tf listener node and make global
    tf::TransformListener listener;
    listPoint = &listener;

    ROS_INFO("Loading robot model");

    // set and configure to planning group
    std::string PLANNING_GROUP;
    ros::param::get("dof_PoseGenerator/arm_name",PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    movePoint = &move_group;

    // set and configure to URDF / robot description
    // load robot model into node
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    // ROS: The RobotModel class contains the relationships between all links and joints including their
    // joint limit properties as loaded from the URDF.

    // load robot state into node
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    kin_model = &robot_state;
    // ROS: The RobotState contains information about the robot at a snapshot in time, storing vectors of
    // joint positions and optionally velocities and accelerations that can be used to obtain kinematic
    // information about the robot that depends on it's current state such as the Jacobian of an end effector

    // load joint model group into node
    // joint model group is connected to robot state
    ROS_INFO("Declaring joint_model_group");
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;

    // declare matrix to compute goal joint angles (optional)
    // Eigen::Isometry3d target_joints;
    std::vector<double> target_joints;

    // get the current state of robot to initialize
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // tie start state to address of the current state of the robot
    moveit::core::RobotState start_state(*move_group.getCurrentState());
    // initialize planning interface variable
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    myPlanPoint = &my_plan;

    // set max planning time for robot arm
    // setting value too low can cause arm to fail planning when there is plan available
    // setting value too high can cause arm to get stuck planning a path that is not possible
    move_group.setPlanningTime(2.0);
    // set goal tolerance for move_group
    move_group.setGoalTolerance(0.005);

    // print out debugging information into terminal
    ROS_INFO("Printing Basic Information Now:");
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));


    // declare RViz interfacing variables
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    // ROS: The package MoveItVisualtools provides many capabilities for visualizing objects, robots
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script



    // declare home and ready global variables


    std::string degreesFreedom;
    ros::param::get("dof_PoseGenerator/degrees_of_freedom",degreesFreedom);

    presetPositionNames.push_back("home");
    presetPositionNames.push_back("ready");


    if (strcmp(degreesFreedom.c_str(),"3a") == 0)
    {
        std::cout << "Three Degrees of Freedom" << std::endl;
        //declare preset joint positions
        // home: -110, -136, -144 [deg]
        // home: -1.91986, -2.37365, -2.51327 [rad]

        // ready: -54, -126, -87 [deg]
        // ready: -0.942478, -2.19911, -1.51844 [rad]
        presetJointValues.push_back({-1.91986, -2.37365, -2.51327});
        presetJointValues.push_back({-0.942478, -2.19911, -1.51844});
    }
    else if (strcmp(degreesFreedom.c_str(),"4a") == 0)
    {
        std::cout << "Four Degrees of Freedom" << std::endl;
        presetJointValues.push_back({0.0, -1.91986, -2.37365, -2.51327});
        presetJointValues.push_back({0.0, -0.942478, -2.19911, -1.51844});
    }
    else if (strcmp(degreesFreedom.c_str(),"5a") == 0)
    {
        std::cout << "Config Z 5 Degrees of Freedom" << std::endl;

        presetJointValues.push_back({0.0, -1.9266, 0.0, -2.0307, -2.3778});
        presetJointValues.push_back({0.1, -0.9424, 0.5, -2.0, -1.5});
    }
    else if (strcmp(degreesFreedom.c_str(),"5b") == 0)
    {
        std::cout << "Config Y 5 Degrees of Freedom" << std::endl;

        presetJointValues.push_back({0.0, -1.9266, 0.0, -2.0307, -2.3778});
        presetJointValues.push_back({0.1, -0.9424, 0.0, -2.0, -1.5});        
    }

    // (impedance control) publish to desired hebi torques rostopic
    ros::Publisher desired_efforts_pub = node.advertise<geometry_msgs::Vector3>("desired_hebi_efforts", 1);
    // (impedance control) subscribe to current hebi joint angles rostopic
    ros::Subscriber hebi_joints_sub = node.subscribe("hebi_joints", 1, hebiJointsCallback);

    // subscribe to desired pose rostopic
    // ros::Subscriber sent_msg_sub = node.subscribe("desired_pose", 1, poseTransformCallback);

    // (stabilization and impedance control) declaring variables
    geometry_msgs::Pose target_push_pose2;
    float xErrP;
    float yErrP;
    float zErrP;
    float xErrD;
    float yErrD;
    float zErrD;
    float impGainXp = 50.0;
    float impGainYp = 4.0;
    float impGainZp = 4.0;
    float impGainXd = 2.0;
    float impGainYd = 2.0;
    float impGainZd = 2.0;
    float desVelAngX = 0.0;
    float desVelAngY = 0.0;
    float desVelAngZ = 0.0;
    Eigen::VectorXd endLinkEffort(6);
    Eigen::VectorXd jointVelocityVect(3);
    Eigen::VectorXd taskSpaceVelocityVect;
    Eigen::MatrixXd wristJacobian;
    Eigen::VectorXd desiredTorquesEigen;
    geometry_msgs::Vector3 desired_torques;
    float baseMaxTorque = 2.0;
    float elbowMaxTorque = 2.0;
    float wristMaxTorque = 2.0;

    Eigen::Isometry3d actual_endeff_state;

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

    ros::Rate rate(20.0);
    beginTime = ros::Time::now();


    while(ros::ok())
    {
        beginTime = ros::Time::now();

        // get the current state of the robot
        ros::param::get("dof_PoseGenerator/manipulation_state",maniState);
        
        // currTime = ros::Time::now();
        // durVar = currTime - beginTime;

        if(!moveReady)
        {
            boost::shared_ptr<geometry_msgs::Pose const> desired_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/desired_pose");

            poseTransformCallback(desired_pose);
        }

        if (moveReady)
        {

            for (int i = 0; i < presetPositionNames.size(); i++)
            {
                ROS_INFO("The state of manipulation is: %s", maniState.c_str());
                ROS_INFO("Position Name is: %s", presetPositionNames[i].c_str());

                if (strcmp(maniState.c_str(),presetPositionNames[i].c_str()) == 0)
                {
                    int presetIndex = i;
                    // std::cout << presetJointValues[presetIndex][0] << std::endl;
                    std::vector<double> joint_group_positions;
                    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
                    for (unsigned int it = 0; it < joint_group_positions.size(); it++)
                    {
                        joint_group_positions[it] = presetJointValues[presetIndex][it];
                    }

                    move_group.setJointValueTarget(joint_group_positions);
                    moveitSuccess = move_group.plan(my_plan);

                    move_group.execute(my_plan);
                    moveReady = false;

                    break;
                }
            }

            if (moveReady == false)
            {
                continue;
            }

            move_group.setJointValueTarget(target_pose1,"end_link/INPUT_INTERFACE"); //considered a workaround solution

            moveitSuccess = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (!moveitSuccess)
            {
                continue;
            }
            std::cout << "Successful Trajectory: " << moveitSuccess << std::endl;

            
            // execute plan to move_group
            move_group.execute(my_plan);
            moveReady = false;

            actual_endeff_state = current_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
       

        }
        else if (strcmp(maniState.c_str(),"impedance") == 0)
        {
            
            // // Option 2: impedance control based on joint angle errors
            // // compute joint angles of 

            // // compute joint angles at the pushed target pose
            // target_push_pose2 = target_pose1;
            // target_push_pose2.position.z += 0.05;


            // Option 1: impedance control based on x, y, z errors
            target_push_pose2 = target_pose1;
            target_push_pose2.position.x += 0.25;
            jointVelocityVect[0] = hebiJointAngVelocities.at(0);
            jointVelocityVect[1] = hebiJointAngVelocities.at(1);
            jointVelocityVect[2] = hebiJointAngVelocities.at(2);
            // compute fwd kinematics from hebi joints
            robot_state->setJointGroupPositions(joint_model_group, hebiJointAngles);
            end_effector_state = robot_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
            // ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
            // ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

            robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
            // robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().at(0)), reference_point_position, wristJacobian); 
            // ROS_INFO_STREAM("Jacobian Reference (front): " << joint_model_group->getLinkModelNames().at(0) << "\n");
            // ROS_INFO_STREAM("Jacobian Reference (back): " << joint_model_group->getLinkModelNames().back() << "\n");
            // ROS_INFO_STREAM("wristJacobian: \n" << wristJacobian << "\n");

            // end effector translation is x,y,z state
            // error is desired - current
            taskSpaceVelocityVect = wristJacobian*jointVelocityVect;

            
            // TODO: compute distance between desired and current 
            // use robot_state.getFrameTransform (returns Affine3d or Isometry3d) between model frame and link id
            xErrP = target_push_pose2.position.x - end_effector_state.translation().x();
            yErrP = target_push_pose2.position.y - end_effector_state.translation().y();
            zErrP = target_push_pose2.position.z - end_effector_state.translation().z();
            xErrD = desVelAngX - taskSpaceVelocityVect[0];
            yErrD = desVelAngY - taskSpaceVelocityVect[1];
            zErrD = desVelAngZ - taskSpaceVelocityVect[2];

            // std::cout << "xErrP:  " << xErrP << std::endl;
            // std::cout << "yErrP:  " << yErrP << std::endl;
            // std::cout << "zErrP:  " << zErrP << std::endl;

            // set effort from gains and errors
            endLinkEffort[0] = impGainXp*xErrP + impGainXd*xErrD; // x directional force
            endLinkEffort[1] = impGainYp*yErrP + impGainYd*yErrD; // y directional force
            endLinkEffort[2] = impGainZp*zErrP + impGainZd*zErrD; // z directional force
            endLinkEffort[3] = 0;
            endLinkEffort[4] = 0;
            endLinkEffort[5] = 0;

            // std::cout << "Cartesion Error" << endLinkEffort << std::endl;

            // perform jacobian transform from end effector force to joint torques
            // current_hebi_joints[0] = hebiJointAngles.at(0);
            // current_hebi_joints[1] = hebiJointAngles.at(1);
            // current_hebi_joints[2] = hebiJointAngles.at(2);



            // desired_torques = wristJacobian.transpose()*endLinkEffort;
            desired_torques.x = (wristJacobian.transpose()*endLinkEffort)[0];
            // desired_torques.x = 3.05;

            // std::cout << "wrist jacobian: " << wristJacobian.transpose() << std::endl;
            // std::cout << "end link effort: " << endLinkEffort << std::endl;
            // std::cout << "base torque: " << desired_torques.x << std::endl;

            // if (abs(desired_torques.x) > baseMaxTorque)
            // {
            //     desired_torques.x = baseMaxTorque*(desired_torques.x / abs(desired_torques.x));
            //     // ROS_INFO_STREAM("Desired Base Torque Direction: " << (desired_torques.x / abs(desired_torques.x)) << '\n');
            // }
            desired_torques.y = (wristJacobian.transpose()*endLinkEffort)[1];
            // desired_torques.y = -0.29;
            // if (abs(desired_torques.y) > elbowMaxTorque)
            // {
            //     desired_torques.y = elbowMaxTorque*(desired_torques.y / abs(desired_torques.y));
            // }
            desired_torques.z = (wristJacobian.transpose()*endLinkEffort)[2];
            // desired_torques.z = 0.15;
            // if (abs(desired_torques.z) > wristMaxTorque)
            // {
            //     desired_torques.z = wristMaxTorque*(desired_torques.z / abs(desired_torques.z));
            // }

            // ROS_INFO_STREAM("Desired Torques : \n" << desired_torques << "\n");

            desired_efforts_pub.publish(desired_torques);

            // TODO: read off desired torques, clip torques that are above certain joint thresholds
            // TODO: publish desired torques for HEBI node to pick up
            currTime = ros::Time::now();
            durVar = currTime - beginTime;
            ROS_INFO_STREAM("Impedance Time Interval: " << (float) durVar.toSec() << "\n");


        } 
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
