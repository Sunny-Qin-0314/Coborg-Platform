#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "hebi_cpp_api/lookup.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "find_hebi_modules");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ROS_INFO("Hello World\n");
    }

    return 0;
}
