#include <ros/init.h>
#include <ros/node_handle.h>
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "help_node");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}