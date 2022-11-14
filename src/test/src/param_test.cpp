#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "param_test_node");
    ros::NodeHandle nh;
    int param = 0;
    nh.param("param", param, -1);
    ROS_ERROR("param : %d", param);
    return 0;
}