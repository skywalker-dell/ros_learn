#include "grid_map_msgs/GridMap.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "ros/time.h"


void func()
{
    try {
        throw (1);
    } catch (...) {
        std::cout << "sss" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_demo_node");
    ros::NodeHandle nh;
    while(ros::ok())
    {
        func();
    }
}