#include "grid_map_msgs/GridMap.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "ros/time.h"
#include"std_msgs/String.h"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_demo_node");
    ros::NodeHandle nh;
    auto pub_latch_true = nh.advertise<std_msgs::String>("chatter1", 1, true);
    auto pub_latch_false = nh.advertise<std_msgs::String>("chatter2", 1, false);
    std_msgs::String msg;
    msg.data = "ss";
    pub_latch_true.publish(msg);
    pub_latch_false.publish(msg);
    ros::Rate r(1.0);
    while(ros::ok())
    {
        r.sleep();
    }
    return 0;
}