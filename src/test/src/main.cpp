#include "grid_map_msgs/GridMap.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "ros/time.h"


void cb(const grid_map_msgs::GridMapConstPtr &ptr)
{
    auto now = ros::Time::now();
    ROS_INFO("%lf", (now - ptr->info.header.stamp).toSec());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_demo_node");
    ros::NodeHandle nh;
    auto sub = nh.subscribe<grid_map_msgs::GridMap>("/elevation_mapping/elevation_map", 1, cb);
    ros::spin();
    return 0;
}