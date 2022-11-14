#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "grid_map_msgs/GridMap.h"
#include "ros/time.h"

void cb(const grid_map_msgs::GridMapConstPtr &map)
{
    auto now = ros::Time::now().toSec();
    auto end = map->info.header.stamp.toSec();
    std::cerr << "delta_time: " << (now - end) << std::endl;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    auto sub = nh.subscribe("/elevation_mapping/elevation_map", 1, cb);
    ros::spin();
    return 0;
}