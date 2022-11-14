#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void raw_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::cout << msg->data.size()  << std::endl;
}

void downsampled_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::cout << msg->data.size()  << std::endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "points_cnt_node");
    ros::NodeHandle nh;
    auto raw_points_sub = nh.subscribe("/camera/depth/points", 1, raw_points_cb);
    auto downsampled_points_sub = nh.subscribe("/camera/depth/points_downsampled", 1, downsampled_points_cb);
    ros::spin();
    return 0;
}