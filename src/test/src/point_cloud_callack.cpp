#include <ros/ros.h>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"

void cb(const sensor_msgs::PointCloud2ConstPtr &msg_ptr)
{
    auto now = ros::Time::now();
    ROS_INFO("delta time: %lf", (now - msg_ptr->header.stamp).toSec());
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "point_cloud_calback");
    ros::NodeHandle nh;
    auto sub = nh.subscribe("/camera/depth/color/points", 1, cb);
    ros::spin();
    return 0;
}