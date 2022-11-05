#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


void cb(const std_msgs::StringConstPtr &ptr)
{
    ROS_WARN("aaaa");
    ros::Duration(1).sleep();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_demo_node");
    ros::NodeHandle nh;
    auto sub = nh.subscribe<std_msgs::String>("chatter", 1, cb);

    ros::Rate r(100);    
    while(ros::ok())
    {   
        ros::spinOnce();
        std::cout << "sss" << std::endl;
        r.sleep();
    }
    return 0;
}