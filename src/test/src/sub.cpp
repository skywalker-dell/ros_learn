#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/topic.h"
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
    while(ros::ok())
    {
        auto now = ros::Time::now();
        auto msg_ptr = ros::topic::waitForMessage<std_msgs::String>("chatter", ros::Duration(5.0));
        std::cout << (ros::Time::now() - now).toSec() << std::endl;
    }
    return 0;
}