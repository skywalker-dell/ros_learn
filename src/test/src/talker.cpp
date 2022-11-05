#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;
    auto pub = nh.advertise<std_msgs::String>("chatter", 1, true);
    ros::Rate r(100.0);
    std_msgs::String msg;
    msg.data = "hello";
    while (ros::ok())
    {
        pub.publish(msg);
        r.sleep();
    }
    return 0;
}