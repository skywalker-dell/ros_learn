#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;
    auto pub = nh.advertise<std_msgs::String>("chatter", 1, true);
    ros::Rate r(10.0);
    int cnt = 0;
    std_msgs::String msg;
    while (ros::ok())
    {
        msg.data = std::to_string(cnt++);
        pub.publish(msg);
        r.sleep();
    }
    return 0;
}