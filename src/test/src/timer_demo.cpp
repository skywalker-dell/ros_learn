#include <ostream>
#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"


void cb(const ros::TimerEvent&)
{
    std::cout << "1111" << std::endl;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "timer_demo_node");
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1), cb, false, false);
    timer.setPeriod(ros::Duration(5.0));
    timer.start();
    ros::spin();
    return 0;
}