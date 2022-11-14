







#include "ros/duration.h"
#include <ostream>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "duration_demo_node");
    ros::start();
    auto last = ros::Time::now();
    ros::Duration(2.0).sleep();
    auto now = ros::Time::now();
    if ((last - now) > ros::Duration(1.0))
    {
        std::cout << "sss" << std::endl;
    }
    return 0;
}