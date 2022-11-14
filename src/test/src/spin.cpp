#include <ostream>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

void cb(const std_msgs::StringConstPtr &msg_ptr)
{
    // sleep(2);
    std::cout << msg_ptr->data << std::endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "spin_demo_node");
    ros::NodeHandle nh;
    auto sub = nh.subscribe("chatter", 2, cb);
    ros::Rate r(5.0);
    int cnt = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        // std::cout << cnt++ << std::endl;
        r.sleep();
    }

    return 0;

}