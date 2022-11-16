#include "ros/ros.h"
#include "std_msgs/String.h"



void cb(const std_msgs::String::ConstPtr& ptr, const std::string& prefix)
{
  std::cout << prefix + "sss" << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sub_mutiple_param_demo_node");
  ros::NodeHandle nh;
  std::string prefix = "hello";
  auto sub = nh.subscribe<std_msgs::String>("chatter", 1, boost::bind(&cb, _1, prefix));
  ros::spin();
  return 0;
}