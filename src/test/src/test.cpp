#include "grid_map_msgs/GridMapInfo.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "grid_map_msgs/GridMap.h"
#include "ros/time.h" 

void cb(const std_msgs::String::ConstPtr& ptr, const std::string& prefix)
{
  std::cout << prefix + "sss" << std::endl;
}

void cb1(const std_msgs::String::ConstPtr& ptr)
{
  std::cout << prefix + "sss" << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sub_mutiple_param_demo_node");
  ros::NodeHandle nh;
  std::string prefix = "hello";
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter", 1, boost::bind(cb, _1, prefix));
  nh.subscribe()
  ros::spin();
  return 0;
}