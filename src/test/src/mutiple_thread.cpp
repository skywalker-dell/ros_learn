#include <unistd.h>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "std_msgs/String.h"

void cb(const std_msgs::String::ConstPtr& msg_ptr)
{
  sleep(2);
  std::cout << msg_ptr->data << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mutiple_thread_demo");
  ros::NodeHandle nh;
  auto sub = nh.subscribe("chatter", 1, cb);
  ros::MultiThreadedSpinner spinner(1);
  std::cout << "sss" << std::endl;
//   ros::waitForShutdown();
  return 0;
}