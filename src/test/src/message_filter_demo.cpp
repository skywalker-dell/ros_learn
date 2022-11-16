#include "message_filters/synchronizer.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

ros::Publisher pub;

void cb(const nav_msgs::Path::ConstPtr& msg1, const nav_msgs::Path::ConstPtr& msg2)
{
  ROS_INFO("delta time: %lf", (msg1->header.stamp - msg2->header.stamp).toSec());
  std_msgs::String msg;
  msg.data = "ss";
  pub.publish(msg);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "message_filter_demo_node");
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::String>("fused_data", 1);
  message_filters::Subscriber<nav_msgs::Path> sub1(nh, "chatter1", 1);
  message_filters::Subscriber<nav_msgs::Path> sub2(nh, "chatter2", 1);

  using my_policy = message_filters::sync_policies::ExactTime<nav_msgs::Path, nav_msgs::Path>;
  message_filters::Synchronizer<my_policy> sync(my_policy(10), sub1, sub2);

  sync.registerCallback(boost::bind(cb, _1, _2));
  ros::spin();
  return 0;
}