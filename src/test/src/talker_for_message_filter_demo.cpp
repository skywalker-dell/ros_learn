#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include "ros/timer.h"
#include "ros/timer_options.h"
#include "nav_msgs/Path.h"

ros::Publisher pub1;
ros::Publisher pub2;

void timer1_cb(const ros::TimerEvent&)
{
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  pub1.publish(msg);
  ROS_INFO("pub1");
}

void timer2_cb(const ros::TimerEvent&)
{
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  pub2.publish(msg);
  ROS_INFO("pub2");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "talker_node_for_message_filter_demo");
  ros::NodeHandle nh;
  pub1 = nh.advertise<nav_msgs::Path>("chatter1", 1);
  pub2 = nh.advertise<nav_msgs::Path>("chatter2", 1);

  auto timer1 = nh.createTimer(ros::Duration(0.1), timer1_cb, false, false);  // 10hz
  auto timer2 = nh.createTimer(ros::Duration(0.2), timer2_cb, false, false);  // 5hz

  timer1.start();
  timer2.start();
  ros::spin();
  return 0;
}