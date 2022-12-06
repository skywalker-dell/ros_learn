#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/time.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf1_demo_node");
  ros::NodeHandle nh;
  ros::Rate r(10.0);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf2_ros::
  while (ros::ok())
  {
    auto start = ros::Time::now();
    if (listener.canTransform("/frame1", "/frame2", ros::Time(0)))
    {
      ROS_INFO("delta_time: %lf", (ros::Time::now() - start).toSec());
      listener.lookupTransform("/frame1", "/frame2", ros::Time(0), transform);
    }
    r.sleep();
  }
  return 0;
}