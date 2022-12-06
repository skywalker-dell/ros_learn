#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/time.h"
#include "tf2/convert.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf2/convert.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf2_demo_node");
  ros::NodeHandle nh;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  tf::StampedTransform t;

  broadcaster.sendTransform(static_transformStamped);



  tf::StampedTransform transform;
  ros::Rate r(20.0);
  while (ros::ok())
  {
    auto start = ros::Time::now();
    if (buffer.canTransform("frame1", "frame2", ros::Time(0), ros::Duration(3.0)))
    {
      ROS_INFO("delta_time: %lf", (ros::Time::now() - start).toSec());
      buffer.lookupTransform("frame1", "frame2", ros::Time(0));
    }
    // buffer.waitForTransform("/frame1", "/frame1", ros::Time(0), ros::Duration(3.0));
    r.sleep();
  }
}