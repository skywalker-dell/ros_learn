#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "terrain_action/detect.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf/transform_listener.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "detact_node");
  ros::NodeHandle nh;
  tf::TransformListener listener;
  terrain_action::Detection detection(listener);
  ros::spin();
  return 0;
}