#include "terrain_action/interact_points_visualization.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "visualization_msgs/Marker.h"

void terrain_action::InteractPointsVisualization::init()
{
  ros::NodeHandle nh;
  low_points_.ns = "interact_low_points";
  high_points_.ns = "interact_high_points";
  low_points_.lifetime = high_points_.lifetime = ros::Duration();
  low_points_.action = high_points_.action = visualization_msgs::Marker::ADD;
  low_points_.type = high_points_.type = visualization_msgs::Marker::POINTS;
  low_points_.scale.x = high_points_.scale.x = 0.05;
  low_points_.scale.y = high_points_.scale.y = 0.05;
  low_points_.scale.z = high_points_.scale.z = 0.05;

  low_points_.color.r = 1.0f;
  low_points_.color.a = 1.0;
  high_points_.color.g = 1.0f;
  high_points_.color.a = 1.0;

  low_points_pub_ = nh.advertise<visualization_msgs::Marker>("interact_low_points", 1);
  high_points_pub_ = nh.advertise<visualization_msgs::Marker>("interact_high_points", 1);

  // high_points_.scale.z = 0.2; // http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
}

void terrain_action::InteractPointsVisualization::visualize(const points_tpye& low_points,
                                                            const points_tpye& high_points, const std::string& frame_id,
                                                            const ros::Time& stamp)
{
  low_points_.points.clear();
  high_points_.points.clear();
  low_points_.header.frame_id = high_points_.header.frame_id = frame_id;
  low_points_.header.stamp = high_points_.header.stamp = stamp;
  if (!high_points.empty())
  {
    for (auto& point : high_points)
    {
      high_points_.points.emplace_back(point);
    }
    high_points_pub_.publish(high_points_);
  }
  if (!low_points.empty())
  {
    for (auto& point : low_points)
    {
      low_points_.points.emplace_back(point);
    }
    low_points_pub_.publish(low_points_);
  }
}