//
// Created by arx on 22-11-1.
//
#include "terrain_action/robot_move_region_visualization.h"

namespace terrain_action
{
static void setupColor(const int color_value, std_msgs::ColorRGBA& color)
{
  Eigen::Vector3f color_vector;
  color_vector(0) = (color_value >> 16) & 0x0000ff;
  color_vector(1) = (color_value >> 8) & 0x0000ff;
  color_vector(2) = color_value & 0x0000ff;
  color.r = color_vector(0);
  color.g = color_vector(1);
  color.b = color_vector(2);
  color.a = 1.0;
}

void RobotMoveRegionVisualization::init()
{
  ros::NodeHandle nh("~");

  marker_.ns = "robot_shape_region";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_.scale.x = 0.003;
  marker_.points.resize(n_vertices_);

  std_msgs::ColorRGBA color_;
  setupColor(color_value_, color_);
  marker_.colors.resize(n_vertices_, color_);
  pub_ = nh.advertise<visualization_msgs::Marker>("robot_shape_region", 1);
}

void RobotMoveRegionVisualization::visualize(double robot_width, double move_length)
{
  marker_.header.frame_id = "base_footprint";
  marker_.header.stamp = ros::Time::now();
  marker_.points[0].x = 0;
  marker_.points[0].y = robot_width / 2.0;
  marker_.points[1].x = move_length;
  marker_.points[1].y = robot_width / 2.0;
  marker_.points[2].x = move_length;
  marker_.points[2].y = -1 * robot_width / 2.0;
  marker_.points[3].x = 0;
  marker_.points[3].y = -1 * robot_width / 2.0;
  marker_.points[4].x = 0;
  marker_.points[4].y = robot_width / 2.0;
  pub_.publish(marker_);
}

}  // namespace terrain_action
