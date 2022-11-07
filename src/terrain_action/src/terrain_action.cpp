//
// Created by arx on 22-10-31.
//
#include <cmath>
#include <iterator>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/time.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <thread>
#include "geometry_msgs/Point.h"
#include "grid_map_core/GridMap.hpp"
#include "ros/node_handle.h"
#include "terrain_action/terrain_action.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_storage.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace terrain_action
{
// 四边形的四个顶点分别为：(0, width/2) (length, width/2) (length, -width/2) (0, -width/2)
static bool judgeContain(double robot_width, double robot_move_length, geometry_msgs::Point& point)
{
  bool x_ok = std::abs(point.x - 0.5 * robot_move_length) < std::abs(0.5 * robot_move_length);
  bool y_ok = std::abs(point.y - 0.5 * robot_width) < (0.5 * robot_width);
  if (x_ok && y_ok)
    return true;
  else
    return false;
}

// 四边形的四个顶点分别为：(-robot_length/2, width/2) (length, width/2) (length, -width/2) (-robot_length/2, -width/2)
static bool judgeContain(double robot_width, double robot_length, double robot_move_length, geometry_msgs::Point& point)
{
  // 理论上若robot_base_footprint机器人的中心，下面的计算应该使用robot_length的一半
  // 但是为了冗余直接使用robot_length
  if (robot_move_length > 0)
  {
    robot_length = -robot_length;
  }
  double x_mid = (robot_move_length + robot_length) / 2.0;
  double x_length = std::abs(robot_move_length - robot_length);
  bool x_ok = std::abs(point.x - x_mid) < (0.5 * x_length);
  bool y_ok = std::abs(point.y - 0.5 * robot_width) < (0.5 * robot_width);
  if (x_ok && y_ok)
    return true;
  else
    return false;
}

ActionPlanner::ActionPlanner(tf2_ros::Buffer& tf_buffer) : tf_buffer_(tf_buffer)
{
  private_nh_ = ros::NodeHandle("~");

  private_nh_.param("robot_up_height", robot_up_height_, 0.7);
  private_nh_.param("robot_down_height", robot_down_height_, 0.5);
  private_nh_.param("min_jump_height", min_jump_height_, 0.05);
  private_nh_.param("max_step_height", max_step_height_, 0.2);
  private_nh_.param("robot_width", robot_width_, 0.4);
  private_nh_.param("robot_length", robot_length_, 0.4);

  private_nh_.param("odom_topic_name", odom_topic_name_, std::string("/odom"));
  private_nh_.param("octomap_topic_name", octomap_topic_name_, std::string("/octomap_binary"));
  private_nh_.param("elevation_map_topic_name", elevation_map_topic_name_,
                    std::string("/elevation_mapping/elevation_map"));
  private_nh_.param("robot_base_footprint_frame_id", robot_base_footprint_frame_id_, std::string("base_footprint"));
  private_nh_.param("elevation_map_frame_id", elevation_map_frame_id_, std::string("odom"));

  ros::NodeHandle nh;
  elevation_map_sub_ =
      nh.subscribe<grid_map_msgs::GridMap>(elevation_map_topic_name_, 1, &ActionPlanner::elevationMapCb, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_name_, 1, &ActionPlanner::odomCb, this);

  robot_move_region_visualization_.init();
  interact_points_visualization_.init();

  run();
}

void ActionPlanner::run()
{
  ROS_INFO("start terrain_action_planner...");
  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!ifMsgUpdated())
    {
      ros::Duration(0.1).sleep();
      continue;
    }
    actBasedOnElevationMap();
    r.sleep();
  }
}

bool ActionPlanner::ifMsgUpdated() const
{
  if (odom_sub_.getNumPublishers() < 1 || elevation_map_sub_.getNumPublishers() < 1)
    return false;

  auto now = ros::Time::now();
  auto delta_odom_time = (now - odom_msg_.header.stamp).toSec();
  auto delta_elevation_map_time = (now - ros::Time().fromNSec(elevation_map_.getTimestamp())).toSec();
  // std::cout << delta_odom_time << ";" << delta_elevation_map_time << std::endl;
  if (delta_odom_time < 5.0 && delta_elevation_map_time < 5.0)
  {
    return true;
  }
  else
  {
    ROS_INFO("terrain_action_planer error: msg is outdated");
    return false;
  }
}

// 统一在robot_base_footprint 坐标系下做碰撞检测
void ActionPlanner::actBasedOnElevationMap() const
{
  auto vel = odom_msg_.twist.twist.linear;
  double car_speed = sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
  double robot_move_length = vel.x * 1.0;  // 预测1s走过的距离
  // TODO: 待注释
  robot_move_length = 1.0;
  // 机器人静止，维持当前状态即可。TODO: 待取消注释
  // if (car_speed < 0.1) return;

  std::vector<grid_map::Position> all_region_high_points;
  std::vector<grid_map::Position> all_region_low_points;
  for (grid_map::GridMapIterator iter(elevation_map_); !iter.isPastEnd(); ++iter)
  {
    grid_map::Position position;
    elevation_map_.getPosition(*iter, position);
    double height = elevation_map_.at("elevation", *iter);
    if (!std::isnan(height) && height > robot_down_height_ && height < robot_up_height_)
    {
      all_region_high_points.emplace_back(position);
    }
    else if (!std::isnan(height) && height > min_jump_height_ && height < max_step_height_)
    {
      all_region_low_points.emplace_back(position);
    }
  }

  // 将elevation map的点由原始的不动的世界坐标系转换到随着机器人动而运动的robot_base_footprint
  std::vector<geometry_msgs::Point> robot_move_shape_region_high_points;
  std::vector<geometry_msgs::Point> robot_move_shape_region_low_points;

  ROS_INFO("delta_time: %lf", (ros::Time().fromNSec(elevation_map_.getTimestamp()) - odom_msg_.header.stamp).toSec());

  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        robot_base_footprint_frame_id_, odom_msg_.header.stamp, elevation_map_frame_id_,
        ros::Time().fromNSec(elevation_map_.getTimestamp()), elevation_map_frame_id_, ros::Duration(1.0));

    if (all_region_high_points.size() > 0)
    {
      for (auto& point : all_region_high_points)
      {
        geometry_msgs::Point point_elevation_map_frame_id;
        geometry_msgs::Point point_with_base_foot_print_frame_id;
        point_elevation_map_frame_id.x = point.x();
        point_elevation_map_frame_id.y = point.y();
        tf2::doTransform(point_elevation_map_frame_id, point_with_base_foot_print_frame_id, transform);
        if (judgeContain(robot_width_, robot_length_, robot_move_length, point_with_base_foot_print_frame_id))
        {
          robot_move_shape_region_high_points.emplace_back(point_with_base_foot_print_frame_id);
        }
      }
    }
    if (all_region_low_points.size() > 0)
    {
      for (auto& point : all_region_low_points)
      {
        geometry_msgs::Point point_elevation_map_frame_id;
        geometry_msgs::Point point_with_base_foot_print_frame_id;
        point_elevation_map_frame_id.x = point.x();
        point_elevation_map_frame_id.y = point.y();
        tf2::doTransform(point_elevation_map_frame_id, point_with_base_foot_print_frame_id, transform);
        if (judgeContain(robot_width_, robot_length_, robot_move_length, point_with_base_foot_print_frame_id))
        {
          robot_move_shape_region_low_points.emplace_back(point_with_base_foot_print_frame_id);
        }
      }
    }
  }

  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  interact_points_visualization_.visualize(robot_move_shape_region_low_points, robot_move_shape_region_high_points,
                                           robot_base_footprint_frame_id_, odom_msg_.header.stamp);

  const int min_points_num = 3;
  if (robot_move_shape_region_high_points.size() > min_points_num)  // 蹲下
  {
    ROS_INFO("down");
  }
  else
  {
    ROS_INFO("up");
  }
  if (robot_move_shape_region_low_points.size() > min_points_num)  // 跳跃
  {
    std::sort(robot_move_shape_region_low_points.begin(), robot_move_shape_region_low_points.end(),
              [&](auto a, auto b) { return a.x < b.x; });
    if (robot_move_shape_region_low_points[0].x < 0.6)
    {
      ROS_INFO("jump!");
    }
    else
    {
      ROS_INFO("ready to jump");
    }
  }
}

void ActionPlanner::odomCb(const nav_msgs::Odometry::ConstPtr& odom_ptr)
{
  odom_msg_ = *odom_ptr;
  robot_move_region_visualization_.visualize(robot_width_, 1.0);  // TODO: 待删除
}

void ActionPlanner::elevationMapCb(const grid_map_msgs::GridMap::ConstPtr& grid_map_ptr)
{
  grid_map::GridMapRosConverter::fromMessage(*grid_map_ptr, elevation_map_);
}

}  // namespace terrain_action
