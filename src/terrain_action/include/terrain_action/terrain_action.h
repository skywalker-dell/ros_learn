//
// Created by arx on 22-10-31.
//

#ifndef LEARN_TERRAIN_ACTION_NODE_H
#define LEARN_TERRAIN_ACTION_NODE_H

#include <memory>
#include <tf2_ros/buffer.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "grid_map_msgs/GridMap.h"
#include "robot_move_region_visualization.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "thread_safe_data_wrapper.h"

// 导航在构建栅格地图只需要起跳高度到蹲下的最高高度即可。

// 动作响应只需要2.5D地图即可，高度为0-机器人的最大高度
// 因为

// 假设点云滤波已经完成
// 仅使用线速度进行分析，往前运动1s中的时间，判断是否与障碍物相交
// 判断相交方式：
namespace terrain_action
{
class ActionPlanner
{
public:
  explicit ActionPlanner(tf2_ros::Buffer& tf_buffer);

private:
  void run();
  bool ifMsgUpdated() const;
  void actBasedOnElevationMap() const;
  void odomCb(const nav_msgs::Odometry::ConstPtr& odom_ptr);
  void elevationMapCb(const grid_map_msgs::GridMap::ConstPtr& grid_map_ptr);

private:
  ros::Subscriber odom_sub_;           // 订阅里程计信息
  ros::Subscriber elevation_map_sub_;  // 订阅高程图
  ros::ServiceClient jump_cli_;        // 发送跳跃请求

  nav_msgs::Odometry odom_msg_;
  grid_map::GridMap elevation_map_;

  ros::NodeHandle private_nh_;
  tf2_ros::Buffer& tf_buffer_;
  RobotMoveRegionVisualization robot_move_region_visualization_;

private:
  std::string odom_topic_name_;
  std::string octomap_topic_name_;
  std::string elevation_map_topic_name_;

  std::string robot_base_footprint_frame_id_;
  std::string elevation_map_frame_id_;

  double robot_up_height_;    // 站起的高度
  double robot_down_height_;  // 蹲下的高度
  double min_jump_height_;    // 高于该值触发跳跃
  double max_step_height_;    // 最大能够跳跃的台阶高度
  double robot_width_;        // 机器人宽度
  double robot_length_;
};

}  // namespace terrain_action

#endif  // LEARN_TERRAIN_ACTION_NODE_H
