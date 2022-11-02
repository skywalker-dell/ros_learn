//
// Created by arx on 22-10-31.
//

#ifndef LEARN_TERRAIN_ACTION_NODE_H
#define LEARN_TERRAIN_ACTION_NODE_H

#include <memory>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "fcl/fcl.h"
#include "grid_map_msgs/GridMap.h"
#include "robot_move_region_visualization.h"
#include "grid_map_ros/grid_map_ros.hpp"


// 导航在构建栅格地图只需要起跳高度到蹲下的最高高度即可。

// 动作响应只需要2.5D地图即可，高度为0-机器人的最大高度
// 因为

// 假设点云滤波已经完成
// 仅使用线速度进行分析，往前运动1s中的时间，判断是否与障碍物相交
// 判断相交方式：

class ActionPlanner {
public:
    ActionPlanner();

private:
    void run();
    bool if_msg_updated() const;
    void act_based_on_elevation_map() const;
    void odomCb(const nav_msgs::Odometry::ConstPtr &odom_ptr);
    void octomapCb(const octomap_msgs::Octomap::ConstPtr &octomap_msg_ptr);
    void elevationMapCb(const grid_map_msgs::GridMap::ConstPtr &grid_map_ptr);

private:
    ros::Subscriber odom_sub_; // 订阅里程计信息
    ros::Subscriber octomap_sub_;  // 订阅八叉树地图
    ros::Subscriber elevation_map_sub_; // 订阅高程图
    ros::ServiceClient jump_cli_; // 发送跳跃请求
    nav_msgs::Odometry odom_msg_;
    grid_map_msgs::GridMap::ConstPtr elevation_map_ptr_;
    RobotMoveRegionVisualization robot_move_region_visualization_;
    ros::NodeHandle private_nh_;


private:
    std::string odom_topic_name_;
    std::string octomap_topic_name_;
    std::string elevation_map_topic_name_;

    std::string robot_base_footprint_frame_id_;
    std::string elevation_map_frame_id_;

    double robot_up_height_; // 站起的高度
    double robot_down_height_; // 蹲下的高度
    double min_jump_height_; // 高于该值触发跳跃
    double max_step_height_; // 最大能够跳跃的台阶高度
    double robot_width_; // 机器人宽度
};



#endif //LEARN_TERRAIN_ACTION_NODE_H





