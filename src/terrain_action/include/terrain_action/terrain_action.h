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
    void odom_cb(const nav_msgs::Odometry::ConstPtr &odom_ptr);
    void octomap_cb(const octomap_msgs::Octomap::ConstPtr &octomap_msg_ptr);

private:
    ros::Subscriber odom_sub_; // 订阅里程计信息
    ros::Subscriber map_sub_;  // 订阅当前的地图
    ros::ServiceClient jump_cli_; // 发送跳跃请求
    nav_msgs::Odometry odom_msg_;
    ros::NodeHandle private_nh_;
    std::string odom_topic_name_;
    std::string octomap_topic_name_;

private:
    double robot_max_height_;
    double robot_min_height_;
    double robot_width_;
};



#endif //LEARN_TERRAIN_ACTION_NODE_H
