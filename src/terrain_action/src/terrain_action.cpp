//
// Created by arx on 22-10-31.
//
#include <iterator>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/time.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <thread>
#include "terrain_action/terrain_action.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "geometry_msgs/PointStamped.h"
#include "chrono"


// 四边形的四个顶点分别为：(0, width/2) (length, width/2) (length, -width/2) (0, -width/2)
static bool judgeContain(double robot_width, double robot_move_length, Eigen::Vector2d &point)
{
    bool x_ok = std::abs(point.x() - 0.5 * robot_move_length) < std::abs(0.5 * robot_move_length);
    bool y_ok = std::abs(point.y() - 0.5 * robot_width) < (0.5 * robot_width);
    if (x_ok && y_ok)
        return true;
    else
        return false;
}

// 四边形的四个顶点分别为：(-robot_length/2, width/2) (length, width/2) (length, -width/2) (-robot_length/2, -width/2)
static bool judgeContain(double robot_width, double robot_length, double robot_move_length, Eigen::Vector2d &point)
{
    // 理论上若robot_base_footprint机器人的中心，下面的计算应该使用robot_length的一半
    // 但是为了冗余直接使用robot_length
    if (robot_move_length > 0) 
    {
        robot_length = -robot_length;
    }
    double x_mid = (robot_move_length + robot_length) / 2.0;
    double x_length = std::abs(robot_move_length - robot_length);
    bool x_ok = std::abs(point.x() - x_mid) < (0.5 * x_length);
    bool y_ok = std::abs(point.y() - 0.5 * robot_width) < (0.5 * robot_width);
    if (x_ok && y_ok)
        return true;
    else
        return false;
}


ActionPlanner::ActionPlanner(tf2_ros::Buffer &tf_buffer) : tf_buffer_(tf_buffer) 
{
    private_nh_ = ros::NodeHandle("~");

    private_nh_.param("robot_up_height", robot_up_height_, 0.7);
    private_nh_.param("robot_down_height", robot_down_height_, 0.5);
    private_nh_.param("min_jump_height", min_jump_height_, 0.05);
    private_nh_.param("max_step_height", max_step_height_, 0.2);
    private_nh_.param("robot_width", robot_width_, 0.4);
    private_nh_.param("robot_length", robot_length_, 0.4);

    private_nh_.getParam("odom_topic_name", odom_topic_name_);
    private_nh_.getParam("octomap_topic_name", octomap_topic_name_);
    private_nh_.getParam("elevation_map_topic_name", elevation_map_topic_name_);
    private_nh_.getParam("robot_base_footprint_frame_id", robot_base_footprint_frame_id_);
    private_nh_.getParam("elevation_map_frame_id", elevation_map_frame_id_);

    elevation_map_sub_ = private_nh_.subscribe<grid_map_msgs::GridMap>(elevation_map_topic_name_, 1, &ActionPlanner::elevationMapCb, this);
    odom_sub_ = private_nh_.subscribe<nav_msgs::Odometry>(odom_topic_name_, 1, &ActionPlanner::odomCb, this);

    robot_move_region_visualization_.init();

    run();
}

void ActionPlanner::run()
{
    ROS_INFO("start terrain_action_planner...");
    ros::Rate r(20);
    while(ros::ok())
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

    std::cout << delta_odom_time << ";" << delta_elevation_map_time << std::endl;
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
    auto start = ros::Time::now();
    auto vel = odom_msg_.twist.twist.linear;
    double car_speed = sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
    double robot_move_length = vel.x * 1.0; // 预测1s走过的距离
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
        if (height > robot_down_height_)
        {
            all_region_high_points.emplace_back(position);
        }
        else if (height > min_jump_height_ && height < max_step_height_)
        {
            all_region_low_points.emplace_back(position);
        }
    }
    auto end = ros::Time::now();
    ROS_INFO("%lf", (end-start).toSec());

    std::vector<grid_map::Position> robot_move_shape_region_high_points;
    std::vector<grid_map::Position> robot_move_shape_region_low_points;

    if (all_region_high_points.size() > 0)
    {
        for (auto &point :all_region_high_points)
        {
            geometry_msgs::PointStamped point_with_elevation_map_frame_id;
            point_with_elevation_map_frame_id.header.stamp = ros::Time().fromNSec(elevation_map_.getTimestamp());

            point_with_elevation_map_frame_id.header.frame_id = elevation_map_frame_id_;
            point_with_elevation_map_frame_id.point.x = point.x();
            point_with_elevation_map_frame_id.point.y = point.y();
            point_with_elevation_map_frame_id.point.z = 0.0;
            geometry_msgs::PointStamped point_with_robot_base_footprint_id;
            point_with_robot_base_footprint_id = tf_buffer_.transform(point_with_elevation_map_frame_id, robot_base_footprint_frame_id_);
            point.x() = point_with_robot_base_footprint_id.point.x;
            point.y() = point_with_robot_base_footprint_id.point.y;
        }
        for (auto &point :all_region_high_points)
        {
            if (judgeContain(robot_width_, robot_length_, robot_move_length, point))
            {
                robot_move_shape_region_high_points.emplace_back(point);
            }
        }
    }
    if (all_region_low_points.size() > 0)
    {
        for (auto &point : all_region_low_points)
        {
            geometry_msgs::PointStamped point_with_elevation_map_frame_id;
            point_with_elevation_map_frame_id.header.stamp = ros::Time().fromNSec(elevation_map_.getTimestamp());
            point_with_elevation_map_frame_id.header.frame_id = elevation_map_frame_id_;
            point_with_elevation_map_frame_id.point.x = point.x();
            point_with_elevation_map_frame_id.point.y = point.y();
            point_with_elevation_map_frame_id.point.z = 0.0;
            geometry_msgs::PointStamped point_with_robot_base_footprint_id;
            point_with_robot_base_footprint_id = tf_buffer_.transform(point_with_elevation_map_frame_id, robot_base_footprint_frame_id_);
            point.x() = point_with_robot_base_footprint_id.point.x;
            point.y() = point_with_robot_base_footprint_id.point.y;
        }
        for (auto &point : all_region_low_points)
        {
            if (judgeContain(robot_width_, robot_move_length, point))
            {
                robot_move_shape_region_low_points.emplace_back(point);
            }
        }
    }

    const int min_points_num = 5;
    if (all_region_high_points.size() > min_points_num) // 蹲下
    {
        ROS_INFO("down");
    } 
    else 
    {
        ROS_INFO("up");
    }
    if (all_region_low_points.size() > min_points_num) // 跳跃
    {
        std::sort(robot_move_shape_region_low_points.begin(), robot_move_shape_region_low_points.end(),
                  [&](auto a, auto b){return a.x() < b.x();});
        if (robot_move_shape_region_low_points[0].x() < 0.6)
        {
            ROS_INFO("jump!");
        }
        else
        {
            ROS_INFO("ready to jump");
        }
    }
}

void ActionPlanner::odomCb(const nav_msgs::Odometry::ConstPtr &odom_ptr)
{
    // auto now = ros::Time::now();
    // ROS_INFO("%lf", (now - odom_ptr->header.stamp).toSec());
    odom_msg_ = *odom_ptr;
    robot_move_region_visualization_.visualize(robot_width_, 1.0); // TODO: 待删除
}

void ActionPlanner::elevationMapCb(const grid_map_msgs::GridMap::ConstPtr &grid_map_ptr)
{
    // auto now = ros::Time::now();
    // ROS_INFO("%lf", (now - grid_map_ptr->info.header.stamp).toSec());
    grid_map::GridMapRosConverter::fromMessage(*grid_map_ptr, elevation_map_);
}




