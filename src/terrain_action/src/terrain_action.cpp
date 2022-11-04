//
// Created by arx on 22-10-31.
//
#include <iterator>
#include <ros/init.h>
#include <ros/time.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <thread>
#include "terrain_action/terrain_action.h"
#include "geometry_msgs/PointStamped.h"
#include "chrono"


// 四边形的四个顶点分别为：(0, width/2) (length, width/2) (length, -width/2) (0, -width/2)
static bool judgeContain(double width, double length, Eigen::Vector2d &point)
{
    bool x_ok = std::abs(point.x() - 0.5 * length) < 0.5 * length;
    bool y_ok = std::abs(point.y() - 0.5 * width) < 0.5 * width;
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
            ros::Duration(0.5).sleep();
            continue;
        }
        actBasedOnElevationMap();
        r.sleep();
    }
}

bool ActionPlanner::ifMsgUpdated() const
{
    auto now = ros::Time::now();
    auto delta_odom_time = (now - odom_msg_.header.stamp).toSec();
    auto delta_elevation_map_time = (now - ros::Time().fromNSec(elevation_map_.getTimestamp())).toSec();
    if (delta_odom_time < 3.0 && delta_elevation_map_time < 3.0)
    {
        return true;
    }
    else
    {
        ROS_WARN("terrain_action_planer error: msg is outdated");
        return false;
    }
}

// 统一在robot_base_footprint 坐标系下做碰撞检测
void ActionPlanner::actBasedOnElevationMap() const
{
    auto vel = odom_msg_.twist.twist.linear;
    double car_speed = sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
    if (car_speed < 0.1) return;


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
    for (auto &point :all_region_low_points)
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

    std::vector<grid_map::Position> robot_move_shape_region_high_points;
    std::vector<grid_map::Position> robot_move_shape_region_low_points;
    for (auto &point :all_region_high_points)
    {
        if (judgeContain(robot_width_, (vel.x * 1.0), point))
        {
            robot_move_shape_region_high_points.emplace_back(point);
        }
    }
    for (auto &point :all_region_low_points)
    {
        if (judgeContain(robot_width_, (vel.x * 1.0), point))
        {
            robot_move_shape_region_low_points.emplace_back(point);
        }
    }

    const int min_points_num = 5;
    if (all_region_high_points.size() > min_points_num) // 蹲下
    {
        ROS_WARN("down");
    } else {
        ROS_WARN("up");
    }
    if (all_region_low_points.size() > min_points_num) // 跳跃
    {
        std::sort(robot_move_shape_region_low_points.begin(), robot_move_shape_region_low_points.end(),
                  [&](auto a, auto b){return a.x() < b.x();});
        if (robot_move_shape_region_low_points[0].x() < 0.6)
        {
            ROS_WARN("jump!");
        }
        else
        {
            ROS_WARN("ready to jump");
        }
    }
}

void ActionPlanner::odomCb(const nav_msgs::Odometry::ConstPtr &odom_ptr)
{
    odom_msg_ = *odom_ptr;
    robot_move_region_visualization_.visualize(robot_width_, 1.0); // TODO: 待删除
}

void ActionPlanner::elevationMapCb(const grid_map_msgs::GridMap::ConstPtr &grid_map_ptr)
{
    auto now = ros::Time::now();
    grid_map::GridMapRosConverter::fromMessage(*grid_map_ptr, elevation_map_);
}



//void ActionPlanner::octomapCb(const octomap_msgs::Octomap::ConstPtr &octomap_msg_ptr) {
//    ros::Time start = ros::Time::now();
//    octomap::AbstractOcTree* abstract_octotree_ptr = octomap_msgs::binaryMsgToMap(*octomap_msg_ptr);
//
//
//    auto octomap_octotree_ptr = std::dynamic_pointer_cast<octomap::OcTree>(abstract_octotree_ptr);
//
//    //------------------进行碰撞检测------------------------
//    // 构建octomap碰撞体
//    fcl::OcTree<float> *tree = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(tree_ptr));
//    auto tree_obj = std::shared_ptr<fcl::CollisionGeometry<float>>(tree);
//    fcl::CollisionObject<float> treeObj(tree_obj);
//
//    // 构建机器人碰撞体
//    auto slam_car_ptr = std::make_shared<fcl::Box<float>>(odom_msg_.twist.twist.linear.x * 1.0, robot_width_, robot_max_height_);
//    fcl::Vector3f translation(odom_msg_.pose.pose.position.x, odom_msg_.pose.pose.position.y, odom_msg_.pose.pose.position.z);
//    fcl::Quaternionf rotation(odom_msg_.pose.pose.orientation.w, odom_msg_.pose.pose.orientation.x, odom_msg_.pose.pose.orientation.y, odom_msg_.pose.pose.orientation.z);
//    fcl::CollisionObject<float> slamCarObject(slam_car_ptr);
//    slamCarObject.setTransform(rotation, translation);
//
//    // 碰撞检测
//    fcl::CollisionRequest<float> requestType(1, false, 1, false);
//    fcl::CollisionResult<float> collisionResult;
//    fcl::collide(&slamCarObject, &treeObj, requestType, collisionResult);
//    bool is_collision = collisionResult.isCollision();
//
//    if (is_collision)
//    {
//        ROS_WARN("may collision");
//    }
//    else
//    {
//        ROS_INFO("ok");
//    }
//
//    ros::Time end = ros::Time::now();
//    if ((end - start).toSec() > 1.0)
//        ROS_WARN("Time consume: %f", (end-start).toSec());
//}



