//
// Created by arx on 22-10-31.
//
#include <string>
#include "terrain_action/terrain_action.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"


ActionPlanner::ActionPlanner() {
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

    odom_sub_ = private_nh_.subscribe<nav_msgs::Odometry>(odom_topic_name_, 1, &ActionPlanner::odomCb, this);
    octomap_sub_ = private_nh_.subscribe<octomap_msgs::Octomap>(octomap_topic_name_, 1, &ActionPlanner::octomapCb, this);
    elevation_map_sub_ = private_nh_.subscribe<grid_map_msgs::GridMap>(elevation_map_topic_name_, 1, &ActionPlanner::elevationMapCb, this);

    robot_move_region_visualization_.init();

    run();
}

void ActionPlanner::run()
{
    ros::Rate r(20);
    while(ros::ok())
    {
        if (!if_msg_updated()) continue;
        act_based_on_elevation_map();
        r.sleep();
    }
}

bool ActionPlanner::if_msg_updated() const
{
    auto now = ros::Time::now();
    auto delta_odom_time = (now - odom_msg_.header.stamp).toSec();
    auto delta_elevation_map_time = (now - elevation_map_ptr_->info.header.stamp).toSec();
    if (delta_odom_time < 0.5 && delta_elevation_map_time < 0.5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 统一在robot_base_footprint 坐标系下做碰撞检测
void ActionPlanner::act_based_on_elevation_map() const
{
    auto vel = odom_msg_.twist.twist.linear;
    double car_speed = sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
    if (car_speed < 0.1) return;
    robot_move_region_visualization_.visualize(robot_width_,  car_speed * 1.0);

    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*elevation_map_ptr_, map);
    double resolution = elevation_map_ptr_->info.resolution;

    double x_start = 0.0;
    double x_end =  vel.x * 1.0;
    double y_start = -0.5 * robot_width_;
    double y_end = 0.5 * robot_width_;
    Eigen::Vector2d up_left(x_start, y_end);
    Eigen::Vector2d up_right(x_end, y_end);
    Eigen::Vector2d down_right(x_end, y_start);
    Eigen::Vector2d down_left(x_start, y_start);

    int x_num = std::floor((x_end - x_start) / resolution);
    int y_num = std::floor((y_end - y_start) / resolution);

    std::vector<Eigen::Vector2d> robot_move_region_points;
    for (int i = 0; i <= x_num; ++i)
    {
        for (int j = 0; j <= y_num; ++j)
        {
            robot_move_region_points.emplace_back(Eigen::Vector2d(x_start+i*resolution, y_start+i*resolution));
        }
    }

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
//    for (auto &point: robot_move_region_points)
//    {
//        geometry_msgs::PointStamped point_base_footprint;
//        point_base_footprint.header.stamp = elevation_map_ptr_->info.header.stamp;
//        point_base_footprint.header.frame_id = robot_base_footprint_frame_id_;
//        point_base_footprint.point.x = point.x();
//        point_base_footprint.point.y = point.y();
//        point_base_footprint.point.z = 0.0;
//        geometry_msgs::PointStamped point_odom;
//        point_odom = buffer.transform(point_base_footprint, robot_base_footprint_frame_id_);
//        point.x() = point_odom.point.x;
//        point.y() = point_odom.point.y;
//    }

    std::vector<grid_map::Position> high_points;
    std::vector<grid_map::Position> low_points;
    for (grid_map::GridMapIterator iter(map); !iter.isPastEnd(); ++iter)
    {
        grid_map::Position position;
        map.getPosition(*iter, position);
        double height = map.at("elevation", *iter);
        if (height > robot_down_height_)
        {
            high_points.emplace_back(position);
        }
        else if (height > min_jump_height_ && height < max_step_height_)
        {
            low_points.emplace_back(position);
        }
    }
    const int min_points_num = 5;
    if (high_points.size() > low_points.size() && high_points.size() > min_points_num) // 蹲下
    {
        ;
    }
    else if (low_points.size() > high_points.size() && low_points.size() > min_points_num) // 跳跃
    {
        ;
    }
}

void ActionPlanner::odomCb(const nav_msgs::Odometry::ConstPtr &odom_ptr)
{
    odom_msg_ = *odom_ptr;
    robot_move_region_visualization_.visualize(robot_width_,   1.0);


}

void ActionPlanner::elevationMapCb(const grid_map_msgs::GridMap::ConstPtr &grid_map_ptr)
{

    elevation_map_ptr_ = grid_map_ptr;

    // 可视化机器人未来1s的运动空间

    // 计算当前机器人未来通行空间的点
    double resolution = grid_map_ptr->info.resolution;

    double x_start = this->odom_msg_.pose.pose.position.x;
    double x_end = x_start + this->odom_msg_.twist.twist.linear.x * 1.0;
    double y_start = this->odom_msg_.pose.pose.position.y;
    double y_end = y_start + this->odom_msg_.twist.twist.linear.y * 1.0;


}

void ActionPlanner::octomapCb(const octomap_msgs::Octomap::ConstPtr &octomap_msg_ptr) {
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
}



