//
// Created by arx on 22-10-31.
//
#include "terrain_action/terrain_action.h"

ActionPlanner::ActionPlanner() {
    private_nh_ = ros::NodeHandle("~");

    private_nh_.param("robot_max_height", robot_max_height_, 0.7);
    private_nh_.param("robot_min_height", robot_min_height_, 0.5);
    private_nh_.param("robot_width", robot_width_, 0.4);
//    private_nh_.param("odom_topic_name", odom_topic_name_, 1);
//    private_nh_.param("octomap_topic_name", octomap_topic_name_, "asd");

    odom_sub_ = private_nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &ActionPlanner::odom_cb, this);
    map_sub_ = private_nh_.subscribe<octomap_msgs::Octomap>("/octomap_full", 1, &ActionPlanner::octomap_cb, this);
}

void ActionPlanner::odom_cb(const nav_msgs::Odometry::ConstPtr &odom_ptr) {
    odom_msg_ = *odom_ptr;
}

void ActionPlanner::octomap_cb(const octomap_msgs::Octomap::ConstPtr &octomap_msg_ptr) {
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



