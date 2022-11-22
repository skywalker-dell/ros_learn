#include <terrain_action/plane_detect.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl-1.10/pcl/impl/point_types.hpp>
#include <string>
#include "Eigen/src/Core/Matrix.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2/transform_storage.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Eigen>

namespace terrain_action
{
void PlaneDetection::onInit()
{
  readParams();
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh_ = getPrivateNodeHandle();
  points_sub_ = nh.subscribe(pointcloud_in_name_, 1, &PlaneDetection::pointsCallback, this);
  std::string cropped_point_cloud_name;
  cropped_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_out_name_, 1, false);
}

void PlaneDetection::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
  tf2_ros::TransformListener tf_listener_(tf_buffer_);

  // 首先转换坐标系
  sensor_msgs::PointCloud2Ptr transformed_pointcloud;
  try
  {
    pcl_ros::transformPointCloud("robot_basefootprint", *pointcloud, *transformed_pointcloud, tf_buffer_);
    pcl::PointCloud<point_type>::Ptr point_cloud;
    pcl::fromROSMsg(*pointcloud, *point_cloud);
    pcl::PointCloud<point_type>::Ptr filtered_point_cloud;

    // 降采样，同时去掉一些NaN值
    pcl::PCLPointCloud2::Ptr downsampled_point_cloud;
    pcl::toPCLPointCloud2(*point_cloud, *downsampled_point_cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(downsampled_point_cloud);
    sor.setLeafSize(0.05, 0.03, 0.03);
    sor.filter(*downsampled_point_cloud);
    pcl::fromPCLPointCloud2(*downsampled_point_cloud, *filtered_point_cloud);

    // 在z轴滤波， 去掉地面和高于一定值的点云
    pcl::PassThrough<point_type> pass_filter;
    pass_filter.setInputCloud(filtered_point_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.03, 0.25);
    pass_filter.filter(*filtered_point_cloud);

    if (filtered_point_cloud->size() < 10)
      return;

    // 计算法向量,采用openmp加速
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);  // 手动设置线程数，否则提示错误
    ne.setInputCloud(filtered_point_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.10);
    ne.compute(*cloud_normals);

    // 根据法向量筛选点
    pcl::SACSegmentationFromNormals<point_type, pcl::Normal> seg;  //分割对象
    seg.setOptimizeCoefficients(true);
    // https://pointclouds.org/documentation/group__sample__consensus.html
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(5000);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(filtered_point_cloud);
    seg.setInputNormals(cloud_normals);
    seg.setAxis(Eigen::Vector3f{ 0, 0, 1 });
    seg.setEpsAngle(31.4 / 180);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients;
    seg.segment(*inliers, *coefficients);

    // 获得在平面内的点
    pcl::ExtractIndices<point_type> extract;  //点提取对象
    extract.setInputCloud(filtered_point_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filtered_point_cloud);

    if (filtered_point_cloud->size() < 10)
      return;

    ROS_WARN("stair detected!");
    // 计算当前点到平面的垂直距离
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];
    double distance = std::abs(D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
    ROS_WARN("distance: %lf", distance);

    
  }
  catch (...)
  {
    ROS_WARN("transform error");
  }
}

bool PlaneDetection::readParams()
{
  ros::NodeHandle private_nh_ = getPrivateNodeHandle();
  private_nh_.param<std::string>("pointcloud_in_name_", pointcloud_in_name_, "pointcloud_in_name");
  private_nh_.param<std::string>("pointcloud_in_name_", pointcloud_in_name_, "pointcloud_in_name");
  return true;
}

}  // namespace terrain_action
PLUGINLIB_EXPORT_CLASS(terrain_action::PlaneDetection, nodelet::Nodelet)
