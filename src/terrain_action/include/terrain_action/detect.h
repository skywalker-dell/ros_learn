#include "ros/node_handle.h"
#include "nodelet/nodelet.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
namespace terrain_action
{
using point_type = pcl::PointXYZ;

class Detection
{
public:
  explicit Detection(tf::TransformListener &listener);

private:
  bool readParams();
  void pointsCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

private:
  ros::Subscriber points_sub_;
  ros::Publisher cropped_point_cloud_pub_;
  ros::Publisher downsampled_pointcloud_pub_;
  ros::Publisher plane_pointcloud_pub_;
  tf::TransformListener &tf_listener_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

private:
  std::string pointcloud_in_name_;
  std::string pointcloud_out_name_;
};
}  // namespace terrain_action