#include "ros/node_handle.h"
#include "nodelet/nodelet.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
namespace terrain_action
{
using point_type = pcl::PointXYZ;

class PlaneDetection : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  bool readParams();
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  ros::Subscriber points_sub_;
  tf2_ros::Buffer tf_buffer_;
  ros::Publisher cropped_point_cloud_pub_;

  ros::Publisher downsampled_point_cloud_pub_;


private:
  std::string pointcloud_in_name_;
  std::string pointcloud_out_name_;
};
}  // namespace terrain_action
