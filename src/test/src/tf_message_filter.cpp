#include "ros/init.h"
#include "ros/node_handle.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

class PoseDrawer
{
public:
  PoseDrawer()
  {
    point_sub_.subscribe(nh_, "points_chatter", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub_, tf_listener_, "odom", 10);
    // tf_filter_->registerCallback(PoseDrawer::msgCallBack);
  }

private:
  void msgCallBack(const geometry_msgs::PointStampedConstPtr& msg)
  {
  }

private:
  ros::NodeHandle nh_;
  tf::MessageFilter<geometry_msgs::PointStamped>* tf_filter_;
  tf::TransformListener tf_listener_;
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf_message_filter_demo_node");
}