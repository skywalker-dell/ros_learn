#ifndef INTERACT_POINTS_VISUALIZATION_H
#define INTERACT_POINTS_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>
#include "ros/time.h"

using points_tpye = std::vector<geometry_msgs::Point>;

namespace terrain_action
{
class InteractPointsVisualization
{
public:
  void init();
  void visualize( const points_tpye& low_points, const points_tpye& high_points, const std::string& frame_id,
                 const ros::Time& stamp);

private:
  visualization_msgs::Marker low_points_;
  ros::Publisher high_points_pub_;
  visualization_msgs::Marker high_points_;
  ros::Publisher low_points_pub_;
};
}  // namespace terrain_action
#endif