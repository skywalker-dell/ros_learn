//
// Created by arx on 22-10-30.
//

#include "terrain_action/terrain_action.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "terrain_action_node");
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  terrain_action::ActionPlanner action_planner(buffer);
  ros::spin();
  return 0;
}
