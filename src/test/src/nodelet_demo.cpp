#include <test/nodelet_demo.h>
#include <pluginlib/class_list_macros.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(example_pkg::MyNodeClass, nodelet::Nodelet)

namespace example_pkg
{
void MyNodeClass::onInit()
{
   ROS_WARN("Initializing nodelet...");
}
}  // namespace example_pkg
