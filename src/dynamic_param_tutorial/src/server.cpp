#include <ros/init.h>
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_param_tutorial/demoConfig.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "server_node");
  dynamic_reconfigure::Server<dynamic_param_tutorial::demoConfig> server;
  dynamic_reconfigure::Server<dynamic_param_tutorial::demoConfig>::CallbackType f;



}