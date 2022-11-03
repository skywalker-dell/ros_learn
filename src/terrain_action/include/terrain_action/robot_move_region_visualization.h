//
// Created by arx on 22-11-1.
//

#ifndef LEARN_ROBOT_SHAPE_H
#define LEARN_ROBOT_SHAPE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
class RobotMoveRegionVisualization{
public:
    void init();
    void visualize(double robot_width, double move_length);

private:
    visualization_msgs::Marker marker_;
    ros::Publisher pub_;
    int n_vertices_ = 5;
    int color_value_ = 16711680; // https://www.wolframalpha.com/input?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D255%2C+g%3D0%2C+b%3D0%7D

}
;

#endif //LEARN_ROBOT_SHAPE_H
