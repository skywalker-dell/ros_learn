//
// Created by arx on 22-10-30.
//

#include "terrain_action/terrain_action.h"


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "terrain_action_node");
    ActionPlanner action_planner;
    ros::spin();
    return 0;
}

