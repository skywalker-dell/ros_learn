//
// Created by arx on 22-10-20.
//

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class MetadataPub{
public:
    MetadataPub()
    {
        _pub = _n.advertise<nav_msgs::MapMetaData>("/map_metadata", 1);
        _sub = _n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &MetadataPub::cb, this);
    }

    void cb(const nav_msgs::OccupancyGrid &msg)
    {
        auto metadata = msg.header;
        _pub.publish(metadata);
    }

private:
    ros::NodeHandle _n;
    ros::Publisher _pub;
    ros::Subscriber _sub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform");
    MetadataPub helper;
    ros::spin();
    return 0;
}
