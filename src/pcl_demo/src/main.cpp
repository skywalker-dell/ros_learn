
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h> //滤波相关
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

using PointTypeIRT = RsPointXYZIRT;
using PointTypeI = pcl::PointXYZI;

class pcl_sub {
private:
    ros::NodeHandle n;
    ros::Subscriber subCloud;
    ros::Publisher pubCloud;
    pcl::PointCloud<PointTypeIRT> ring_specific_pcl; // 取特定线的激光雷达数据
    pcl::PointCloud<PointTypeIRT> filtered_pcl; // 经过稀疏点云滤波算法最终的点云数据
    sensor_msgs::PointCloud2 filtered_msg; // 等待发送的已经处理完的点云数据

    chrono::time_point<chrono::system_clock> start, end;
    chrono::duration<double> elapsed_time;

public:
    pcl_sub()
        : n("~")
    {
        subCloud = n.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, &pcl_sub::getcloud, this); //接收点云数据，进入回调函数getcloud()
        pubCloud = n.advertise<sensor_msgs::PointCloud2>("/adjustd_cloud", 1000); //建立了一个发布器，主题是/adjusted_cloud，方便之后发布调整后的点云
    }

    //回调函数
    void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        start = chrono::system_clock::now(); // 记录一下处理时间

        ring_specific_pcl.clear(); // 有效点的点云需要在每帧开始的时候初始化
        pcl::PointCloud<PointTypeIRT>::Ptr raw_pcl_ptr(new pcl::PointCloud<PointTypeIRT>); //放在这里是因为，每次都需要重新初始化
        pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr); //把msg消息转化为点云
        // cout << "调用成功" << endl;
        for (int i = 0; i < raw_pcl_ptr->points.size(); i++) {
            if (raw_pcl_ptr->points[i].ring > 14 && raw_pcl_ptr->points[i].ring < 18)
            // if (raw_pcl_ptr->points[i].ring == 16 ) // 留下第1根线
            {
                ring_specific_pcl.push_back(raw_pcl_ptr->points[i]);
            }
        }
        // 稀疏点云滤波处理
        pcl::PointCloud<PointTypeIRT> filtered_pcl; //放在这里是因为，每次都需要重新初始化
        filtered_pcl.clear();
        pcl::RadiusOutlierRemoval<PointTypeIRT> sor; //创建滤波器对象
        sor.setInputCloud(ring_specific_pcl.makeShared()); //设置待滤波的点云
        sor.setRadiusSearch(0.05); // 设置搜索半径
        sor.setMinNeighborsInRadius(3); // 设置一个内点最少的邻居数目
        sor.setNegative(false);
        sor.filter(filtered_pcl); // 滤波结果存储到cloud_filtered

        pcl::toROSMsg(filtered_pcl, filtered_msg); //将点云转化为消息才能发布
        filtered_msg.header = laserCloudMsg->header;
        pubCloud.publish(filtered_msg); //发布调整之后的点云数据，主题为/adjustd_cloud

        end = chrono::system_clock::now();
        elapsed_time = end - start;
        cout << "process time: " << elapsed_time.count() * 1000 << "[msec]" << endl; // 记录单帧处理时间
    }

    ~pcl_sub() { }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "filtered");

    pcl_sub ps;

    ros::spin();
    return 0;
}