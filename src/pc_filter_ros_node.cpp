#include "pointcloud_filter_ros/pc_filter_ros.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv ,"pc_filter_ros_node");
    ros::NodeHandle nh("~");
    pc_filter_ros::PCFilterROS pointcloud_filter(nh);

    ros::spin();
    return 0;
}