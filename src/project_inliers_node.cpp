#include "pointcloud_filter_ros/project_inliers.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv ,"project_inliers_node");
    ros::NodeHandle nh("~");
    pc_filter_ros::PCFilterROS pointcloud_filter(nh);

    ros::spin();
    return 0;
}