#include "pointcloud_filter_ros/project_inliers.hpp"

namespace pc_filter_ros {
    PCFilterROS::PCFilterROS(ros::NodeHandle& nh)
        : nh_p_(nh)
        , cloud_sub_(nh_p_.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
          "input_cloud", 100, &PCFilterROS::cloudCb, this))
        , filtered_cloud_pub_(
          nh_p_.advertise<pcl::PCLPointCloud2>("filtered_cloud", 10))
    {
        loadParam();
    }

    void PCFilterROS::loadParam() {
        nh_p_.getParam("apex/x_m", cone_param_.apex_x_m);
        nh_p_.getParam("apex/y_m", cone_param_.apex_y_m);
        nh_p_.getParam("apex/z_m", cone_param_.apex_z_m);
        nh_p_.getParam("axis_direction/x", cone_param_.axis_direction_x);
        nh_p_.getParam("axis_direction/y", cone_param_.axis_direction_y);
        nh_p_.getParam("axis_direction/z", cone_param_.axis_direction_z);
        nh_p_.getParam("opening_angle_rad", cone_param_.opening_angle_rad);
    }

    void
    PCFilterROS::cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(
        new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(7);
        coefficients->values[0] = cone_param_.apex_x_m;
        coefficients->values[1] = cone_param_.apex_y_m;
        coefficients->values[2] = cone_param_.apex_z_m;
        coefficients->values[3] = cone_param_.axis_direction_x;
        coefficients->values[4] = cone_param_.axis_direction_y;
        coefficients->values[5] = cone_param_.axis_direction_z;
        coefficients->values[6] = cone_param_.opening_angle_rad;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_CONE);
        proj.setInputCloud(input);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        pcl::PCLPointCloud2 point_cloud2;
        pcl::toPCLPointCloud2(*cloud_projected, point_cloud2);

        filtered_cloud_pub_.publish(point_cloud2);
    }
} // namespace pc_filter_ros