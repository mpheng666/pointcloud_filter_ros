#ifndef PC_FILTER_ROS_PC_FILTER_ROS_HPP_
#define PC_FILTER_ROS_PC_FILTER_ROS_HPP_

#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/model_outlier_removal.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace pc_filter_ros {

    struct ConeParam
    {
        double apex_x_m{0.0};
        double apex_y_m{0.0};
        double apex_z_m{0.0};
        double axis_direction_x{0.0};
        double axis_direction_y{0.0};
        double axis_direction_z{0.0};
        double opening_angle_rad{0.0};
    };

    class PCFilterROS {
    public:
        PCFilterROS(ros::NodeHandle& nh);

    private:
        ros::NodeHandle nh_p_;
        ros::NodeHandle nh_;

        ros::Subscriber cloud_sub_;
        ros::Publisher filtered_cloud_pub_;

        ConeParam cone_param_;

        void loadParam();

        void cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);
    };
} // namespace pc_filter_ros

#endif