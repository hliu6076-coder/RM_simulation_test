#include "odom_depth_tf.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>

OdomDepthTransform::OdomDepthTransform()
    : rclcpp::Node("odom_depth_transform_node")
        {
            sub_depth_filtered = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/filtered_depth_frame", 
                rclcpp::SensorDataQoS(), 
                std::bind(&OdomDepthTransform::depth_filter_feedback, this, std::placeholders::_1));
        }
void OdomDepthTransform::depth_filter_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    // 接收rgb_depth.cpp处理后的点云数据，计算位姿变化
    pcl::PointCloud<pcl::PointXYZ> current_cloud_data;
    pcl::PointCloud<pcl::PointXYZ> previous_cloud_data;
    pcl::fromROSMsg(*msg, current_cloud_data);
    if (!first_time_stemp_initialized_){
        previous_cloud_data = current_cloud_data;
        first_time_stemp_initialized_ = true;
    } 
    previous_cloud_data = current_cloud_data;
    rclcpp::Time current_stamp(msg->header.stamp, RCL_ROS_TIME);
    const double dt = (current_stamp - previous_stamp).seconds();

    // 处理异常点云时间间隔
    if (dt < 0.0 || dt > 0.3) {
        RCLCPP_WARN(get_logger(),"异常点云时间间隔: %.3f s",dt);
        previous_stamp = current_stamp;
        return;
    }
    // 使用ICP算法进行点云配准
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(current_cloud_data.makeShared());
    icp.setInputTarget(previous_cloud_data.makeShared());
    icp.setMaximumIterations(30);
    icp.setMaxCorrespondenceDistance(0.30);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    pcl::PointCloud<PointT> aligned_cloud;
    icp.align(aligned_cloud);

    icp.align(aligned_cloud);
    if (!icp.hasConverged()) {RCLCPP_WARN(get_logger(),"ICP没有收敛, dt=%.3f",dt);
        return;
    }
    const double fitness = icp.getFitnessScore();
    const Eigen::Matrix4f transform = icp.getFinalTransformation();

    RCLCPP_INFO(get_logger(),"ICP收敛: dt=%.3f score=%.6f",dt,fitness);
    std::cout << transform << std::endl;

    // 结束时间帧和点云更新
    previous_stamp = current_stamp;
    previous_cloud_data = current_cloud_data;

}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDepthTransform>());
  rclcpp::shutdown();
  return 0;
}