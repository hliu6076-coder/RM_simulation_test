#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class OdomDepthTransform : public rclcpp::Node
{
    public:
        OdomDepthTransform();

    private:
        bool reference_initialized_{false};
        rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
        pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
        Eigen::Isometry3d odom_base_pose_{Eigen::Isometry3d::Identity()};

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth_filtered_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        void depth_filter_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void save_reference(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
            const rclcpp::Time & stamp);
        void publish_odometry(
            const builtin_interfaces::msg::Time & stamp,
            double dt,
            const Eigen::Isometry3d & relative_base_pose,
            double fitness);
};
