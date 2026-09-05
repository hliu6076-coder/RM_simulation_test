#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OdomDepthTransform : public rclcpp::Node
{
    public:
        OdomDepthTransform();

    private:
        bool first_time_stemp_initialized_{false};
        rclcpp::Time previous_stamp{0, 0, RCL_ROS_TIME};
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth_filtered;
        void depth_filter_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};