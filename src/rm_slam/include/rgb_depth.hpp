#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class RgbDepth : public rclcpp::Node
{
    public:
        RgbDepth();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth_raw;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_info;
        void depth_info_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
};