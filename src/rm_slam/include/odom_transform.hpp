#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

class OdomTransform : public rclcpp::Node
{
    public:
        OdomTransform(); 

    private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

        bool timestamp_initialized_{false};
        bool bias_calibrated_{false};
        int calibration_count_{0};
        int calibration_samples_{200};
        bool planar_mode_{true};
        bool publish_tf_{true};
        bool enable_zupt_{true};
        int stationary_count_{0};
        int stationary_samples_{20};
        double gravity_{9.80665};
        double calibration_accel_tolerance_{0.8};
        double calibration_gyro_tolerance_{0.15};
        double acceleration_deadband_{0.05};
        double stationary_accel_threshold_{0.15};
        double stationary_orientation_rate_threshold_{0.02};
        std::string odom_frame_{"odom"};
        std::string base_frame_{"base_link"};

        rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
        tf2::Vector3 position_{0.0, 0.0, 0.0};
        tf2::Vector3 velocity_{0.0, 0.0, 0.0};
        tf2::Quaternion previous_orientation_{0.0, 0.0, 0.0, 1.0};
        tf2::Vector3 acceleration_bias_body_{0.0, 0.0, 0.0};
        tf2::Vector3 angular_velocity_bias_body_{0.0, 0.0, 0.0};
        tf2::Vector3 acceleration_bias_sum_{0.0, 0.0, 0.0};
        tf2::Vector3 angular_velocity_bias_sum_{0.0, 0.0, 0.0};

        void Imu_Process(const sensor_msgs::msg::Imu::SharedPtr msg);
        bool UpdateBiasCalibration(
            const tf2::Vector3 & acceleration_body,
            const tf2::Vector3 & angular_velocity_body,
            const tf2::Quaternion & orientation);
        static void ApplyDeadband(tf2::Vector3 & value, double threshold);
        void PublishOdometry(
            const sensor_msgs::msg::Imu & imu,
            const tf2::Quaternion & orientation,
            const tf2::Vector3 & angular_velocity);
};
