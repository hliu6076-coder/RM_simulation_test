#include "odom_transform.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <memory>

OdomTransform::OdomTransform()
    : rclcpp::Node("odom_transform_node")
{
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);
    planar_mode_ = declare_parameter<bool>("planar_mode", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    enable_zupt_ = declare_parameter<bool>("enable_zupt", true);
    stationary_samples_ = declare_parameter<int>("stationary_samples", 20);
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    calibration_accel_tolerance_ =
        declare_parameter<double>("calibration_accel_tolerance", 0.8);
    calibration_gyro_tolerance_ =
        declare_parameter<double>("calibration_gyro_tolerance", 0.15);
    acceleration_deadband_ =
        declare_parameter<double>("acceleration_deadband", 0.05);
    stationary_accel_threshold_ =
        declare_parameter<double>("stationary_accel_threshold", 0.15);
    stationary_orientation_rate_threshold_ =
        declare_parameter<double>("stationary_orientation_rate_threshold", 0.02);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    if (calibration_samples_ < 1) {
        calibration_samples_ = 1;
    }
    if (stationary_samples_ < 1) {
        stationary_samples_ = 1;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom_imu", 10);
    sub_imu_raw = create_subscription<sensor_msgs::msg::Imu>(
        "/odin1/imu/data_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&OdomTransform::Imu_Process, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "等待连续 %d 帧静止 IMU 数据进行零偏标定（planar_mode=%s）",
        calibration_samples_,
        planar_mode_ ? "true" : "false");
}

bool OdomTransform::UpdateBiasCalibration(
    const tf2::Vector3 & acceleration_body,
    const tf2::Vector3 & angular_velocity_body,
    const tf2::Quaternion & orientation)
{
    const tf2::Vector3 gravity_world(0.0, 0.0, gravity_);
    const tf2::Vector3 expected_gravity_body =
        tf2::quatRotate(orientation.inverse(), gravity_world);
    const tf2::Vector3 acceleration_residual =
        acceleration_body - expected_gravity_body;

    const bool stable =
        acceleration_residual.length() <= calibration_accel_tolerance_ &&
        angular_velocity_body.length() <= calibration_gyro_tolerance_;

    if (!stable) {
        if (calibration_count_ > 0) {
            RCLCPP_DEBUG(get_logger(), "机器人尚未稳定，重新开始 IMU 零偏采样");
        }
        calibration_count_ = 0;
        acceleration_bias_sum_.setValue(0.0, 0.0, 0.0);
        angular_velocity_bias_sum_.setValue(0.0, 0.0, 0.0);
        return false;
    }

    acceleration_bias_sum_ += acceleration_residual;
    angular_velocity_bias_sum_ += angular_velocity_body;
    ++calibration_count_;

    if (calibration_count_ < calibration_samples_) {
        return false;
    }

    acceleration_bias_body_ = acceleration_bias_sum_ / calibration_count_;
    angular_velocity_bias_body_ = angular_velocity_bias_sum_ / calibration_count_;
    bias_calibrated_ = true;

    RCLCPP_INFO(
        get_logger(),
        "IMU 零偏标定完成: accel=[%.5f %.5f %.5f] m/s^2, gyro=[%.5f %.5f %.5f] rad/s",
        acceleration_bias_body_.x(), acceleration_bias_body_.y(), acceleration_bias_body_.z(),
        angular_velocity_bias_body_.x(), angular_velocity_bias_body_.y(),
        angular_velocity_bias_body_.z());
    return true;
}

void OdomTransform::ApplyDeadband(tf2::Vector3 & value, const double threshold)
{
    if (std::abs(value.x()) < threshold) {
        value.setX(0.0);
    }
    if (std::abs(value.y()) < threshold) {
        value.setY(0.0);
    }
    if (std::abs(value.z()) < threshold) {
        value.setZ(0.0);
    }
}
void OdomTransform::Imu_Process(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Gazebo 传感器的 header.stamp 属于 ROS/仿真时间。
    const rclcpp::Time current_stamp(msg->header.stamp, RCL_ROS_TIME);
    tf2::Quaternion current_orientation(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    if (current_orientation.length2() < 1e-12) {
        return;
    }

    current_orientation.normalize();

    // 第一帧只初始化状态，不参与积分。
    if (!timestamp_initialized_) {
        previous_stamp_ = current_stamp;
        previous_orientation_ = current_orientation;
        timestamp_initialized_ = true;
        return;
    }

    // 直接使用相邻两帧消息时间戳的间隔。
    const double dt = (current_stamp - previous_stamp_).seconds();

    if (dt <= 0.0 || dt > 0.1) {
        RCLCPP_WARN(
            get_logger(),
            "异常 IMU 时间间隔: %.6f s",
            dt);
        previous_stamp_ = current_stamp;
        previous_orientation_ = current_orientation;
        return;
    }

    tf2::Vector3 acceleration_body(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    tf2::Vector3 angular_velocity_body(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);

    if (!bias_calibrated_) {
        UpdateBiasCalibration(
            acceleration_body,
            angular_velocity_body,
            current_orientation);
        previous_stamp_ = current_stamp;
        previous_orientation_ = current_orientation;
        return;
    }

    acceleration_body -= acceleration_bias_body_;
    angular_velocity_body -= angular_velocity_bias_body_;

    tf2::Vector3 acceleration_world =
        tf2::quatRotate(current_orientation, acceleration_body);
    acceleration_world -= tf2::Vector3(0.0, 0.0, gravity_);
    ApplyDeadband(acceleration_world, acceleration_deadband_);

    // IMU 单独无法区分静止和匀速直线运动，因此该 ZUPT 是可关闭的地面车假设。
    tf2::Quaternion delta_orientation =
        previous_orientation_.inverse() * current_orientation;
    delta_orientation.normalize();
    const double orientation_rate =
        std::abs(delta_orientation.getAngleShortestPath()) / dt;
    const bool stationary_candidate =
        acceleration_world.length() <= stationary_accel_threshold_ &&
        orientation_rate <= stationary_orientation_rate_threshold_;
    if (enable_zupt_ && stationary_candidate) {
        ++stationary_count_;
        // 确认窗口期间不让小噪声继续进入积分，但保留已有速度。
        acceleration_world.setValue(0.0, 0.0, 0.0);
        if (stationary_count_ >= stationary_samples_) {
            velocity_.setValue(0.0, 0.0, 0.0);
        }
    } else {
        stationary_count_ = 0;
    }

    // 地面机器人没有真实升降自由度。三维地图仍可包含不同高度的环境点。
    if (planar_mode_) {
        acceleration_world.setZ(0.0);
        velocity_.setZ(0.0);
        position_.setZ(0.0);
    }

    position_ += velocity_ * dt + acceleration_world * (0.5 * dt * dt);
    velocity_ += acceleration_world * dt;

    if (planar_mode_) {
        velocity_.setZ(0.0);
        position_.setZ(0.0);
    }

    PublishOdometry(*msg, current_orientation, angular_velocity_body);
    previous_stamp_ = current_stamp;
    previous_orientation_ = current_orientation;
}

void OdomTransform::PublishOdometry(
    const sensor_msgs::msg::Imu & imu,
    const tf2::Quaternion & orientation,
    const tf2::Vector3 & angular_velocity)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = imu.header.stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position_.x();
    odometry.pose.pose.position.y = position_.y();
    odometry.pose.pose.position.z = position_.z();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = velocity_.x();
    odometry.twist.twist.linear.y = velocity_.y();
    odometry.twist.twist.linear.z = velocity_.z();
    odometry.twist.twist.angular.x = angular_velocity.x();
    odometry.twist.twist.angular.y = angular_velocity.y();
    odometry.twist.twist.angular.z = angular_velocity.z();
    odom_publisher_->publish(odometry);

    if (!publish_tf_) {
        return;
    }

    geometry_msgs::msg::TransformStamped odom_transform;
    odom_transform.header.stamp = imu.header.stamp;
    odom_transform.header.frame_id = odom_frame_;
    odom_transform.child_frame_id = base_frame_;
    odom_transform.transform.translation.x = position_.x();
    odom_transform.transform.translation.y = position_.y();
    odom_transform.transform.translation.z = position_.z();
    odom_transform.transform.rotation.x = orientation.x();
    odom_transform.transform.rotation.y = orientation.y();
    odom_transform.transform.rotation.z = orientation.z();
    odom_transform.transform.rotation.w = orientation.w();
    tf_broadcaster_->sendTransform(odom_transform);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTransform>());
    rclcpp::shutdown();
    return 0;
}
