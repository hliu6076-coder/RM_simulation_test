#include "odom_depth_tf.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>

namespace
{
Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond rotation(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z);
  rotation.normalize();

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    transform.translation.x,
    transform.translation.y,
    transform.translation.z);
  return result;
}
}  // namespace

OdomDepthTransform::OdomDepthTransform()
: rclcpp::Node("odom_depth_transform_node")
{
  previous_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sub_depth_filtered_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/filtered_depth_frame", rclcpp::SensorDataQoS(),
    std::bind(&OdomDepthTransform::depth_filter_feedback, this, std::placeholders::_1));

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
}

void OdomDepthTransform::save_reference(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const rclcpp::Time & stamp)
{
  *previous_cloud_ = *cloud;
  previous_stamp_ = stamp;
  reference_initialized_ = true;
}

void OdomDepthTransform::depth_filter_feedback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  auto current_cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(*msg, *current_cloud);
  if (current_cloud->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到空点云，跳过本帧");
    return;
  }

  const rclcpp::Time current_stamp(msg->header.stamp, RCL_ROS_TIME);
  if (!reference_initialized_) {
    save_reference(current_cloud, current_stamp);
    RCLCPP_INFO(get_logger(), "已保存第一帧点云作为 ICP 参考帧");
    return;
  }

  const double dt = (current_stamp - previous_stamp_).seconds();
  // 仿真负载较高时 10 Hz 点云可能延迟到数百毫秒；超过 1 秒才视为断流。
  if (dt <= 0.0 || dt > 1.0) {
    RCLCPP_WARN(get_logger(), "异常点云时间间隔: %.3f s，重置参考帧", dt);
    save_reference(current_cloud, current_stamp);
    return;
  }

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(current_cloud);
  icp.setInputTarget(previous_cloud_);
  icp.setMaximumIterations(30);
  icp.setMaxCorrespondenceDistance(0.30);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-5);

  CloudT aligned_cloud;
  icp.align(aligned_cloud);  // 每帧只执行一次 ICP。

  if (!icp.hasConverged()) {
    RCLCPP_WARN(get_logger(), "ICP 没有收敛，保持位姿并重置参考帧，dt=%.3f", dt);
    save_reference(current_cloud, current_stamp);
    return;
  }

  // source=当前帧、target=上一帧，结果是 T_previous_sensor_current_sensor。
  const double fitness = icp.getFitnessScore();
  Eigen::Isometry3d previous_sensor_current_sensor = Eigen::Isometry3d::Identity();
  previous_sensor_current_sensor.matrix() = icp.getFinalTransformation().cast<double>();
  Eigen::Quaterniond icp_rotation(previous_sensor_current_sensor.rotation());
  icp_rotation.normalize();
  previous_sensor_current_sensor.linear() = icp_rotation.toRotationMatrix();

  geometry_msgs::msg::TransformStamped base_sensor_msg;
  try {
    base_sensor_msg = tf_buffer_->lookupTransform(
      "base_link", msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN(
      get_logger(), "无法取得 base_link <- %s 外参: %s",
      msg->header.frame_id.c_str(), error.what());
    save_reference(current_cloud, current_stamp);
    return;
  }

  // T_previous_base_current_base =
  // T_base_sensor * T_previous_sensor_current_sensor * T_sensor_base。
  const Eigen::Isometry3d base_sensor = transform_to_eigen(base_sensor_msg.transform);
  const Eigen::Isometry3d previous_base_current_base =
    base_sensor * previous_sensor_current_sensor * base_sensor.inverse();

  const double translation_step = previous_base_current_base.translation().norm();
  const Eigen::AngleAxisd rotation_step(previous_base_current_base.rotation());
  if (!std::isfinite(fitness) || fitness > 0.20 ||
    translation_step > 0.50 || std::abs(rotation_step.angle()) > 0.80)
  {
    RCLCPP_WARN(
      get_logger(), "拒绝异常 ICP: score=%.4f translation=%.3f rotation=%.3f",
      fitness, translation_step, std::abs(rotation_step.angle()));
    save_reference(current_cloud, current_stamp);
    return;
  }

  odom_base_pose_ = odom_base_pose_ * previous_base_current_base;
  publish_odometry(msg->header.stamp, dt, previous_base_current_base, fitness);
  save_reference(current_cloud, current_stamp);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "深度 ICP 里程计运行中: dt=%.3f score=%.6f", dt, fitness);
}

void OdomDepthTransform::publish_odometry(
  const builtin_interfaces::msg::Time & stamp,
  double dt,
  const Eigen::Isometry3d & relative_base_pose,
  double fitness)
{
  const Eigen::Vector3d position = odom_base_pose_.translation();
  Eigen::Quaterniond orientation(odom_base_pose_.rotation());
  orientation.normalize();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = position.x();
  odom_msg.pose.pose.position.y = position.y();
  odom_msg.pose.pose.position.z = position.z();
  odom_msg.pose.pose.orientation.x = orientation.x();
  odom_msg.pose.pose.orientation.y = orientation.y();
  odom_msg.pose.pose.orientation.z = orientation.z();
  odom_msg.pose.pose.orientation.w = orientation.w();

  const double pose_variance = std::clamp(fitness, 1e-4, 1.0);
  odom_msg.pose.covariance.fill(0.0);
  odom_msg.pose.covariance[0] = pose_variance;
  odom_msg.pose.covariance[7] = pose_variance;
  odom_msg.pose.covariance[14] = 2.0 * pose_variance;
  odom_msg.pose.covariance[21] = 2.0 * pose_variance;
  odom_msg.pose.covariance[28] = 2.0 * pose_variance;
  odom_msg.pose.covariance[35] = pose_variance;

  odom_msg.twist.twist.linear.x = relative_base_pose.translation().x() / dt;
  odom_msg.twist.twist.linear.y = relative_base_pose.translation().y() / dt;
  odom_msg.twist.twist.linear.z = relative_base_pose.translation().z() / dt;
  const Eigen::AngleAxisd delta_rotation(relative_base_pose.rotation());
  odom_msg.twist.twist.angular.x = delta_rotation.axis().x() * delta_rotation.angle() / dt;
  odom_msg.twist.twist.angular.y = delta_rotation.axis().y() * delta_rotation.angle() / dt;
  odom_msg.twist.twist.angular.z = delta_rotation.axis().z() * delta_rotation.angle() / dt;
  odom_msg.twist.covariance.fill(0.0);
  odom_msg.twist.covariance[0] = pose_variance / (dt * dt);
  odom_msg.twist.covariance[7] = pose_variance / (dt * dt);
  odom_msg.twist.covariance[14] = 2.0 * pose_variance / (dt * dt);
  odom_msg.twist.covariance[21] = 2.0 * pose_variance / (dt * dt);
  odom_msg.twist.covariance[28] = 2.0 * pose_variance / (dt * dt);
  odom_msg.twist.covariance[35] = pose_variance / (dt * dt);

  odom_publisher_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped odom_transform;
  odom_transform.header = odom_msg.header;
  odom_transform.child_frame_id = odom_msg.child_frame_id;
  odom_transform.transform.translation.x = position.x();
  odom_transform.transform.translation.y = position.y();
  odom_transform.transform.translation.z = position.z();
  odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(odom_transform);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDepthTransform>());
  rclcpp::shutdown();
  return 0;
}
