#include "scan_process.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <limits>
#include <cmath>

ScanProcess::ScanProcess(): rclcpp::Node("scan_process_node")
{
  sub_pointcloud_process = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/filtered_depth_scan",
    rclcpp::SensorDataQoS(),
    std::bind(&ScanProcess::scan_process_feedback, this, std::placeholders::_1));

  pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/points_base", rclcpp::SensorDataQoS());
  scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}
void ScanProcess::scan_process_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  try {
    const auto transform = tf_buffer_->lookupTransform(
      "base_link", msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.05));

    sensor_msgs::msg::PointCloud2 cloud_base;
    tf2::doTransform(*msg, cloud_base, transform);
    cloud_base.header.stamp = msg->header.stamp;
    cloud_base.header.frame_id = "base_link";
    pointcloud_publisher_->publish(cloud_base);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = "base_link";
    scan.angle_min = -static_cast<float>(M_PI) / 3.0f;
    scan.angle_max =  static_cast<float>(M_PI) / 3.0f;
    scan.angle_increment = static_cast<float>(M_PI) / 360.0f;
    scan.scan_time = 0.1f;
    scan.time_increment = 0.0f;
    scan.range_min = 0.2f;
    scan.range_max = 30.0f;

    const std::size_t beam_count = static_cast<std::size_t>(std::lround(
      (scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1U;
    scan.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_base, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_base, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_base, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < 0.05f || z > 0.60f) {
        continue;
      }

      const float range = std::hypot(x, y);
      const float angle = std::atan2(y, x);
      if (range < scan.range_min || range > scan.range_max ||
          angle < scan.angle_min || angle > scan.angle_max) {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(
        (angle - scan.angle_min) / scan.angle_increment);
      if (index < scan.ranges.size() && range < scan.ranges[index]) {
        scan.ranges[index] = range;
      }
    }

    scan_publisher_->publish(scan);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "无法把点云变换到 base_link: %s", error.what());
  }
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanProcess>());
  rclcpp::shutdown();
  return 0;
}
