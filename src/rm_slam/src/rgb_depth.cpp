#include "rgb_depth.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h> 
#include <pcl/filters/voxel_grid.h>
RgbDepth::RgbDepth()
    : rclcpp::Node("rgb_depth_node")
        {
            sub_depth_raw = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/odin1/depthmod/depthmod/points",
                rclcpp::SensorDataQoS(),
                std::bind(&RgbDepth::depth_info_feedback, this, std::placeholders::_1));

            pub_depth_info = create_publisher<sensor_msgs::msg::PointCloud2>(
                "/filtered_depth_frame",
                rclcpp::SensorDataQoS());

            pub_depth_scan = create_publisher<sensor_msgs::msg::PointCloud2>(
                "/filtered_depth_scan",
                rclcpp::SensorDataQoS());
        }
void RgbDepth::depth_info_feedback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 清洗NaN点，输出深度图像点云数据
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(
        input_cloud,
        output_cloud,
        valid_indices);
    RCLCPP_DEBUG(this->get_logger(), "input points: %zu, output points: %zu", input_cloud.size(), output_cloud.size());
    
    // 裁剪距离，只处理 Odin1 的 0.2m～30m 有效量程
    pcl::PointCloud<pcl::PointXYZ> filter_distance_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(output_cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2f, 30.0f);
    pass.filter(filter_distance_cloud);
    RCLCPP_DEBUG(get_logger(),"raw=%zu valid=%zu range=%zu",input_cloud.size(),output_cloud.size(),filter_distance_cloud.size());

    // 稠密分支：保留角分辨率，供 PointCloud2 -> LaserScan 和 RTAB-Map 使用。
    sensor_msgs::msg::PointCloud2 scan_input_msg;
    pcl::toROSMsg(filter_distance_cloud, scan_input_msg);
    scan_input_msg.header = msg->header;
    pub_depth_scan->publish(scan_input_msg);

    //重采样icp,提取特征点，降低icp处理压力
    pcl::PointCloud<pcl::PointXYZ> voxel_output_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(filter_distance_cloud.makeShared());
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // ICP 分支使用 5cm 体素
    voxel_filter.filter(voxel_output_cloud);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "raw=%zu valid=%zu range=%zu voxel=%zu",
        input_cloud.size(), output_cloud.size(),
        filter_distance_cloud.size(), voxel_output_cloud.size());

    //发布处理后的点云数据
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(voxel_output_cloud, output_msg);
    output_msg.header = msg->header;
    pub_depth_info->publish(output_msg);


}
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbDepth>());
    rclcpp::shutdown();
    return 0;
}
