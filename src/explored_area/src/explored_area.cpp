#include <pcl/kdtree/kdtree_flann.h>

#include <functional>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "explored_area/explored_area.hpp"

namespace explored_area {
ExploredAreaNode::ExploredAreaNode(const rclcpp::NodeOptions &options)
    : Node("explored_area", options) {
  this->declare_parameter<int>("robot_count", 3);
  this->get_parameter("robot_count", robot_count);

  for (int i = 0; i < robot_count; i++) {
    registered_scan_sub_[i] =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_" + std::to_string(i) + "/total_registered_scan",
        rclcpp::SensorDataQoS(),
        std::bind(&ExploredAreaNode::RegisteredScanCallBack, this,
                  std::placeholders::_1));
  }

  for (int i = 0; i < robot_count; i++) {
    registered_scan_sub_[i] =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_" + std::to_string(i) + "/realsense_pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&ExploredAreaNode::RealsenseScanCallBack, this,
                  std::placeholders::_1));
  }

  explored_area_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);
  explored_volume_pub_ =
    this->create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);
  traveling_dis_pub_ =
    this->create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);

  exploredVolumeCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredVolumeCloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredRGBVolumeCloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  exploredRGBVolumeCloud2 =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  exploredAreaCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredAreaCloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredRGBAreaCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  exploredRGBAreaCloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  exploredVolumeDwzFilter.setLeafSize(
    exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);
  exploredAreaDwzFilter.setLeafSize(
    exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredRGBVolumeDwzFilter.setLeafSize(
    exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);
  exploredRGBAreaDwzFilter.setLeafSize(
    exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
}

void ExploredAreaNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize *
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = registered_scan_msg->header.stamp;
    exploredArea2.header.frame_id = registered_scan_msg->header.frame_id;
    explored_area_pub_->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  explored_volume_pub_->publish(exploredVolumeMsg);

  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  traveling_dis_pub_->publish(travelingDisMsg);
}

void ExploredAreaNode::RealsenseScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr realsense_scan_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*realsense_scan_msg, *laserCloud);

  *exploredRGBVolumeCloud += *laserCloud;

  exploredRGBVolumeCloud2->clear();
  exploredRGBVolumeDwzFilter.setInputCloud(exploredRGBVolumeCloud);
  exploredRGBVolumeDwzFilter.filter(*exploredRGBVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud = exploredRGBVolumeCloud;
  exploredRGBVolumeCloud = exploredRGBVolumeCloud2;
  exploredRGBVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize *
                   exploredVolumeVoxelSize *
                   exploredRGBVolumeCloud->points.size();

  *exploredRGBAreaCloud += *laserCloud;
}
}  // namespace explored_area

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(explored_area::ExploredAreaNode)