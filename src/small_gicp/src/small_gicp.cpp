#include "small_gicp/small_gicp.hpp"
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include <small_gicp/pcl/pcl_registration.hpp>

#define REG_TYPE RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>

namespace small_gicp
{
SmallGicpNode::SmallGicpNode(const rclcpp::NodeOptions & options) : Node("small_gicp", options)
{
  this->declare_parameter<int>("robot_count", 5);
  this->get_parameter("robot_count", robot_count);

  robot_0_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_0/explored_areas",
    5,
    std::bind(&SmallGicpNode::Robot0CallBack, this, std::placeholders::_1));
  robot_1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_1/explored_areas",
    5,
    std::bind(&SmallGicpNode::Robot1CallBack, this, std::placeholders::_1));
  robot_2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_2/explored_areas",
    5,
    std::bind(&SmallGicpNode::Robot2CallBack, this, std::placeholders::_1));
  robot_3_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_3/explored_areas",
    5,
    std::bind(&SmallGicpNode::Robot3CallBack, this, std::placeholders::_1));
  robot_4_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_4/explored_areas",
    5,
    std::bind(&SmallGicpNode::Robot4CallBack, this, std::placeholders::_1));

  align_thread_ = std::thread(&SmallGicpNode::AlignPointCloud, this);

  aligned_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_scan", 5);
  RCLCPP_INFO(this->get_logger(), "Small GICP Node Start! Robot count is %d.", robot_count);
}

SmallGicpNode::~SmallGicpNode()
{
  if (align_thread_.joinable()) {
    align_thread_.join();
  }
}

void SmallGicpNode::Robot0CallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  robot_point_cloud[0] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
  init_state = true;
}

void SmallGicpNode::Robot1CallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  robot_point_cloud[1] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
}
void SmallGicpNode::Robot2CallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  robot_point_cloud[2] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
}
void SmallGicpNode::Robot3CallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  robot_point_cloud[3] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
}
void SmallGicpNode::Robot4CallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  robot_point_cloud[4] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
}

void SmallGicpNode::AlignPointCloud()
{
  std::shared_ptr<REG_TYPE> reg(new REG_TYPE());
  reg->setNumThreads(8);
  reg->setCorrespondenceRandomness(20);
  reg->setMaxCorrespondenceDistance(1.0);
  reg->setVoxelResolution(1.0);
  reg->setRegistrationType("VGICP");

  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (!init_state) {
      continue;
    }
    aligned_result = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*robot_point_cloud[0]);
    for (int i = 1; i < robot_count; i++) {
      reg->setInputTarget(aligned_result);
      reg->setInputSource(robot_point_cloud[i]);
      aligned_tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      reg->align(*aligned_tmp);
      aligned_result = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*aligned_tmp);
    }
    aligned_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*aligned_result, *aligned_msg);
    aligned_msg->header.stamp = this->now();
    aligned_msg->header.frame_id = "map";
    aligned_scan_pub_->publish(*aligned_msg);
  }
}
} // namespace small_gicp

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp::SmallGicpNode)