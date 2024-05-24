#include "small_gicp/small_gicp.hpp"
#include <functional>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include <small_gicp/pcl/pcl_registration.hpp>

#define REG_TYPE RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>

namespace small_gicp
{
SmallGicpNode::SmallGicpNode(const rclcpp::NodeOptions & options) : Node("small_gicp", options) {
  robot_0_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/robot_0/registered_scan", 5, std::bind(&SmallGicpNode::RegisteredScanCallBack, this, std::placeholders::_1));
  robot_1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/robot_1/registered_scan", 5, std::bind(&SmallGicpNode::RegisteredScanCallBack, this, std::placeholders::_1));
  robot_2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/robot_2/registered_scan", 5, std::bind(&SmallGicpNode::RegisteredScanCallBack, this, std::placeholders::_1));
  robot_3_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/robot_3/registered_scan", 5, std::bind(&SmallGicpNode::RegisteredScanCallBack, this, std::placeholders::_1));
  robot_4_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/robot_4/registered_scan", 5, std::bind(&SmallGicpNode::RegisteredScanCallBack, this, std::placeholders::_1));

  aligned_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_scan", 5);
}

void SmallGicpNode::RegisteredScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg){
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  if (!init_state) {
    aligned_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pointcloud_tmp);
    init_state = true;
  }else{
    std::shared_ptr<REG_TYPE> reg(new REG_TYPE());
    reg->setNumThreads(8);
    reg->setCorrespondenceRandomness(20);
    reg->setMaxCorrespondenceDistance(1.0);
    reg->setVoxelResolution(1.0);
    reg->setRegistrationType("VGICP");

    reg->setInputTarget(aligned_point_cloud);
    reg->setInputSource(pointcloud_tmp);

    reg->align(*aligned_point_cloud);
  }
  std::shared_ptr<sensor_msgs::msg::PointCloud2> aligned_msg(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(*aligned_point_cloud, *aligned_msg);
  aligned_msg->header.stamp = registered_scan_msg->header.stamp;
  aligned_msg->header.frame_id = "map";
  aligned_scan_pub_->publish(*aligned_msg);
}
} // namespace small_gicp

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp::SmallGicpNode)