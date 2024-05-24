#ifndef SMALL_GICP
#define SMALL_GICP

#include <pcl/impl/point_types.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace small_gicp
{
class SmallGicpNode : public rclcpp::Node
{
public:
  SmallGicpNode(const rclcpp::NodeOptions & options);

private:
  bool init_state = false;

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_point_cloud;

  void
  RegisteredScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot_0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot_1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot_2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot_3_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot_4_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
};
} // namespace small_gicp

#endif