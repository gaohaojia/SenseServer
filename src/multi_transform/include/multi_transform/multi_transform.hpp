#ifndef MULTI_TRANSFORM
#define MULTI_TRANSFORM

#include <arpa/inet.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>

#include <netinet/in.h>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <thread>

#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace multi_transform
{
class MultiTransformNode : public rclcpp::Node
{
public:
  MultiTransformNode(const rclcpp::NodeOptions & options);
  ~MultiTransformNode() override;

private:
  std::thread send_thread_;
  std::thread recv_thread_;

  void NetworkSendThread();
  void NetworkRecvThread();

  sensor_msgs::msg::PointCloud2 DeserializePointCloud2(const std::vector<uint8_t>& data);
  geometry_msgs::msg::TransformStamped DeserializeTransform(const std::vector<uint8_t>& data);

  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr, client_addr, saved_client_addr[5];
  bool client_life[5];
};
} // namespace multi_transform

#endif