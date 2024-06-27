#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "multi_transform/multi_transform.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions & options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  // UDP
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  memset(&server_addr, 0, sizeof(client_addr));
  memset(client_life, 0, sizeof(client_life));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Bind failed!");
    close(sockfd);
    return;
  }

  send_thread_ = std::thread(&MultiTransformNode::NetworkSendThread, this);
  recv_thread_ = std::thread(&MultiTransformNode::NetworkRecvThread, this);
  RCLCPP_INFO(this->get_logger(), "Server start at ip: %s, port: %d", ip.c_str(), port);
}

MultiTransformNode::~MultiTransformNode()
{
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  close(sockfd);
}

void MultiTransformNode::NetworkSendThread() {}

void MultiTransformNode::NetworkRecvThread()
{
  int n, len = sizeof(client_addr);
  int packet_idx[5] = {0, 0, 0, 0, 0};
  int packet_type[5] = {-1, -1, -1, -1, -1};
  std::vector<uint8_t> buffer[5];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_[5];
  for (int i = 0; i < 5; i++) {
    registered_scan_pub_[i] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/robot_" + std::to_string(i) + "/total_registered_scan", 5);
  }
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer_tmp.data(),
                 BUFFER_SIZE,
                 MSG_WAITALL,
                 (struct sockaddr *)&client_addr,
                 (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t), sizeof(max_idx));

    if (packet_type[id] == -1){
      packet_type[id] = type;
    }else if (packet_type[id] < type){
      continue;
    }else if (packet_type[id] > type){
      packet_type[id] = type;
      packet_idx[id] = 0;
      buffer[id] = std::vector<uint8_t>(0);
    }
    if (idx == 0){
      packet_idx[id] = 0;
      buffer[id] = std::vector<uint8_t>(0);
    }else if (packet_idx[id] != idx) {
      packet_idx[id] = 0;
      packet_type[id] = -1;
      buffer[id] = std::vector<uint8_t>(0);
      continue;
    }

    packet_idx[id]++;
    if (packet_idx[id] == 1) {
      buffer[id].insert(buffer[id].begin(),
                        buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                        buffer_tmp.end());
    } else {
      buffer[id].insert(buffer[id].end(),
                        buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                        buffer_tmp.end());
    }

    if (packet_idx[id] != max_idx) {
      continue;
    }
    try {
      if (type == 0) { // PointCloud2
        std::shared_ptr<sensor_msgs::msg::PointCloud2> totalRegisteredScan =
          std::make_shared<sensor_msgs::msg::PointCloud2>(
            MultiTransformNode::DeserializePointCloud2(buffer[id]));
        registered_scan_pub_[id]->publish(*totalRegisteredScan);
      } else if (type == 1) { // Transform
        std::shared_ptr<geometry_msgs::msg::TransformStamped> transformStamped =
          std::make_shared<geometry_msgs::msg::TransformStamped>(
            MultiTransformNode::DeserializeTransform(buffer[id]));
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_broadcaster_->sendTransform(*transformStamped);
      }
    } catch (...) {
    }

    packet_idx[id] = 0;
    packet_type[id] = -1;
    buffer[id] = std::vector<uint8_t>(0);

    // sendto(sockfd, "got!", strlen("got!"), 0, (const struct sockaddr *)&client_addr, len);
  }
}

// PointCloud2 Deserialization
sensor_msgs::msg::PointCloud2
MultiTransformNode::DeserializePointCloud2(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  sensor_msgs::msg::PointCloud2 pointcloud2_msg;
  serializer.deserialize_message(&serialized_msg, &pointcloud2_msg);

  return pointcloud2_msg;
}

// Transform Deserialization
geometry_msgs::msg::TransformStamped
MultiTransformNode::DeserializeTransform(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<geometry_msgs::msg::TransformStamped> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  geometry_msgs::msg::TransformStamped transform_msg;
  serializer.deserialize_message(&serialized_msg, &transform_msg);

  return transform_msg;
}
} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)