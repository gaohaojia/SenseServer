#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
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
  std::vector<uint8_t> buffer[5];
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
           buffer_tmp.data(),
           BUFFER_SIZE,
           MSG_WAITALL,
           (struct sockaddr *)&client_addr,
           (socklen_t *)&len);
    if (n < 0){
      continue;
    }
    buffer_tmp.resize(n);
    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t), sizeof(max_idx));
    
    RCLCPP_INFO(this->get_logger(), "now:%d id:%u type:%u idx:%u max_idx:%u", packet_idx[id], id, type, idx, max_idx);
    if (packet_idx[id] != idx){
      packet_idx[id] = 0;
      buffer[id] = std::vector<uint8_t>(0);
      continue;
    }
    packet_idx[id]++;
    RCLCPP_INFO(this->get_logger(), "1");
    const int l = buffer[id].size() + buffer_tmp.size();
    RCLCPP_INFO(this->get_logger(), "%d", l);
    buffer[id].resize(l);
    RCLCPP_INFO(this->get_logger(), "3");
    if (packet_idx[id] == 1){
      buffer[id].insert(buffer[id].begin(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    }else{
      buffer[id].insert(buffer[id].end(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    }

    RCLCPP_INFO(this->get_logger(), "%ld", buffer_tmp.size());

    if (packet_idx[id] != max_idx){
      continue;
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud2> totalRegisteredScan =
      std::make_shared<sensor_msgs::msg::PointCloud2>(
        MultiTransformNode::DeserializePointCloud2(buffer[id]));
    RCLCPP_INFO(this->get_logger(), "%d", totalRegisteredScan->header.stamp.sec);

    packet_idx[id] = 0;
    buffer[id] = std::vector<uint8_t>(0);
    
    // sendto(sockfd, "got!", strlen("got!"), 0, (const struct sockaddr *)&client_addr, len);
  }
}

// 反序列化 PointCloud2 函数
sensor_msgs::msg::PointCloud2
MultiTransformNode::DeserializePointCloud2(const std::vector<uint8_t> & buffer_tmp)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  // 将字节数组复制到序列化消息中
  serialized_msg.get_rcl_serialized_message().buffer_length = buffer_tmp.size();
  serialized_msg.get_rcl_serialized_message().buffer_capacity = buffer_tmp.size();
  serialized_msg.get_rcl_serialized_message().buffer = const_cast<uint8_t *>(buffer_tmp.data());

  // 反序列化消息
  sensor_msgs::msg::PointCloud2 pointcloud2_msg;
  serializer.deserialize_message(&serialized_msg, &pointcloud2_msg);

  return pointcloud2_msg;
}

} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)