#ifndef ROBOT_COMMUNICATION
#define ROBOT_COMMUNICATION

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
#include <sensor_msgs/msg/detail/image__struct.hpp>
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

#define MAX_ROBOT_COUNT 5

namespace robot_communication
{
class RobotCommunicationNode : public rclcpp::Node
{
public:
  RobotCommunicationNode(const rclcpp::NodeOptions & options);
  ~RobotCommunicationNode() override;

private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr, saved_client_addr[MAX_ROBOT_COUNT];

  int robot_count = 3;

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread prepare_buffer_thread_;
  std::thread parse_buffer_thread_[MAX_ROBOT_COUNT];

  struct SendBuffer
  {
    int id;
    std::vector<uint8_t> buffer;
  };
  std::queue<SendBuffer> buffer_queue;

  struct PrepareBuffer
  {
    int id;
    std::vector<uint8_t> buffer;
    int msg_type;
  };
  std::queue<PrepareBuffer> prepare_buffer_queue;
  std::queue<std::vector<uint8_t>> parse_buffer_queue[MAX_ROBOT_COUNT];

  void InitServer();

  void NetworkSendThread();
  void NetworkRecvThread();
  void PrepareBufferThread();
  void ParseBufferThread(const int robot_id);

  void WayPointCallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg,
                        const int robot_id);

  template <class T> std::vector<uint8_t> SerializeMsg(const T & msg);
  template <class T> T DeserializeMsg(const std::vector<uint8_t> & data);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr realsense_image_pub_[MAX_ROBOT_COUNT];

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr way_point_sub_[MAX_ROBOT_COUNT];
};
} // namespace robot_communication

#endif