#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mad_icp_ros {
class MapServer : public rclcpp::Node {
public:
  MapServer(const rclcpp::NodeOptions &options);

protected:
  void init_params();
  void init_publishers();
  void load_map();

  std::string map_filename_;
  std::string map_format_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_lowres_;
  sensor_msgs::msg::PointCloud2 map_msg_, map_lowres_msg_;
};
} // namespace mad_icp_ros
