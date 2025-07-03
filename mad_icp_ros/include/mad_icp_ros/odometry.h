#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "odometry/pipeline.h"

namespace mad_icp_ros {

class Odometry : public rclcpp::Node {
 public:
  Odometry(const rclcpp::NodeOptions& options);

 protected:
  // TODO make these ROS2 parameters
  std::string frame_id_{"os0_sensor"};
  size_t intensity_thr_{0};
  //

  std::string dataset_config_file_path_;
  std::string mad_icp_config_file_path_;

  std::unique_ptr<Pipeline> pipeline_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  //

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // store each message here
  ContainerType pc_container_;
  // PointCloud2 -> std::vector -> madtree (can I save 1 conversion if I write
  // PointCloud2 to madtree?)

  double min_range_{0};
  double max_range_{0};
  double b_max_{0};
  double b_min_{0};
  double b_ratio_{0};
  double p_th_{0};
  double rho_ker_{0};
  int n_{0};

  double sensor_hz_{0};
  bool deskew_{0};
  Eigen::Matrix4d lidar_to_base_;

  rclcpp::Time stamp_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void publish_odom_tf() const;

  void reset();
};
}  // namespace mad_icp_ros