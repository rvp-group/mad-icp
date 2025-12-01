#pragma once
#include "odometry/mad_icp.h"
#include "tools/frame.h"
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <Eigen/Dense>

namespace mad_icp_ros {
class Localizer : public rclcpp::Node {
public:
  Localizer(const rclcpp::NodeOptions &options);

protected:
  // mad_icp odometry parameters. They are explained in the
  // config/mad_icp/jackal.yaml.
  // Overriding these parameters at runtime has no effect
  double b_max_{0};
  double b_min_{0};
  double b_ratio_{0};
  double p_th_{0};
  double rho_ker_{0};
  int n_{0};
  size_t max_icp_its_{0};
  double delta_chi_threshold_{0};
  size_t frame_window_{0};
  double sensor_hz_{0};
  bool deskew_{0};
  double min_range_{0};
  double max_range_{0};
  double intensity_thr_{0};
  int max_parallel_levels_{0};
  bool realtime_{false};
  bool load_map_from_file_{false};
  Eigen::Isometry3d lidar_in_base_;
  Eigen::Isometry3d frame_to_map_;
  bool initialized_{false};

  int num_threads_{0};
  float loop_time_;

  std::string base_frame_{"base_link"};

  bool publish_tf_{false};
  bool publish_pose_{false};
  bool publish_odom_{false};
  bool use_tf_for_extrinsics_{false};

  // mad-icp
  std::unique_ptr<MADicp> icp_;
  std::shared_ptr<Frame> map_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      iguess_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Helper functions
  void reset();
  void init_params();
  void init_subscribers();
  void init_publishers();
  void initialize(const sensor_msgs::msg::PointCloud2::SharedPtr);
  void compute(const sensor_msgs::msg::PointCloud2::SharedPtr);
  // if use_tf_for_extrinsics is true, query tf tree to update lidar_in_base_
  void update_extrinsics(const sensor_msgs::msg::PointCloud2::SharedPtr);
  void publish_state(const Eigen::Isometry3d &, const rclcpp::Time &);

  // Initialize the global MAD tree with the given points
  void initialize_map(const std::vector<Eigen::Vector3d> &);

  // Callbacks
  void callback_cloud_in(const sensor_msgs::msg::PointCloud2::SharedPtr);
  void callback_initialpose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void callback_map_in(const sensor_msgs::msg::PointCloud2::SharedPtr);
};
} // namespace mad_icp_ros
