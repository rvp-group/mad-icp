// Copyright 2024 R(obots) V(ision) and P(erception) group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <message_filters/subscriber.h>
#include <odometry/mad_icp.h>
#include <odometry/vel_estimator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tools/frame.h>

#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_set>
#include <vector>

#include "mad_icp_ros/pose_index.h"

namespace mad_icp_ros {

/**
 * @brief Pose-graph based MAD-ICP localizer
 *
 * Localizes against a pre-built map of MADtrees loaded from binary files.
 * Uses nearest-neighbor search on poses to dynamically select active keyframes.
 *
 * Nav2 compliant: publishes map→odom transform for localization.
 */
class PoseGraphLocalizer : public rclcpp::Node {
 public:
  explicit PoseGraphLocalizer(const rclcpp::NodeOptions& options);
  ~PoseGraphLocalizer() override;

 private:
  // Localizer state
  enum class State {
    UNINITIALIZED,        // Waiting for map to load
    WAITING_INITIALPOSE,  // Map loaded, waiting for initial pose
    LOCALIZED             // Actively localizing
  };

  // Initialization
  void initParams();
  void initSubscribers();
  void initPublishers();
  void loadMap();

  // Callbacks
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void keyframeUpdateTimerCallback();

  // Core localization
  void updateActiveKeyframes(const Eigen::Vector3d& position);
  void updateExtrinsics(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // TF/Publishing
  void publishState(const Eigen::Isometry3d& map_T_base,
                    const rclcpp::Time& stamp);
  void publishDebugClouds(const ContainerType& current_cloud,
                          const rclcpp::Time& stamp);

  // Velocity estimation
  void updateTrajectory(const Eigen::Isometry3d& pose);
  void estimateVelocity();

  // Parameters
  std::string map_dir_;
  size_t num_nearest_keyframes_{4};
  double keyframe_update_rate_{1.0};
  double sensor_hz_{10.0};
  double min_range_{0.5};
  double max_range_{100.0};
  double b_max_{0.2};
  double b_min_{0.1};
  double b_ratio_{0.02};
  double rho_ker_{0.1};
  size_t max_icp_its_{15};
  double delta_chi_threshold_{1e-6};
  int num_threads_{4};
  int max_parallel_levels_{2};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  std::string map_frame_{"map"};
  Eigen::Isometry3d lidar_in_base_;
  bool use_tf_for_extrinsics_{false};
  bool publish_tf_{true};
  bool publish_odom_{true};
  bool publish_pose_{true};
  bool publish_odom_base_identity_{true};
  bool publish_debug_clouds_{false};
  size_t trajectory_buffer_size_{50};

  // State
  State state_{State::UNINITIALIZED};
  Eigen::Isometry3d map_T_base_;   // Current localized pose (map to base_link)
  Eigen::Isometry3d map_T_odom_;   // Correction transform (map to odom)
  Eigen::Isometry3d odom_T_base_;  // Latest wheel odom (odom to base_link)
  bool have_wheel_odom_{false};

  // Map data
  PoseIndex pose_index_;

  // Active keyframes for ICP
  struct ActiveKeyframe {
    uint32_t pose_id;
    MADtree* tree;  // Non-owning pointer (owned by PoseIndex)
    LeafList leaves;
    Eigen::Isometry3d pose;
  };
  std::vector<ActiveKeyframe> active_keyframes_;
  std::unordered_set<uint32_t> active_pose_ids_;

  // ICP
  std::unique_ptr<MADicp> icp_;

  // Velocity estimation
  std::shared_ptr<VelEstimator> vel_estimator_;
  std::deque<Eigen::Isometry3d> trajectory_buffer_;
  Vector6d velocity_current_ = Vector6d::Zero();
  static constexpr int TRAJECTORY_INTERPOLATION_NUM_POSES = 10;

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      cloud_sub_tfsync_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
      cloud_tfsync_filter_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr keyframe_update_timer_;
};

}  // namespace mad_icp_ros
