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

#include "mad_icp_ros/pose_graph_localizer.h"
#include "mad_icp_ros_utils/utils.h"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <tools/lie_algebra.h>

#include <cmath>
#include <limits>

namespace mad_icp_ros {

namespace {

Eigen::Matrix4d parseIsometry(const std::vector<double>& mat_vec) {
  return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(mat_vec.data());
}

}  // namespace

PoseGraphLocalizer::PoseGraphLocalizer(const rclcpp::NodeOptions& options)
    : Node("pose_graph_localizer", options) {
  initParams();
  initPublishers();
  initSubscribers();
  loadMap();
}

PoseGraphLocalizer::~PoseGraphLocalizer() = default;

void PoseGraphLocalizer::initParams() {
  map_dir_ = this->declare_parameter("map_dir", "");
  num_nearest_keyframes_ = this->declare_parameter("num_nearest_keyframes", 4);
  keyframe_update_rate_ = this->declare_parameter("keyframe_update_rate", 1.0);
  sensor_hz_ = this->declare_parameter("sensor_hz", 10.0);
  min_range_ = this->declare_parameter("min_range", 0.5);
  max_range_ = this->declare_parameter("max_range", 100.0);
  b_max_ = this->declare_parameter("b_max", 0.2);
  b_min_ = this->declare_parameter("b_min", 0.1);
  b_ratio_ = this->declare_parameter("b_ratio", 0.02);
  rho_ker_ = this->declare_parameter("rho_ker", 0.1);
  max_icp_its_ = this->declare_parameter("max_icp_its", 15);
  delta_chi_threshold_ = this->declare_parameter("delta_chi_threshold", 1e-6);
  num_threads_ = this->declare_parameter("num_threads", 4);
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  map_frame_ = this->declare_parameter("map_frame", "map");
  lidar_in_base_ = parseIsometry(this->declare_parameter(
      "lidar_in_base",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}));
  use_tf_for_extrinsics_ = this->declare_parameter("use_tf_for_extrinsics", false);
  publish_tf_ = this->declare_parameter("publish_tf", true);
  publish_odom_ = this->declare_parameter("publish_odom", true);
  publish_pose_ = this->declare_parameter("publish_pose", true);
  trajectory_buffer_size_ = this->declare_parameter("trajectory_buffer_size", 50);

  max_parallel_levels_ = static_cast<int>(std::log2(num_threads_));

  // Initialize ICP solver
  icp_ = std::make_unique<MADicp>(b_max_, rho_ker_, b_ratio_, num_threads_);

  // Initialize velocity estimator
  vel_estimator_ = std::make_shared<VelEstimator>(sensor_hz_);

  // Initialize poses
  map_T_base_.setIdentity();
  map_T_odom_.setIdentity();
  odom_T_base_.setIdentity();
}

void PoseGraphLocalizer::initSubscribers() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  if (use_tf_for_extrinsics_) {
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    RCLCPP_INFO(get_logger(), "Setting up tf2 MessageFilter for cloud-tf sync");

    cloud_sub_tfsync_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, "cloud_in", rmw_qos_profile_sensor_data);

    cloud_tfsync_filter_ =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            *cloud_sub_tfsync_, *tf_buffer_, base_frame_, 10,
            this->get_node_logging_interface(), this->get_node_clock_interface());

    cloud_tfsync_filter_->registerCallback(&PoseGraphLocalizer::cloudCallback, this);
  } else {
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", qos,
        std::bind(&PoseGraphLocalizer::cloudCallback, this, std::placeholders::_1));
  }

  // Initial pose subscription (from RViz 2D Pose Estimate)
  initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", qos,
          std::bind(&PoseGraphLocalizer::initialPoseCallback, this,
                    std::placeholders::_1));

  // Optional wheel odometry subscription
  wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "wheel_odom", qos,
      std::bind(&PoseGraphLocalizer::wheelOdomCallback, this, std::placeholders::_1));

  // Keyframe update timer
  if (keyframe_update_rate_ > 0) {
    auto period = std::chrono::duration<double>(1.0 / keyframe_update_rate_);
    keyframe_update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PoseGraphLocalizer::keyframeUpdateTimerCallback, this));
  }
}

void PoseGraphLocalizer::initPublishers() {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void PoseGraphLocalizer::loadMap() {
  if (map_dir_.empty()) {
    RCLCPP_ERROR(get_logger(), "map_dir parameter is not set!");
    state_ = State::UNINITIALIZED;
    return;
  }

  RCLCPP_INFO(get_logger(), "Loading map from: %s", map_dir_.c_str());

  if (!pose_index_.loadMap(map_dir_)) {
    RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", map_dir_.c_str());
    state_ = State::UNINITIALIZED;
    return;
  }

  RCLCPP_INFO(get_logger(), "Map loaded successfully with %zu poses", pose_index_.size());
  state_ = State::WAITING_INITIALPOSE;
}

void PoseGraphLocalizer::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (state_ != State::LOCALIZED) {
    if (state_ == State::WAITING_INITIALPOSE) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for initial pose on /initialpose topic");
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Map not loaded. Cannot localize.");
    }
    return;
  }

  if (active_keyframes_.empty()) {
    RCLCPP_WARN(get_logger(), "No active keyframes. Waiting for keyframe update.");
    return;
  }

  // Update extrinsics if using TF
  updateExtrinsics(msg);

  // Filter and convert point cloud
  ContainerType cloud;
  mad_icp_ros::utils::filter_pc(msg, min_range_, max_range_, 0, cloud);

  if (cloud.empty()) {
    RCLCPP_WARN(get_logger(), "Empty cloud after filtering");
    return;
  }

  // Build MADtree for current scan
  auto current_tree = std::make_shared<MADtree>(
      &cloud, cloud.begin(), cloud.end(), b_max_, b_min_, 0, max_parallel_levels_,
      nullptr, nullptr);

  LeafList current_leaves;
  current_tree->getLeafs(std::back_insert_iterator<LeafList>(current_leaves));

  // Compute motion prediction from velocity
  estimateVelocity();
  Vector6d dx = velocity_current_ / sensor_hz_;
  Eigen::Isometry3d dX;
  dX.setIdentity();
  dX.linear() = expMapSO3(dx.tail(3));
  dX.translation() = dx.head(3);

  // Initial guess: previous pose + predicted motion (in lidar frame)
  Eigen::Isometry3d map_T_lidar_prev = map_T_base_ * lidar_in_base_;
  Eigen::Isometry3d initial_guess = map_T_lidar_prev * dX;

  // Setup ICP
  icp_->setMoving(current_leaves);
  icp_->init(initial_guess);

  // ICP iterations
  double last_chi = std::numeric_limits<double>::max();
  for (size_t it = 0; it < max_icp_its_; ++it) {
    icp_->resetAdders();

    // Align against all active keyframes
    for (const auto& kf : active_keyframes_) {
      icp_->update(kf.tree);
    }

    icp_->updateState();

    double chi_change = std::abs(last_chi - icp_->chi_adder_);
    if (chi_change < delta_chi_threshold_) {
      RCLCPP_DEBUG(get_logger(), "ICP converged at iteration %zu (chi=%f)", it,
                   icp_->chi_adder_);
      break;
    }
    last_chi = icp_->chi_adder_;
  }

  // Extract result
  Eigen::Isometry3d map_T_lidar = icp_->X_;
  map_T_base_ = map_T_lidar * lidar_in_base_.inverse();

  // Update trajectory buffer for velocity estimation
  updateTrajectory(map_T_base_);

  // Publish localization result
  publishState(map_T_base_, msg->header.stamp);
}

void PoseGraphLocalizer::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (state_ == State::UNINITIALIZED) {
    RCLCPP_WARN(get_logger(), "Map not loaded yet. Cannot set initial pose.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Received initial pose");

  // Extract pose from message
  Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  q.normalize();

  map_T_base_.setIdentity();
  map_T_base_.translation() = position;
  map_T_base_.linear() = q.toRotationMatrix();

  // Reset trajectory buffer
  trajectory_buffer_.clear();
  trajectory_buffer_.push_back(map_T_base_);
  velocity_current_.setZero();

  // Update active keyframes based on initial position
  updateActiveKeyframes(position);

  if (!active_keyframes_.empty()) {
    state_ = State::LOCALIZED;
    RCLCPP_INFO(get_logger(), "Localization initialized with %zu active keyframes",
                active_keyframes_.size());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to find any keyframes near initial pose!");
  }
}

void PoseGraphLocalizer::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract odom→base transform
  Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y,
                    msg->pose.pose.position.z);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  q.normalize();

  odom_T_base_.setIdentity();
  odom_T_base_.translation() = t;
  odom_T_base_.linear() = q.toRotationMatrix();
  have_wheel_odom_ = true;
}

void PoseGraphLocalizer::keyframeUpdateTimerCallback() {
  if (state_ != State::LOCALIZED) {
    return;
  }

  Eigen::Vector3d current_position = map_T_base_.translation();
  updateActiveKeyframes(current_position);
}

void PoseGraphLocalizer::updateActiveKeyframes(const Eigen::Vector3d& position) {
  if (!pose_index_.isLoaded()) {
    return;
  }

  // Find N nearest poses
  auto nearest_ids = pose_index_.findNearestPoses(position, num_nearest_keyframes_);

  // Check if any change needed
  std::unordered_set<uint32_t> new_ids(nearest_ids.begin(), nearest_ids.end());
  if (new_ids == active_pose_ids_) {
    return;  // No change needed
  }

  // Update active keyframes
  active_keyframes_.clear();
  active_pose_ids_.clear();

  for (uint32_t pose_id : nearest_ids) {
    const auto& map_pose = pose_index_.getPose(pose_id);
    MADtree* tree = pose_index_.getTree(pose_id);

    if (!tree) {
      RCLCPP_WARN(get_logger(), "No tree found for pose %u", pose_id);
      continue;
    }

    ActiveKeyframe kf;
    kf.pose_id = pose_id;
    kf.tree = tree;
    kf.pose = map_pose.pose;

    // Get leaves for ICP
    kf.leaves.clear();
    tree->getLeafs(std::back_insert_iterator<LeafList>(kf.leaves));

    active_keyframes_.push_back(std::move(kf));
    active_pose_ids_.insert(pose_id);
  }

  RCLCPP_DEBUG(get_logger(), "Updated active keyframes: %zu", active_keyframes_.size());
}

void PoseGraphLocalizer::updateExtrinsics(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!use_tf_for_extrinsics_) {
    return;
  }

  try {
    auto t = tf_buffer_->lookupTransform(msg->header.frame_id, base_frame_,
                                         msg->header.stamp);
    lidar_in_base_ = tf2::transformToEigen(t);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Could not get lidar→base transform: %s", ex.what());
  }
}

void PoseGraphLocalizer::publishState(const Eigen::Isometry3d& map_T_base,
                                       const rclcpp::Time& stamp) {
  auto t = map_T_base.translation();
  Eigen::Quaterniond q(map_T_base.linear());
  q.normalize();

  // Compute map→odom transform
  if (have_wheel_odom_) {
    // map_T_odom = map_T_base * odom_T_base.inverse()
    map_T_odom_ = map_T_base * odom_T_base_.inverse();
  } else {
    // No wheel odom: assume odom = base
    map_T_odom_ = map_T_base;
  }

  // Publish pose
  if (publish_pose_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose.position.x = t.x();
    pose_msg.pose.pose.position.y = t.y();
    pose_msg.pose.pose.position.z = t.z();
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);
  }

  // Publish odometry
  if (publish_odom_) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg);
  }

  // Publish TF: map→odom
  if (publish_tf_) {
    auto t_odom = map_T_odom_.translation();
    Eigen::Quaterniond q_odom(map_T_odom_.linear());
    q_odom.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = t_odom.x();
    tf_msg.transform.translation.y = t_odom.y();
    tf_msg.transform.translation.z = t_odom.z();
    tf_msg.transform.rotation.x = q_odom.x();
    tf_msg.transform.rotation.y = q_odom.y();
    tf_msg.transform.rotation.z = q_odom.z();
    tf_msg.transform.rotation.w = q_odom.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void PoseGraphLocalizer::updateTrajectory(const Eigen::Isometry3d& pose) {
  if (trajectory_buffer_.size() >= trajectory_buffer_size_) {
    trajectory_buffer_.pop_front();
  }
  trajectory_buffer_.push_back(pose);
}

void PoseGraphLocalizer::estimateVelocity() {
  vel_estimator_->init(velocity_current_);

  if (trajectory_buffer_.size() < TRAJECTORY_INTERPOLATION_NUM_POSES) {
    return;  // Not enough poses yet
  }

  std::vector<Eigen::Isometry3d> odom_window;
  odom_window.reserve(TRAJECTORY_INTERPOLATION_NUM_POSES);

  int start_idx =
      std::max(0, static_cast<int>(trajectory_buffer_.size()) -
                      TRAJECTORY_INTERPOLATION_NUM_POSES);
  for (size_t i = start_idx; i < trajectory_buffer_.size(); ++i) {
    odom_window.push_back(trajectory_buffer_[i]);
  }

  vel_estimator_->setOdometry(odom_window);
  vel_estimator_->oneRound();
  velocity_current_ = vel_estimator_->X_;
}

}  // namespace mad_icp_ros

RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::PoseGraphLocalizer)
