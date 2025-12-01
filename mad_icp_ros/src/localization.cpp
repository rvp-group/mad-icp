#include "mad_icp_ros/localization.h"
#include "mad_icp_ros_utils/utils.h"
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <iterator>
#include <limits>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tools/mad_tree.h>
#include <vector>

namespace mad_icp_ros {

void convert_pointcloud2_xyz_to_eigen(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg, ContainerType &dst) {
  // Similar to mad_icp_ros::utils::filter_pc
  size_t num_points = msg->height * msg->width;
  dst.clear();
  dst.reserve(num_points);

  const uint8_t *data_ptr = msg->data.data();

  // Find field offsets
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;

  for (const auto &field : msg->fields) {
    if (field.name == "x")
      offset_x = field.offset;
    if (field.name == "y")
      offset_y = field.offset;
    if (field.name == "z")
      offset_z = field.offset;
  }

  if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
    throw std::runtime_error("PointCloud2 missing required fields");
  }

  for (size_t i = 0; i < num_points; ++i, data_ptr += msg->point_step) {
    float x = *reinterpret_cast<const float *>(data_ptr + offset_x);
    float y = *reinterpret_cast<const float *>(data_ptr + offset_y);
    float z = *reinterpret_cast<const float *>(data_ptr + offset_z);

    Eigen::Vector3d point(x, y, z);

    if (!std::isfinite(x + y + z))
      continue;

    dst.emplace_back(point);
  }
}

Localizer::Localizer(const rclcpp::NodeOptions &options)
    : Node("mad_icp_localizer", options) {
  init_params();
  reset();

  init_publishers();
  init_subscribers();
}

void Localizer::reset() {
  icp_ = std::make_unique<MADicp>(b_max_, rho_ker_, b_ratio_, num_threads_);
  frame_to_map_.setIdentity();
  initialized_ = false;
}

Eigen::Matrix4d parse_isometry(std::vector<double> mat_vec) {
  return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
      mat_vec.data());
}

void Localizer::init_params() {
  min_range_ = this->declare_parameter("min_range", 0.0);
  max_range_ = this->declare_parameter("max_range", 0.0);
  b_max_ = this->declare_parameter("b_max", 0.2);
  b_min_ = this->declare_parameter("b_min", 0.1);
  b_ratio_ = this->declare_parameter("b_ratio", 0.02);
  p_th_ = this->declare_parameter("p_th", 0.8);
  rho_ker_ = this->declare_parameter("rho_ker", 0.1);
  n_ = this->declare_parameter("n", 10);
  sensor_hz_ = this->declare_parameter("sensor_hz", 0.0);
  deskew_ = this->declare_parameter("deskew", false);
  realtime_ = this->declare_parameter("realtime", false);
  num_threads_ = this->declare_parameter("num_threads", 4);
  intensity_thr_ = this->declare_parameter("intensity_thr", 0.0);
  max_icp_its_ = this->declare_parameter("max_icp_its", 15);
  delta_chi_threshold_ = this->declare_parameter("delta_chi_threshold", 1e-6);
  frame_window_ = this->declare_parameter("frame_window", 10);
  lidar_in_base_ = parse_isometry(this->declare_parameter(
      "lidar_in_base",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0, 1.0}));
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  use_tf_for_extrinsics_ =
      this->declare_parameter("use_tf_for_extrinsics", false);

  publish_tf_ = this->declare_parameter("publish_tf", true);
  publish_pose_ = this->declare_parameter("publish_pose", true);
  publish_odom_ = this->declare_parameter("publish_odom", true);

  max_parallel_levels_ = static_cast<int>(std::log2(num_threads_));

  loop_time_ = (1. / sensor_hz_) * 1000;
}
void Localizer::init_subscribers() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", qos,
      std::bind(&Localizer::callback_cloud_in, this, std::placeholders::_1));
  iguess_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", qos,
          std::bind(&Localizer::callback_initialpose, this,
                    std::placeholders::_1));

  auto qos_map = rclcpp::QoS(rclcpp::KeepLast(1))
                     .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                     .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", qos_map,
      std::bind(&Localizer::callback_map_in, this, std::placeholders::_1));
}
void Localizer::init_publishers() {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pose", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
void Localizer::initialize(const sensor_msgs::msg::PointCloud2::SharedPtr) {}
void Localizer::compute(const sensor_msgs::msg::PointCloud2::SharedPtr) {}
void Localizer::update_extrinsics(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  Eigen::Isometry3d bTl = lidar_in_base_;
  if (use_tf_for_extrinsics_) {
    // update bTl from tf tree
    geometry_msgs::msg::TransformStamped t;
    const auto child_id = msg->header.frame_id.c_str();
  }
}
void Localizer::publish_state(const Eigen::Isometry3d &pose,
                              const rclcpp::Time &time) {
  const auto t = pose.translation();
  const auto q = Eigen::Quaterniond(pose.linear()).normalized();

  geometry_msgs::msg::PoseWithCovariance pose_msg;
  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  if (publish_pose_) {

    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = base_frame_;
    pose_stamped.pose = pose_msg;

    pose_pub_->publish(pose_stamped);
  }
  if (publish_odom_) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose = pose_msg;
    odom_pub_->publish(odom_msg);
  }
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = pose_msg.pose.position.x;
    tf_msg.transform.translation.y = pose_msg.pose.position.y;
    tf_msg.transform.translation.z = pose_msg.pose.position.z;
    tf_msg.transform.rotation.x = pose_msg.pose.orientation.x;
    tf_msg.transform.rotation.y = pose_msg.pose.orientation.y;
    tf_msg.transform.rotation.z = pose_msg.pose.orientation.z;
    tf_msg.transform.rotation.w = pose_msg.pose.orientation.w;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void Localizer::callback_cloud_in(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "Map not yet received. Skipping cloud message");
    return;
  }
  this->update_extrinsics(msg);
  std::shared_ptr<Frame> current_frame = std::make_shared<Frame>();
  current_frame->frame_ = 0; // TODO: Verify if this is okay
  current_frame->cloud_ = new ContainerType;
  mad_icp_ros::utils::filter_pc(msg, 0.5, 30, 0, *current_frame->cloud_);

  auto current_tree = std::make_shared<MADtree>(
      current_frame->cloud_, current_frame->cloud_->begin(),
      current_frame->cloud_->end(), b_max_, b_min_, 0, max_parallel_levels_,
      nullptr, nullptr);

  LeafList current_leaves;
  current_tree->getLeafs(std::back_insert_iterator<LeafList>(current_leaves));

  icp_->setMoving(current_leaves);

  auto initial_guess = frame_to_map_;
  icp_->init(initial_guess);

  double last_chi = std::numeric_limits<double>::max();
  RCLCPP_INFO(get_logger(), "##### START ######");
  for (size_t it = 0; it < max_icp_its_; ++it) {
    RCLCPP_INFO(get_logger(), "resetAdders");
    icp_->resetAdders();
    RCLCPP_INFO(get_logger(), "update");
    icp_->update(map_->tree_);
    RCLCPP_INFO(get_logger(), "updateState");
    icp_->updateState();

    // if (abs(last_chi - icp_->chi_adder_) < delta_chi_threshold_)
    //   break;

    last_chi = icp_->chi_adder_;
    RCLCPP_INFO(get_logger(), "[It=%lu] Chi = %lf", it, last_chi);
  }
  RCLCPP_INFO(get_logger(), "------- STOP -------");

  frame_to_map_ = icp_->X_;
  int matched_leaves = 0;
  for (auto *l : current_leaves) {
    matched_leaves += 1 ? l->matched_ : 0;
  }
  this->publish_state(frame_to_map_, msg->header.stamp);
}

void Localizer::callback_initialpose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {}

void Localizer::callback_map_in(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (initialized_)
    return;
  RCLCPP_INFO(get_logger(), "Received map");
  map_ = std::make_shared<Frame>();
  map_->frame_ = 0;
  map_->cloud_ = new ContainerType;
  convert_pointcloud2_xyz_to_eigen(msg, *map_->cloud_);
  map_->frame_to_map_.setIdentity();
  map_->stamp_ = mad_icp_ros::utils::time_to_double(msg->header.stamp);
  map_->tree_ =
      new MADtree(map_->cloud_, map_->cloud_->begin(), map_->cloud_->end(),
                  b_max_, b_min_, 0, max_parallel_levels_, nullptr, nullptr);

  map_->tree_->getLeafs(std::back_insert_iterator<LeafList>(map_->leaves_));
  initialized_ = true;

  icp_->update(map_->tree_);
}
} // namespace mad_icp_ros

RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::Localizer)
