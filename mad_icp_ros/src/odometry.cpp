#include "mad_icp_ros/odometry.h"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mad_icp_ros/utils.h"

mad_icp_ros::Odometry::Odometry(const rclcpp::NodeOptions& options)
    : Node("mad_icp_odometry", options) {
  init_params();

  reset();

  // init_publishers();
  init_subscribers();
}

void mad_icp_ros::Odometry::reset() {
  icp_ = std::make_unique<MADicp>(b_max_, rho_ker_, b_ratio_, num_threads_);

  frame_to_map_.setIdentity();
  keyframe_to_map_.setIdentity();
  pc_container_.resize(0);
  stamp_ = rclcpp::Time();
  initialized_ = false;
  map_updated_ = false;
  seq_ = 0;
  keyframes_.resize(0);
}

void mad_icp_ros::Odometry::init_subscribers() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos,
      std::bind(&Odometry::pointcloud_callback, this, std::placeholders::_1));
}

void mad_icp_ros::Odometry::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  stamp_ = msg->header.stamp;

  RCLCPP_INFO(get_logger(), "scan %f", time_to_double(stamp_));

  if (!initialized_) {
    initialize(msg);
  }
}

void mad_icp_ros::Odometry::initialize(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  stamp_ = msg->header.stamp;

  filter_pc(msg, min_range_, max_range_, intensity_thr_, pc_container_);

  Frame* current_frame(new Frame);
  current_frame->frame_ = seq_;
  current_frame->frame_to_map_ = frame_to_map_;
  current_frame->stamp_ = time_to_double(stamp_);
  current_frame->tree_ =
      new MADtree(&pc_container_, pc_container_.begin(), pc_container_.end(),
                  b_max_, b_min_, 0, max_parallel_levels_, nullptr, nullptr);

  current_frame->tree_->getLeafs(
      std::back_insert_iterator<LeafList>(current_frame->leaves_));

  keyframes_.push_back(current_frame);

  initialized_ = true;
  map_updated_ = true;
  seq_++;
}

void mad_icp_ros::Odometry::init_params() {
  using namespace mad_icp_ros::utils;
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
  use_wheels_ = this->declare_parameter("use_wheels", false);
  num_threads_ = this->declare_parameter("num_threads", 4);
  intensity_thr_ = this->declare_parameter("intensity_thr", 0.0);

  lidar_in_base_ = parse_isometry(this->declare_parameter(
      "lidar_in_base",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0, 1.0}));
}
RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::Odometry)
