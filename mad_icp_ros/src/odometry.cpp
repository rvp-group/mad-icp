#include "mad_icp_ros/odometry.h"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mad_icp_ros/utils.h"

mad_icp_ros::Odometry::Odometry(const rclcpp::NodeOptions& options)
    : Node("mad_icp_odometry", options) {
  init_params();

  reset();

  init_publishers();
  init_subscribers();
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

void mad_icp_ros::Odometry::compute(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  stamp_ = msg->header.stamp;

  filter_pc(msg, min_range_, max_range_, intensity_thr_, pc_container_);

  auto current_tree =
      new MADtree(&pc_container_, pc_container_.begin(), pc_container_.end(),
                  b_max_, b_min_, 0, max_parallel_levels_, nullptr, nullptr);

  LeafList current_leaves;
  current_tree->getLeafs(std::back_insert_iterator<LeafList>(current_leaves));

  // no initial guess for now
  icp_->setMoving(current_leaves);
  icp_->init(frame_to_map_);
  double last_chi = std::numeric_limits<double>::max();

  // do icp iterations
  for (size_t it{0}; it < max_icp_its_; ++it) {
    // TODO add tiem bounds for real time

    // TODO ask Leonardo what is this for. Should I set it also when I break?
    if (it == max_icp_its_ - 1)
      for (MADtree* l : current_leaves) l->matched_ = false;

    icp_->resetAdders();

#pragma omp parallel for
    for (const Frame* frame : keyframes_) {
      icp_->update(frame->tree_);
    }
#pragma omp barrier

    icp_->updateState();

    if (abs(last_chi - icp_->chi_adder_) < delta_chi_threshold_) {
      break;
    }

    last_chi = icp_->chi_adder_;
  }

  frame_to_map_ = icp_->X_;

  // Increment the list of frames
  Frame* current_frame(new Frame);
  current_frame->frame_ = seq_;
  current_frame->frame_to_map_ = frame_to_map_;
  current_frame->stamp_ = time_to_double(stamp_);
  // TODO could we use determinant().inverse()?
  // current_frame->weight_ = 1.0 / icp_->H_adder_.determinant();
  current_frame->weight_ = icp_->H_adder_.inverse().determinant();
  current_tree->applyTransform(frame_to_map_.linear(),
                               frame_to_map_.translation());
  current_frame->tree_ = current_tree;
  current_frame->leaves_ = current_leaves;

  frames_.push_back(current_frame);

  ++seq_;

  int matched_leaves = 0;
  for (MADtree* l : current_leaves) {
    if (l->matched_) {
      matched_leaves++;
    }
  }

  double inliers_ratio = double(matched_leaves) / double(current_leaves.size());

  if (inliers_ratio < p_th_) {
    double best_weight = std::numeric_limits<double>::max();
    int new_seq = 0;
    Frame* best_frame = nullptr;
    for (Frame* frame : frames_) {
      if (frame->weight_ < best_weight) {
        best_weight = frame->weight_;
        new_seq = frame->frame_;
        best_frame = frame;
      }
    }

    while (!frames_.empty() && frames_.front()->frame_ <= new_seq) {
      if (frames_.front()->frame_ < new_seq) {
        delete frames_.front()->tree_;
      }
      frames_.pop_front();
    }

    keyframes_.push_back(best_frame);
    if (keyframes_.size() > num_keyframes_) {
      delete keyframes_.front()->tree_;
      keyframes_.pop_front();
    }
  }
}

void mad_icp_ros::Odometry::reset() {
  icp_ = std::make_unique<MADicp>(b_max_, rho_ker_, b_ratio_, num_threads_);

  frame_to_map_.setIdentity();
  // keyframe_to_map_.setIdentity();
  pc_container_.resize(0);
  stamp_ = rclcpp::Time();
  initialized_ = false;
  map_updated_ = false;
  seq_ = 0;
  keyframes_.resize(0);
  frames_.resize(0);
}

void mad_icp_ros::Odometry::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  stamp_ = msg->header.stamp;

  RCLCPP_INFO(get_logger(), "scan %f", time_to_double(stamp_));

  if (!initialized_) {
    initialize(msg);
  } else {
    compute(msg);
  }

  publish_odom_tf();
}

void mad_icp_ros::Odometry::init_subscribers() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos,
      std::bind(&Odometry::pointcloud_callback, this, std::placeholders::_1));
}

void mad_icp_ros::Odometry::init_publishers() {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void mad_icp_ros::Odometry::publish_odom_tf() {
  auto pose = lidar_in_base_ * frame_to_map_ * lidar_in_base_.inverse();
  // auto vel = pipeline_->currentVel(); // extract also velocity?
  double x = pose(0, 3);
  double y = pose(1, 3);
  double z = pose(2, 3);

  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  q.normalize();

  if (publish_odom_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp_;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.child_frame_id = base_frame_;

    odom_pub_->publish(odom);
  }
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp_;
    transform.header.frame_id = "odom";
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }

  return;
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
  publish_odom_ = this->declare_parameter("publish_odom", false);
  publish_tf_ = this->declare_parameter("publish_tf", false);
  num_threads_ = this->declare_parameter("num_threads", 4);
  num_keyframes_ = this->declare_parameter("num_keyframes", 4);
  intensity_thr_ = this->declare_parameter("intensity_thr", 0.0);
  max_icp_its_ = this->declare_parameter("max_icp_its_", 15);
  delta_chi_threshold_ = this->declare_parameter("delta_chi_threshold_", 1e-6);
  frame_window_ = this->declare_parameter("frame_window_", 10);
  lidar_in_base_ = parse_isometry(this->declare_parameter(
      "lidar_in_base",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0, 1.0}));

  max_parallel_levels_ = static_cast<int>(std::log2(num_threads_));
}

RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::Odometry)
