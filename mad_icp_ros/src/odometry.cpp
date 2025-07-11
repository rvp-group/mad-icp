#include "mad_icp_ros/odometry.h"

#include <sys/time.h>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mad_icp_ros_utils/utils.h"

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
  pc_stamp_ = msg->header.stamp;

  Frame* current_frame(new Frame);
  current_frame->frame_ = seq_;
  current_frame->cloud_ = new ContainerType;
  filter_pc(msg, min_range_, max_range_, intensity_thr_,
            *current_frame->cloud_);

  current_frame->frame_to_map_ = frame_to_map_;
  current_frame->stamp_ = time_to_double(pc_stamp_);
  current_frame->tree_ =
      new MADtree(current_frame->cloud_, current_frame->cloud_->begin(),
                  current_frame->cloud_->end(), b_max_, b_min_, 0,
                  max_parallel_levels_, nullptr, nullptr);

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

  struct timeval preprocessing_start, preprocessing_end, preprocessing_delta;
  gettimeofday(&preprocessing_start, nullptr);

  pc_stamp_ = msg->header.stamp;

  Frame* current_frame(new Frame);
  current_frame->frame_ = seq_;
  current_frame->cloud_ = new ContainerType;

  filter_pc(msg, min_range_, max_range_, intensity_thr_,
            *current_frame->cloud_);

  auto current_tree =
      new MADtree(current_frame->cloud_, current_frame->cloud_->begin(),
                  current_frame->cloud_->end(), b_max_, b_min_, 0,
                  max_parallel_levels_, nullptr, nullptr);

  LeafList current_leaves;
  current_tree->getLeafs(std::back_insert_iterator<LeafList>(current_leaves));

  icp_->setMoving(current_leaves);

  // TODO move to pcloud callback
  if (use_wheels_) {
    Eigen::Isometry3d initial_guess =
        (diff_ * lidar_in_base_).inverse() * wheel_to_map_ * lidar_in_base_;
    icp_->init(initial_guess);
  } else {
    icp_->init(frame_to_map_);
  }

  double last_chi = std::numeric_limits<double>::max();

  float icp_time = 0;
  float total_icp_time = 0;

  gettimeofday(&preprocessing_end, nullptr);
  timersub(&preprocessing_end, &preprocessing_start, &preprocessing_delta);
  const float preprocessing_time = float(preprocessing_delta.tv_sec) * 1000. +
                                   1e-3 * preprocessing_delta.tv_usec;

  struct timeval icp_start, icp_end, icp_delta;

  for (size_t it{0}; it < max_icp_its_; ++it) {
    const float remaining_time =
        loop_time_ - 5.0 - (preprocessing_time + total_icp_time + icp_time);

    if (realtime_ && remaining_time < 0) {
      break;
    }

    gettimeofday(&icp_start, nullptr);

    icp_->resetAdders();

#pragma omp parallel for
    for (const Frame* frame : keyframes_) {
      icp_->update(frame->tree_);
    }
#pragma omp barrier

    icp_->updateState();

    if (abs(last_chi - icp_->chi_adder_) < delta_chi_threshold_) {
      // RCLCPP_INFO(get_logger(), "its: %ld", it);
      break;
    }

    last_chi = icp_->chi_adder_;

    gettimeofday(&icp_end, nullptr);
    timersub(&icp_end, &icp_start, &icp_delta);
    icp_time = float(icp_delta.tv_sec) * 1000. + 1e-3 * icp_delta.tv_usec;

    total_icp_time += icp_time;
  }

  frame_to_map_ = icp_->X_;

  // update diff
  // TODO move this out of here
  if (use_wheels_) {
    diff_ = (wheel_to_map_ * lidar_in_base_) *
            (lidar_in_base_ * frame_to_map_).inverse();
  }

  // Increment the list of frames
  current_frame->frame_to_map_ = frame_to_map_;
  current_frame->stamp_ = time_to_double(pc_stamp_);
  current_frame->weight_ = 1.0 / icp_->H_adder_.determinant();
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

  mad_icp_ros_interfaces::msg::Frame frame_msg;
  mad_icp_ros::utils::serialize(*current_frame, frame_msg);
  frame_pub_->publish(frame_msg);

  // TODO we need to factor in also the time after the ICP for the real time
  // computation. We could maybe use the time taken in the last iteration for
  // this
}

void mad_icp_ros::Odometry::reset() {
  icp_ = std::make_unique<MADicp>(b_max_, rho_ker_, b_ratio_, num_threads_);

  frame_to_map_.setIdentity();
  wheel_to_map_.setIdentity();
  diff_.setIdentity();
  // keyframe_to_map_.setIdentity();
  pc_container_.resize(0);
  pc_stamp_ = rclcpp::Time();
  odom_stamp_ = rclcpp::Time();
  initialized_ = false;
  map_updated_ = false;
  seq_ = 0;
  keyframes_.resize(0);
  frames_.resize(0);
}

void mad_icp_ros::Odometry::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  auto time_now = now();
  pc_stamp_ = msg->header.stamp;

  RCLCPP_INFO(get_logger(), "scan %f", time_to_double(pc_stamp_));

  if (!initialized_) {
    initialize(msg);
  } else {
    compute(msg);
  }

  publish_odom_tf(lidar_in_base_ * frame_to_map_ * lidar_in_base_.inverse(),
                  pc_stamp_);

  RCLCPP_INFO(get_logger(), "took %f",
              time_to_double(now()) - time_to_double(time_now));
}

void mad_icp_ros::Odometry::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  using namespace mad_icp_ros::utils;

  wheel_to_map_.translation() << msg->pose.pose.position.x,
      msg->pose.pose.position.y, msg->pose.pose.position.z;

  wheel_to_map_.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                              msg->pose.pose.orientation.x,
                                              msg->pose.pose.orientation.y,
                                              msg->pose.pose.orientation.z)
                               .toRotationMatrix();

  if (time_to_double(odom_stamp_) < 1e-6) {
    diff_ = (wheel_to_map_ * lidar_in_base_) *
            (lidar_in_base_ * frame_to_map_).inverse();
  }

  odom_stamp_ = msg->header.stamp;
  RCLCPP_INFO(get_logger(), "odom %f", time_to_double(odom_stamp_));

  // TODO if??
  publish_odom_tf(diff_.inverse() * wheel_to_map_, odom_stamp_);
}

void mad_icp_ros::Odometry::init_subscribers() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos,
      std::bind(&Odometry::pointcloud_callback, this, std::placeholders::_1));
  if (use_wheels_)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_init", qos,
        std::bind(&Odometry::odom_callback, this, std::placeholders::_1));
}

void mad_icp_ros::Odometry::init_publishers() {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  frame_pub_ =
      this->create_publisher<mad_icp_ros_interfaces::msg::Frame>("frames", 10);
}

void mad_icp_ros::Odometry::publish_odom_tf(const Eigen::Isometry3d& pose,
                                            const rclcpp::Time& stamp) {
  // auto vel = pipeline_->currentVel(); // extract also velocity?
  double x = pose(0, 3);
  double y = pose(1, 3);
  double z = pose(2, 3);

  Eigen::Quaterniond q(pose.linear());
  q.normalize();

  if (publish_odom_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
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
    transform.header.stamp = stamp;
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

Eigen::Matrix4d parse_isometry(std::vector<double> mat_vec) {
  return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
      mat_vec.data());
}

void mad_icp_ros::Odometry::init_params() {
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
  realtime_ = this->declare_parameter("realtime", false);
  num_threads_ = this->declare_parameter("num_threads", 4);
  num_keyframes_ = this->declare_parameter("num_keyframes", 4);
  intensity_thr_ = this->declare_parameter("intensity_thr", 0.0);
  max_icp_its_ = this->declare_parameter("max_icp_its", 15);
  delta_chi_threshold_ = this->declare_parameter("delta_chi_threshold", 1e-6);
  frame_window_ = this->declare_parameter("frame_window", 10);
  lidar_in_base_ = parse_isometry(this->declare_parameter(
      "lidar_in_base",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0, 1.0}));

  max_parallel_levels_ = static_cast<int>(std::log2(num_threads_));

  loop_time_ = (1. / sensor_hz_) * 1000;

  // base_frame_ = this->declare_parameter("base_frame", "base_link");
}

RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::Odometry)
