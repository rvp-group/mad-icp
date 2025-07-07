#include "mad_icp_ros/odometry.h"

#include <message_filters/synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

Eigen::Matrix4d parseMatrix(const std::vector<std::vector<double>>& vec);

int num_cores = 4;
int num_keyframes = 4;
bool realtime = false;
bool kitti = false;

double time_to_double(const builtin_interfaces::msg::Time& t) {
  return t.sec + t.nanosec * 1e-9;
}

mad_icp_ros::Odometry::Odometry(const rclcpp::NodeOptions& options)
    : Node("mad_icp", options) {
  // ROS2 Parameters

  this->declare_parameter<bool>("use_odom", true);
  this->get_parameter("use_odom", use_odom_);

  // TODO eventually move the config files into launch files

  // Load parameters file
  std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("mad_icp");

  this->declare_parameter<std::string>(
      "dataset_config_file",
      package_share_dir + "/configurations/datasets/vbr_os0.cfg");

  this->get_parameter("dataset_config_file", dataset_config_file_path_);

  this->declare_parameter<std::string>(
      "mad_icp_config_file", package_share_dir + "/configurations/default.cfg");

  this->get_parameter("mad_icp_config_file", mad_icp_config_file_path_);

  // Parse the parameters
  YAML::Node yaml_dataset_config, yaml_mad_icp_config;
  // RCLCPP_INFO(get_logger(), dataset_config_file_path_.c_str());
  // RCLCPP_INFO(get_logger(), mad_icp_config_file_path_.c_str());

  yaml_dataset_config = YAML::LoadFile(dataset_config_file_path_);
  yaml_mad_icp_config = YAML::LoadFile(mad_icp_config_file_path_);

  min_range_ = yaml_dataset_config["min_range"].as<double>();
  max_range_ = yaml_dataset_config["max_range"].as<double>();
  sensor_hz_ = yaml_dataset_config["sensor_hz"].as<double>();
  deskew_ = yaml_dataset_config["deskew"].as<bool>();
  // parsing lidar in base homogenous transformation
  const auto lidar_in_base_vec = yaml_dataset_config["lidar_to_base"]
                                     .as<std::vector<std::vector<double>>>();
  lidar_in_base_ = parseMatrix(lidar_in_base_vec);

  // parse mad-icp configuration
  b_max_ = yaml_mad_icp_config["b_max"].as<double>();
  b_min_ = yaml_mad_icp_config["b_min"].as<double>();
  b_ratio_ = yaml_mad_icp_config["b_ratio"].as<double>();
  p_th_ = yaml_mad_icp_config["p_th"].as<double>();
  rho_ker_ = yaml_mad_icp_config["rho_ker"].as<double>();
  n_ = yaml_mad_icp_config["n"].as<int>();

  reset();

  // Instance the odom publisher
  // TODO think about which QOS we want
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Instance the tf2 broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  out_dump_.open("dump.txt");

  // Subscribers initialization

  // pointclouds:
  // set the qos to match the ouster
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  // pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "points", qos,
  //     std::bind(&Odometry::pointcloud_callback, this,
  //     std::placeholders::_1));
  pc_sub_.subscribe(this, "/points", qos.get_rmw_qos_profile());
  odom_sub_.subscribe(this, "/odom_init");

  sync_.reset(new Sync(Points_Odom_Sync_Policy(10), pc_sub_, odom_sub_));
  sync_->registerCallback(std::bind(
      &Odometry::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void mad_icp_ros::Odometry::callback(
    std::shared_ptr<const sensor_msgs::msg::PointCloud2> points_msg,
    std::shared_ptr<const nav_msgs::msg::Odometry> odom_msg) {
  stamp_ = points_msg->header.stamp;
  auto odom_stamp = odom_msg->header.stamp;

  RCLCPP_INFO(get_logger(), "scan %f", time_to_double(stamp_));
  RCLCPP_INFO(get_logger(), "delta pc/odom %f",
              time_to_double(stamp_) - time_to_double(odom_stamp));

  // mad icp uses unordered clouds
  auto height = points_msg->height;
  auto width = points_msg->width;

  // Convert the PointCloud2 message to mad_icp's ContainerType (std::vector
  // of 3d points)
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_int(*points_msg,
                                                        "intensity");

  pc_container_.clear();
  pc_container_.reserve(points_msg->height * points_msg->width);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_int) {
    auto point = Eigen::Vector3d{*iter_x, *iter_y, *iter_z};
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z()) || *iter_int < intensity_thr_ ||
        point.norm() < min_range_ || point.norm() > max_range_)
      continue;
    pc_container_.emplace_back(point);
  }

  if (use_odom_) {
    auto T1 = Eigen::Isometry3d::Identity();
    T1.translation() << odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;

    T1.linear() = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                     odom_msg->pose.pose.orientation.x,
                                     odom_msg->pose.pose.orientation.y,
                                     odom_msg->pose.pose.orientation.z)
                      .toRotationMatrix();
    auto delta = T0_.inverse() * T1;

    std::cerr << "initial_guess \n" << delta.matrix() << std::endl;

    pipeline_->compute(time_to_double(stamp_), pc_container_, delta);

    T0_ = T1;
  } else {
    pipeline_->compute(time_to_double(stamp_), pc_container_);
  }

  publish_odom_tf();
}

void mad_icp_ros::Odometry::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // auto new_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  // if stamp_ > new_stamp {

  // }
  stamp_ = msg->header.stamp;

  if (use_odom_) {
    auto initial_guess = T0_.inverse();
    pipeline_->compute(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                       pc_container_, initial_guess);

    // reset the odometry counter

    T0_ = Eigen::Isometry3d::Identity();
  } else {
    pipeline_->compute(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                       pc_container_);
  }
  publish_odom_tf();
}

void mad_icp_ros::Odometry::reset() {
  stamp_ = rclcpp::Time();
  pc_container_.clear();

  pipeline_ = std::make_unique<Pipeline>(sensor_hz_, deskew_, b_max_, rho_ker_,
                                         p_th_, b_min_, b_ratio_, num_keyframes,
                                         num_cores, realtime);

  T0_ = Eigen::Isometry3d::Identity();
}

void mad_icp_ros::Odometry::publish_odom_tf(bool publish_odom,
                                            bool publish_tf) {
  auto pose =
      lidar_in_base_ * pipeline_->currentPose() * lidar_in_base_.inverse();
  // auto vel = pipeline_->currentVel(); // extract also velocity?
  double x = pose(0, 3);
  double y = pose(1, 3);
  double z = pose(2, 3);

  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  q.normalize();

  if (publish_odom) {
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

    odom.child_frame_id = base_frame_id_;

    odom_pub_->publish(odom);
  }
  if (publish_tf) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp_;
    transform.header.frame_id = "odom";
    transform.child_frame_id = base_frame_id_;

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }

  out_dump_ << std::fixed << time_to_double(stamp_) << " " << x << " " << y
            << "\n";

  return;
}

Eigen::Matrix4d parseMatrix(const std::vector<std::vector<double>>& vec) {
  std::vector<double> mat_vec;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      mat_vec.push_back(vec[r][c]);
    }
  }
  return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
      mat_vec.data());
}

RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::Odometry)
