#include "mad_icp_ros/utils.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

double mad_icp_ros::utils::time_to_double(
    const builtin_interfaces::msg::Time& t) {
  return t.sec + t.nanosec * 1e-9;
}

Eigen::Matrix4d mad_icp_ros::utils::parse_isometry(
    std::vector<double> mat_vec) {
  return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
      mat_vec.data());
}

void mad_icp_ros::utils::filter_pc(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, double min_range,
    double max_range, double intensity_thr, ContainerType& container) {
  // mad icp uses unordered clouds
  auto height = msg->height;
  auto width = msg->width;

  // Convert the PointCloud2 message to mad_icp's ContainerType (std::vector
  // of 3d points)
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_int(*msg, "intensity");

  container.clear();
  container.reserve(msg->height * msg->width);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_int) {
    auto point = Eigen::Vector3d{*iter_x, *iter_y, *iter_z};
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z()) || *iter_int < intensity_thr ||
        point.norm() < min_range || point.norm() > max_range)
      continue;
    container.emplace_back(point);
  }
}
