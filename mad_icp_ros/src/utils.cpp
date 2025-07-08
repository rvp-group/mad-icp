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

// Do not use PointCloud2Iterators, much overhead
// void mad_icp_ros::utils::filter_pc(
//     const sensor_msgs::msg::PointCloud2::SharedPtr msg, double min_range,
//     double max_range, double intensity_thr, ContainerType& container) {
//   // mad icp uses unordered clouds
//   auto height = msg->height;
//   auto width = msg->width;

//   // Convert the PointCloud2 message to mad_icp's ContainerType (std::vector
//   // of 3d points)
//   sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
//   sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
//   sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
//   sensor_msgs::PointCloud2ConstIterator<float> iter_int(*msg, "intensity");

//   container.clear();
//   container.reserve(msg->height * msg->width);

//   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_int) {
//     auto point = Eigen::Vector3d{*iter_x, *iter_y, *iter_z};
//     if (std::isnan(point.x()) || std::isnan(point.y()) ||
//         std::isnan(point.z()) || *iter_int < intensity_thr ||
//         point.norm() < min_range || point.norm() > max_range)
//       continue;
//     container.emplace_back(point);
//   }
// }

void mad_icp_ros::utils::filter_pc(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, double min_range,
    double max_range, double intensity_thr, ContainerType& container) {
  container.clear();
  container.reserve(msg->height * msg->width);

  const uint8_t* data_ptr = msg->data.data();
  size_t num_points = msg->width * msg->height;

  // Find field offsets
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_intensity = -1;

  for (const auto& field : msg->fields) {
    if (field.name == "x") offset_x = field.offset;
    if (field.name == "y") offset_y = field.offset;
    if (field.name == "z") offset_z = field.offset;
    if (field.name == "intensity") offset_intensity = field.offset;
  }

  if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_intensity < 0) {
    throw std::runtime_error("PointCloud2 missing required fields");
  }

  for (size_t i = 0; i < num_points; ++i, data_ptr += msg->point_step) {
    float x = *reinterpret_cast<const float*>(data_ptr + offset_x);
    float y = *reinterpret_cast<const float*>(data_ptr + offset_y);
    float z = *reinterpret_cast<const float*>(data_ptr + offset_z);
    float intensity =
        *reinterpret_cast<const float*>(data_ptr + offset_intensity);

    Eigen::Vector3d point(x, y, z);

    if (!std::isfinite(x + y + z)) continue;
    if (intensity < intensity_thr) continue;

    double sq_norm = point.squaredNorm();
    if (sq_norm < min_range * min_range || sq_norm > max_range * max_range)
      continue;

    container.emplace_back(point);
  }
}
