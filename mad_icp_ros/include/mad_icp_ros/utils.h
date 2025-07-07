#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tools/mad_tree.h"

namespace mad_icp_ros {
namespace utils {

double time_to_double(const builtin_interfaces::msg::Time& t);

Eigen::Matrix4d parse_isometry(std::vector<double> vec);

// filters a PointCloud2 and puts it into a ContainerType
void filter_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
               double min_range, double max_range, double intensity_thr,
               ContainerType& container);

}  // namespace utils

}  // namespace mad_icp_ros