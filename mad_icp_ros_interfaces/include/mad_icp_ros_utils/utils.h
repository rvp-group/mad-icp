#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mad_icp_ros_interfaces/msg/frame.hpp"
#include "mad_icp_ros_interfaces/msg/mad_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tools/frame.h"
#include "tools/mad_tree.h"

namespace mad_icp_ros {
namespace utils {
void serialize(const MADtree& tree, mad_icp_ros_interfaces::msg::MadTree& msg);
void serialize(const Frame& frame, mad_icp_ros_interfaces::msg::Frame& msg);

MADtree* deserialize(const mad_icp_ros_interfaces::msg::MadTree& msg);
Frame* deserialize(const mad_icp_ros_interfaces::msg::Frame& msg);

double time_to_double(const builtin_interfaces::msg::Time& t);

// filters a PointCloud2 and puts it into a ContainerType
void filter_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
               double min_range, double max_range, double intensity_thr,
               ContainerType& container);
}  // namespace utils
}  // namespace mad_icp_ros