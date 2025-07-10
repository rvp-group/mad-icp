#include "mad_icp_ros_interfaces/msg/frame.hpp"
#include "mad_icp_ros_interfaces/msg/mad_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tools/frame.h"
#include "tools/mad_tree.h"

namespace mad_icp_ros_utils {
void serialize(const MADtree& tree, mad_icp_ros_interfaces::msg::MadTree& msg);
void serialize(const Frame& frame, mad_icp_ros_interfaces::msg::Frame& msg);

MADtree* deserialize(const mad_icp_ros_interfaces::msg::MadTree& msg);
Frame* deserialize(const mad_icp_ros_interfaces::msg::Frame& msg);
}  // namespace mad_icp_ros_utils