#include "mad_icp_ros_interfaces/msg/mad_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tools/mad_tree.h"

namespace mad_icp_ros_utils {
void serialize(const MADtree& tree, mad_icp_ros_interfaces::msg::MadTree& msg);
}