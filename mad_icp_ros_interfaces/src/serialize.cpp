#include "mad_icp_ros_interfaces/serialize.h"

#include <queue>

#include "mad_icp_ros_interfaces/msg/mad_node.hpp"

void mad_icp_ros_utils::serialize(const MADtree& tree,
                                  mad_icp_ros_interfaces::msg::MadTree& msg) {
  assert(sizeof(int64_t) == sizeof(MADtree*));

  // visit the tree
  std::queue<std::pair<const MADtree*, size_t>> queue;

  queue.push({&tree, -1});

  while (!queue.empty()) {
    auto root_ptr = queue.front().first;
    MADtree* left = root_ptr->left_;
    MADtree* right = root_ptr->right_;

    // ROS2 idl does not generate field constructors
    // msg.nodes.emplace_back(*root_ptr.first);
    msg.nodes.emplace_back();
    auto& node = msg.nodes.back();

    node.parent = queue.front().second;
    node.num_points = root_ptr->num_points_;
    node.matched = root_ptr->matched_;
    node.mean.x = root_ptr->mean_.x();
    node.mean.y = root_ptr->mean_.y();
    node.mean.z = root_ptr->mean_.z();

    node.bbox.x = root_ptr->mean_.x();
    node.bbox.y = root_ptr->mean_.y();
    node.bbox.z = root_ptr->mean_.z();

    // Must I copy the values?
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(node.eigenvectors.data()) =
        root_ptr->eigenvectors_;

    auto this_idx = msg.nodes.size() - 1;

    if (left) {
      queue.push({left, this_idx});
      node.left = msg.nodes.size();
    }
    if (right) {
      queue.push({right, this_idx});
      node.right = left ? msg.nodes.size() + 1 : msg.nodes.size();
    }
    queue.pop();
  }
}