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

void mad_icp_ros_utils::serialize(const Frame& frame,
                                  mad_icp_ros_interfaces::msg::Frame& msg) {
  for (size_t i{0}; i < frame.cloud_->size(); ++i) {
    auto& xyz = frame.cloud_->operator[](i);
    msg.cloud.emplace_back();
    msg.cloud.back().x = xyz.x();
    msg.cloud.back().y = xyz.y();
    msg.cloud.back().z = xyz.z();
  }

  serialize(*frame.tree_, msg.tree);

  msg.seq = frame.frame_;
  msg.stamp.sec = static_cast<int32_t>(frame.stamp_);
  msg.stamp.nanosec =
      static_cast<uint32_t>((frame.stamp_ - msg.stamp.sec) * 1e9);

  msg.weight = frame.weight_;

  auto& t = frame.frame_to_map_.translation();

  msg.frame_to_map.translation.x = t.x();
  msg.frame_to_map.translation.y = t.y();
  msg.frame_to_map.translation.z = t.z();

  Eigen::Quaterniond q(frame.frame_to_map_.linear());
  msg.frame_to_map.rotation.x = q.x();
  msg.frame_to_map.rotation.y = q.y();
  msg.frame_to_map.rotation.z = q.z();
  msg.frame_to_map.rotation.w = q.w();

  return;
}

MADtree* mad_icp_ros_utils::deserialize(
    const mad_icp_ros_interfaces::msg::MadTree& msg) {
  if (msg.nodes.empty()) {
    return nullptr;
  }

  auto begin = msg.nodes.begin();

  std::function<MADtree*(
      std::vector<mad_icp_ros_interfaces::msg::MadNode>::const_iterator)>
      aux;

  aux = [&aux, begin](
            std::vector<mad_icp_ros_interfaces::msg::MadNode>::const_iterator
                it) {
    MADtree* root = new MADtree();
    auto& node = *it;

    root->mean_.x() = node.mean.x;
    root->mean_.y() = node.mean.y;
    root->mean_.z() = node.mean.z;

    root->bbox_.x() = node.bbox.x;
    root->bbox_.y() = node.bbox.y;
    root->bbox_.z() = node.bbox.z;

    root->eigenvectors_ =
        Eigen::Map<const Eigen::Matrix<double, 3, 3>>(node.eigenvectors.data());

    root->matched_ = node.matched;
    root->num_points_ = node.num_points;

    if (node.left != -1) {
      root->left_ = aux(begin + node.left);
    }

    if (node.right != -1) {
      root->right_ = aux(begin + node.right);
    }

    return root;
  };

  return nullptr;
}