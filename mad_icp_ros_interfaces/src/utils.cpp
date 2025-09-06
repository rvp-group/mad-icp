#include "mad_icp_ros_utils/utils.h"

#include <queue>

#include "mad_icp_ros_interfaces/msg/mad_node.hpp"

void mad_icp_ros::utils::serialize(const MADtree& tree,
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

    node.bbox.x = root_ptr->bbox_.x();
    node.bbox.y = root_ptr->bbox_.y();
    node.bbox.z = root_ptr->bbox_.z();

    // Must I copy the values?
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(node.eigenvectors.data()) =
        root_ptr->eigenvectors_;

    queue.pop();

    auto this_idx = msg.nodes.size() - 1;

    auto next_free = msg.nodes.size() + queue.size();

    if (left) {
      queue.push({left, this_idx});
      node.left = next_free;
    } else {
      node.left = -1;
    }
    if (right) {
      queue.push({right, this_idx});
      node.right = left ? next_free + 1 : next_free;
    } else {
      node.right = -1;
    }
  }
}

void mad_icp_ros::utils::serialize(const Frame& frame,
                                   mad_icp_ros_interfaces::msg::Frame& msg) {
  // TODO: This is the devil!!! Instead of serializing point by point, just copy the data structure from the msg cloud.
  Eigen::Array3Xd cloud_mat(3, frame.cloud_->size());
  for (size_t i{0}; i < frame.cloud_->size(); ++i) {
    auto& xyz = frame.cloud_->operator[](i);
    cloud_mat.col(i) << xyz.x(), xyz.y(), xyz.z();
  }
  msg.cloud.resize(cloud_mat.size());
  msg.cloud.assign(cloud_mat.data(), cloud_mat.data() + cloud_mat.size());
  msg.cloud_size = frame.cloud_->size();


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

MADtree* deserialize_aux(
    std::vector<mad_icp_ros_interfaces::msg::MadNode>::const_iterator begin,
    std::vector<mad_icp_ros_interfaces::msg::MadNode>::const_iterator it,
    MADtree* parent_ptr) {
  // std::cerr << std::distance(begin, it) << " " << (it->left) << " "
  //<< (it->right) << "\n";
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

  root->parent_ = parent_ptr;

  if (node.left != -1) {
    root->left_ = deserialize_aux(begin, begin + node.left, root);
  }

  if (node.right != -1) {
    root->right_ = deserialize_aux(begin, begin + node.right, root);
  }

  return root;
};

MADtree* mad_icp_ros::utils::deserialize(
    const mad_icp_ros_interfaces::msg::MadTree& msg) {
  if (msg.nodes.empty()) {
    return nullptr;
  }
  // std::cerr << msg.nodes.size() << "\n";
  auto begin = msg.nodes.begin();
  auto out = deserialize_aux(begin, begin, nullptr);
  // std::cerr << "done\n";
  return out;
}

Frame* mad_icp_ros::utils::deserialize(
    const mad_icp_ros_interfaces::msg::Frame& msg) {
  Frame* frame = new Frame();

  Eigen::Array3Xd cloud_mat =
      Eigen::Map<const Eigen::Array3Xd>(msg.cloud.data(), 3, msg.cloud_size);

  frame->cloud_ = new ContainerType;
  for (size_t i{0}; i < msg.cloud_size; ++i) {
    frame->cloud_->emplace_back(cloud_mat.col(i));
  }

  frame->tree_ = deserialize(msg.tree);

  frame->frame_ = msg.seq;
  frame->stamp_ = time_to_double(msg.stamp);

  frame->weight_ = msg.weight;

  Eigen::Quaterniond q(msg.frame_to_map.rotation.w, msg.frame_to_map.rotation.x,
                       msg.frame_to_map.rotation.y,
                       msg.frame_to_map.rotation.z);

  q.normalize();

  Eigen::Vector3d t(msg.frame_to_map.translation.x,
                    msg.frame_to_map.translation.y,
                    msg.frame_to_map.translation.z);

  frame->frame_to_map_ = Eigen::Isometry3d::Identity();
  frame->frame_to_map_.linear() = q.toRotationMatrix();
  frame->frame_to_map_.translation() = t;

  return frame;
}

double mad_icp_ros::utils::time_to_double(
    const builtin_interfaces::msg::Time& t) {
  return t.sec + t.nanosec * 1e-9;
}

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
