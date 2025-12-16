// Copyright 2024 R(obots) V(ision) and P(erception) group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "mad_icp_ros/pose_index.h"
#include "mad_icp_ros/binary_tree_io.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace mad_icp_ros {

PoseIndex::PoseIndex() = default;
PoseIndex::~PoseIndex() = default;

bool PoseIndex::loadMap(const std::string& map_dir) {
  map_dir_ = map_dir;

  // Clear any existing data
  cloud_.poses.clear();
  trees_.clear();
  index_.reset();

  // Open manifest file
  std::string manifest_path = map_dir + "/manifest.txt";
  std::ifstream file(manifest_path);
  if (!file) {
    std::cerr << "Failed to open manifest file: " << manifest_path << std::endl;
    return false;
  }

  std::string line;
  uint32_t pose_id = 0;
  size_t load_errors = 0;

  while (std::getline(file, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') {
      continue;
    }

    MapPose mp;
    mp.pose_id = pose_id;

    std::istringstream iss(line);
    double tx, ty, tz, qx, qy, qz, qw;

    if (!(iss >> mp.timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> mp.tree_filepath)) {
      std::cerr << "Failed to parse manifest line: " << line << std::endl;
      continue;
    }

    // Construct pose from translation and quaternion
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    mp.pose.setIdentity();
    mp.pose.linear() = q.toRotationMatrix();
    mp.pose.translation() = Eigen::Vector3d(tx, ty, tz);

    cloud_.poses.push_back(mp);

    // Load the MADtree
    std::string tree_path = map_dir + "/" + mp.tree_filepath;
    auto tree = deserializeTree(tree_path);
    if (tree) {
      trees_[pose_id] = std::move(tree);
    } else {
      std::cerr << "Failed to load tree for pose " << pose_id << ": " << tree_path << std::endl;
      load_errors++;
    }

    pose_id++;
  }

  if (cloud_.poses.empty()) {
    std::cerr << "No poses loaded from manifest" << std::endl;
    return false;
  }

  std::cout << "Loaded " << cloud_.poses.size() << " poses from manifest" << std::endl;
  std::cout << "Loaded " << trees_.size() << " MADtrees" << std::endl;
  if (load_errors > 0) {
    std::cerr << "Warning: " << load_errors << " trees failed to load" << std::endl;
  }

  // Build k-d tree index
  index_ = std::make_unique<PoseKDTree>(
      3, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf size */));
  index_->buildIndex();

  std::cout << "Built spatial index for pose lookup" << std::endl;

  return true;
}

std::vector<uint32_t> PoseIndex::findNearestPoses(const Eigen::Vector3d& position,
                                                   size_t n) const {
  if (!index_ || cloud_.poses.empty()) {
    return {};
  }

  // Clamp n to available poses
  n = std::min(n, cloud_.poses.size());

  // Prepare query
  std::vector<uint32_t> ret_indices(n);
  std::vector<double> out_dists_sqr(n);

  // nanoflann wants size_t indices
  std::vector<size_t> indices(n);

  nanoflann::KNNResultSet<double> result_set(n);
  result_set.init(&indices[0], &out_dists_sqr[0]);

  index_->findNeighbors(result_set, position.data(), nanoflann::SearchParameters());

  // Convert to uint32_t
  for (size_t i = 0; i < n; ++i) {
    ret_indices[i] = static_cast<uint32_t>(indices[i]);
  }

  return ret_indices;
}

const MapPose& PoseIndex::getPose(uint32_t pose_id) const {
  return cloud_.poses.at(pose_id);
}

MADtree* PoseIndex::getTree(uint32_t pose_id) const {
  auto it = trees_.find(pose_id);
  if (it != trees_.end()) {
    return it->second.get();
  }
  return nullptr;
}

std::string PoseIndex::getTreePath(uint32_t pose_id) const {
  if (pose_id >= cloud_.poses.size()) {
    return "";
  }
  return map_dir_ + "/" + cloud_.poses[pose_id].tree_filepath;
}

}  // namespace mad_icp_ros
