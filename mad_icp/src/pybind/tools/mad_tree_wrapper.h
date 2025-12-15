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

#pragma once

#include "../eigen_stl_bindings.h"
#include <tools/mad_tree.h>
#include <fstream>
#include <queue>

class MADtreeWrapper {
public:
  void build(ContainerType vec, const double b_max, const double b_min, const int max_parallel_level) {
    stored_vec_ = std::move(vec);
    mad_tree_.reset(
      new MADtree(&stored_vec_, stored_vec_.begin(), stored_vec_.end(), b_max, b_min, 0, max_parallel_level, nullptr, nullptr));
  }

  void applyTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    if (mad_tree_) {
      mad_tree_->applyTransform(R, t);
    }
  }

  void serialize(const std::string& filepath) const {
    if (!mad_tree_) {
      throw std::runtime_error("MADtree not built yet");
    }

    std::ofstream file(filepath, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Failed to open file for writing: " + filepath);
    }

    // Count nodes first
    uint32_t num_nodes = countNodes(mad_tree_.get());

    // Write header
    const char magic[4] = {'M', 'A', 'D', 'T'};
    uint32_t version = 1;
    uint32_t reserved = 0;

    file.write(magic, 4);
    file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(num_nodes));
    file.write(reinterpret_cast<const char*>(&reserved), sizeof(reserved));

    // Write nodes in breadth-first order
    std::queue<const MADtree*> queue;
    queue.push(mad_tree_.get());

    while (!queue.empty()) {
      const MADtree* node = queue.front();
      queue.pop();

      // Write node data
      file.write(reinterpret_cast<const char*>(node->mean_.data()), 3 * sizeof(double));
      file.write(reinterpret_cast<const char*>(node->bbox_.data()), 3 * sizeof(double));
      file.write(reinterpret_cast<const char*>(node->eigenvectors_.data()), 9 * sizeof(double));

      int32_t num_points = node->num_points_;
      file.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

      uint8_t flags = 0;
      if (node->left_) flags |= 1;
      if (node->right_) flags |= 2;
      file.write(reinterpret_cast<const char*>(&flags), sizeof(flags));

      uint8_t padding = 0;
      file.write(reinterpret_cast<const char*>(&padding), sizeof(padding));

      // Enqueue children
      if (node->left_) queue.push(node->left_);
      if (node->right_) queue.push(node->right_);
    }

    file.close();
  }

  size_t getNumNodes() const {
    return mad_tree_ ? countNodes(mad_tree_.get()) : 0;
  }

private:
  static uint32_t countNodes(const MADtree* node) {
    if (!node) return 0;
    return 1 + countNodes(node->left_) + countNodes(node->right_);
  }

public:

  std::pair<Eigen::Vector3d, Eigen::Vector3d> search(const Eigen::Vector3d& query) {
    const MADtree* match_leaf = mad_tree_->bestMatchingLeafFast(query);
    // this is the median point and normal calculated with PCA
    return std::make_pair(match_leaf->mean_, match_leaf->eigenvectors_.col(0));
  }

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> searchCloud(const ContainerType& query_cloud) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches(query_cloud.size());
    int idx = 0;
    for (const auto& query : query_cloud) {
      const MADtree* match_leaf = mad_tree_->bestMatchingLeafFast(query);
      matches[idx++]            = std::make_pair(match_leaf->mean_, match_leaf->eigenvectors_.col(0));
    }
    return matches;
  }

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>> searchCloudDist(const ContainerType& query_cloud) {
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>> matches(query_cloud.size());
    int idx = 0;
    for (const auto& query : query_cloud) {
      const MADtree* match_leaf = mad_tree_->bestMatchingLeafFast(query);
      const double dist         = (query - match_leaf->mean_).norm();
      matches[idx++]            = std::make_tuple(match_leaf->mean_, match_leaf->eigenvectors_.col(0), dist);
    }
    return matches;
  }

protected:
  std::unique_ptr<MADtree> mad_tree_ = nullptr;
  ContainerType stored_vec_;
};
