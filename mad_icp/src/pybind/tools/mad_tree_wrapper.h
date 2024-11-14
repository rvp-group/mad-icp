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

class MADtreeWrapper {
public:
  void build(ContainerType vec, const double b_max, const double b_min, const int max_parallel_level) {
    ContainerType* vec_add = &vec;
    mad_tree_.reset(
      new MADtree(vec_add, vec_add->begin(), vec_add->end(), b_max, b_min, 0, max_parallel_level, nullptr, nullptr));
  }

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

protected:
  std::unique_ptr<MADtree> mad_tree_ = nullptr;
};
