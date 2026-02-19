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

#include "utils.h"

#include <Eigen/Core>
#include <atomic>
#include <iostream>
#include <list>
#include <memory>

class MADtree;

// some using
using ContainerType    = std::vector<Eigen::Vector3d>;
using ContainerTypePtr = ContainerType*;
using IteratorType     = typename ContainerType::iterator;
using LeafList         = std::vector<MADtree*>;

struct MADtree {
  MADtree(const ContainerTypePtr vec,
          const IteratorType begin,
          const IteratorType end,
          const double b_max,
          const double b_min,
          const int level,
          const int max_parallel_level,
          MADtree* parent,
          MADtree* plane_predecessor);

  inline ~MADtree() {
    if (left_)
      delete left_;
    if (right_)
      delete right_;
  }

  MADtree(const MADtree& other) = delete;
  bool operator==(const MADtree& other) const;

  void applyTransform(const Eigen::Matrix3d& r, const Eigen::Vector3d& t);

  const MADtree* bestMatchingLeafFast(const Eigen::Vector3d& query) const;

  void build(const ContainerTypePtr vec,
             const IteratorType begin,
             const IteratorType end,
             const double b_max,
             const double b_min,
             const int level,
             const int max_parallel_level,
             MADtree* parent,
             MADtree* plane_predecessor);

  static MADtree* makeSubtree(const ContainerTypePtr vec,
                              const IteratorType begin,
                              const IteratorType end,
                              const double b_max,
                              const double b_min,
                              const int level,
                              const int max_parallel_level,
                              MADtree* parent,
                              MADtree* plane_predecessor);

  void getLeafs(std::back_insert_iterator<std::vector<MADtree*>> it);

  int num_points_;
  bool matched_;
  MADtree* left_   = nullptr;
  MADtree* right_  = nullptr;
  MADtree* parent_ = nullptr;
  Eigen::Vector3d mean_;
  Eigen::Vector3d bbox_;
  Eigen::Matrix3d eigenvectors_;

  // Default constructor for deserialization
  MADtree() : num_points_(0), matched_(false), left_(nullptr), right_(nullptr), parent_(nullptr) {};
};
