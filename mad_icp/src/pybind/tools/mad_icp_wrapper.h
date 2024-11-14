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
#include <odometry/mad_icp.h>
#include <sys/time.h>

class MADicpWrapper {
public:
  MADicpWrapper(const int num_threads) : num_threads_(num_threads_) {
    max_parallel_levels_ = static_cast<int>(std::log2(num_threads_));
    omp_set_num_threads(num_threads_);
  }

  void setQueryCloud(ContainerType query, const double b_max, const double b_min) {
    ContainerType* query_ptr = &query;
    query_tree_.reset(
      new MADtree(query_ptr, query_ptr->begin(), query_ptr->end(), b_max, b_min, 0, max_parallel_levels_, nullptr, nullptr));
    query_tree_->getLeafs(std::back_insert_iterator<LeafList>(query_leaves_));
  }

  void setReferenceCloud(ContainerType reference, const double b_max, const double b_min) {
    ContainerType* reference_ptr = &reference;
    ref_b_max_                   = b_max;
    ref_tree_.reset(new MADtree(
      reference_ptr, reference_ptr->begin(), reference_ptr->end(), ref_b_max_, b_min, 0, max_parallel_levels_, nullptr, nullptr));
  }

  inline Eigen::Matrix4d compute(const Eigen::Matrix4d& T,
                                 const size_t max_icp_iterations,
                                 const double rho_ker,
                                 double b_ratio,
                                 const bool print_stats) {
    mad_icp_.reset(new MADicp(ref_b_max_, rho_ker, b_ratio, 1));
    mad_icp_->setMoving(query_leaves_);
    // make initial guess in right format
    Eigen::Isometry3d dX = Eigen::Isometry3d::Identity();
    dX.linear()          = T.block<3, 3>(0, 0);
    dX.translation()     = T.block<3, 1>(0, 3);
    mad_icp_->init(dX);

    float icp_time = 0;
    struct timeval icp_start, icp_end, icp_delta;
    gettimeofday(&icp_start, nullptr);

    // icp loop
    for (size_t icp_iteration = 0; icp_iteration < max_icp_iterations; ++icp_iteration) {
      if (icp_iteration == max_icp_iterations - 1) {
        for (MADtree* l : query_leaves_) {
          l->matched_ = false;
        }
      }
      mad_icp_->resetAdders();
      mad_icp_->update(ref_tree_.get());
      mad_icp_->updateState();
    }

    gettimeofday(&icp_end, nullptr);
    timersub(&icp_end, &icp_start, &icp_delta);
    icp_time = float(icp_delta.tv_sec) * 1000. + 1e-3 * icp_delta.tv_usec;

    if (print_stats) {
      int matched_leaves = 0;
      for (MADtree* l : query_leaves_) {
        if (l->matched_) {
          matched_leaves++;
        }
      }
      const double inliers_ratio = double(matched_leaves) / double(query_leaves_.size());
      std::cout << "MADicp|compute time " << icp_time << " [ms] " << std::endl;
      std::cout << "MADicp|inliers ratio " << inliers_ratio << std::endl;
      std::cout << "--MADicp|matched leaves " << matched_leaves << std::endl;
      std::cout << "--MADicp|total num leaves " << query_leaves_.size() << std::endl;
    }

    return mad_icp_->X_.matrix();
  }

protected:
  std::unique_ptr<MADicp> mad_icp_     = nullptr;
  std::unique_ptr<MADtree> ref_tree_   = nullptr;
  std::unique_ptr<MADtree> query_tree_ = nullptr;
  LeafList query_leaves_;
  double ref_b_max_;
  int max_parallel_levels_;
  int num_threads_;
};
