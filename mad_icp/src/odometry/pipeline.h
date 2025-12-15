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

#include "mad_icp.h"
#include "vel_estimator.h"
#include <tools/frame.h>
#include <tools/lie_algebra.h>
#include <tools/mad_tree.h>

#include <Eigen/Core>
#include <deque>
#include <sys/time.h>

#include <algorithm>
#include <random>
#include <vector>

class Pipeline {
public:
  Pipeline(double sensor_hz,
           bool deskew,
           double b_max,
           double rho_ker,
           double p_th,
           double b_min,
           double b_ratio,
           int num_keyframes,
           int num_threads,
           bool realtime);
  ~Pipeline();

  // clang-format off
  const Eigen::Matrix<double, 4, 4> currentPose() const { return frame_to_map_.matrix(); }
  const std::vector<Eigen::Isometry3d>& trajectory() const { return trajectory_; }
  const Eigen::Matrix<double, 4, 4> keyframePose() const { return keyframe_to_map_.matrix(); }
  const bool isInitialized() const { return is_initialized_; }
  const size_t currentID() const { return seq_; }
  const size_t keyframeID() const { return seq_keyframe_; }
  // clang-format on

  const bool isMapUpdated();
  const ContainerType currentLeaves();
  const ContainerType modelLeaves();
  void compute(const double& curr_stamp, ContainerType curr_cloud_mem);

protected:
  void initialize(const double& curr_stamp, const ContainerTypePtr curr_cloud);
  void deskew(const ContainerTypePtr& curr_cloud, const Eigen::Isometry3d& T_prev, const Eigen::Isometry3d& T_now);

  MADicp icp_;
  VelEstimator vel_estimator_;

  Eigen::Isometry3d frame_to_map_;
  Eigen::Isometry3d keyframe_to_map_;

  Vector6d current_velocity_;
  Vector6d previous_velocity_;  // For trapezoidal integration

  std::deque<Frame*> keyframes_;
  std::deque<Frame*> frames_;
  std::vector<Eigen::Isometry3d> trajectory_;

  MADtree* current_tree_;
  LeafList model_leaves_, current_leaves_;

  // start some params
  bool deskew_, realtime_;
  int num_keyframes_, num_threads_, max_parallel_levels_;
  double sensor_hz_, b_max_, p_th_, b_min_;
  double delta_chi_threshold_{1e-6};
  // end some params

  size_t seq_;
  size_t seq_keyframe_;
  bool is_initialized_;
  bool is_map_updated_;
  float loop_time;
};
