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

#include <tools/lie_algebra.h>
#include <tools/utils.h>

#include <Eigen/Eigenvalues>
#include <fstream>
#include <iostream>
#include <sstream>

struct VelEstimator {
  VelEstimator(double sensor_hz);

  void setOdometry(const std::vector<Eigen::Isometry3d>& odometry);

  void init(const Vector6d& velocity = Vector6d::Zero());
  void oneRound();

  void errorAndJacobian(Vector6d& e,
                        Matrix6d& J,
                        const Eigen::Isometry3d& T_now,
                        const Eigen::Isometry3d& T_prev,
                        const double delta_t);
  void update(const Eigen::Isometry3d& T_now, const Eigen::Isometry3d& T_prev, const double delta_t, const double scale);

  Matrix6d H_adder_;
  Vector6d X_, b_adder_;
  std::vector<Eigen::Isometry3d> odometry_;
  double ts_;
};
