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

#include <tools/frame.h>
#include <tools/lie_algebra.h>
#include <tools/mad_tree.h>

#include <Eigen/Eigenvalues>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <sstream>

class MADicp {
public:
  using JacobianMatrixType = Eigen::Matrix<double, 1, 6>;

  // MADicp();

  MADicp(double min_ball, double rho_ker, double b_ratio, int num_threads);

  void resetAdders();

  void setMoving(const LeafList& moving_leaves);

  void init(const Eigen::Isometry3d& moving_in_fixed = Eigen::Isometry3d::Identity());

  void oneRound();

  void update(const MADtree* fixed_tree);

  void updateState();

  void errorAndJacobian(double& e,
                        JacobianMatrixType& J,
                        const MADtree& fixed,
                        const MADtree& moving,
                        const Eigen::Vector3d& moving_transformed) const;

  Eigen::Isometry3d X_;

  Matrix6d H_adder_;
  Vector6d b_adder_;

  LeafList moving_leaves_;

  std::vector<Matrix6d> H_adders_;
  std::vector<Vector6d> b_adders_;

  double rho_ker_;
  double min_ball_;
  double b_ratio_;
  int num_threads_;
};
