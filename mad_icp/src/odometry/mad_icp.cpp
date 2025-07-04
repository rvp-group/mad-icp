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

#include "mad_icp.h"

MADicp::MADicp(double min_ball, double rho_ker, double b_ratio, int num_threads) :
  min_ball_(min_ball), rho_ker_(sqrt(rho_ker)), b_ratio_(b_ratio), num_threads_(num_threads) {
  X_.setIdentity();
  H_adder_.setZero();
  b_adder_.setZero();
  chi_adder_ = 0;

  H_adders_   = std::vector<Matrix6d>(num_threads);
  b_adders_   = std::vector<Vector6d>(num_threads);
  chi_adders_ = std::vector<double>(num_threads);
}

void MADicp::resetAdders() {
  H_adder_.setZero();
  b_adder_.setZero();
  chi_adder_ = 0;

  for (size_t i = 0; i < num_threads_; ++i) {
    H_adders_[i].setZero();
    b_adders_[i].setZero();
    chi_adders_[i] = 0;
  }
}

void MADicp::setMoving(const LeafList& moving_leaves) {
  moving_leaves_ = moving_leaves;
}

void MADicp::init(const Eigen::Isometry3d& moving_in_fixed) {
  X_ = moving_in_fixed;
}

void MADicp::errorAndJacobian(double& e,
                              JacobianMatrixType& J,
                              const MADtree& fixed,
                              const MADtree& moving,
                              const Eigen::Vector3d& moving_transformed) const {
  const auto& fixed_point  = fixed.mean_;
  const auto& fixed_normal = fixed.eigenvectors_.col(0);
  const auto& moving_point = moving.mean_;
  const Eigen::Matrix3d& R = X_.linear();

  e                   = (moving_transformed - fixed_point).dot(fixed_normal);
  J.block<1, 3>(0, 0) = fixed_normal.transpose() * R;
  J.block<1, 3>(0, 3) = -J.block<1, 3>(0, 0) * skew(moving_point);
}

void MADicp::update(const MADtree* fixed_tree) {
  const int thread_id = omp_get_thread_num();

  for (auto& moving : moving_leaves_) {
    const Eigen::Vector3d ml = X_ * moving->mean_;
    const auto f             = fixed_tree->bestMatchingLeafFast(ml);

    const double src_ball = min_ball_ + b_ratio_ * moving->mean_.norm();
    if ((ml - f->mean_).norm() > src_ball)
      continue;

    moving->matched_ = true;

    JacobianMatrixType J;
    double e;

    errorAndJacobian(e, J, *f, *moving, ml);

    double scale     = 1.;
    const double chi = abs(e);

    if (chi > rho_ker_) {
      scale = rho_ker_ / chi;
    }
    const double w = 1. - f->bbox_(0) / min_ball_;
    scale *= w * w;

    H_adders_[thread_id] += scale * J.transpose() * J;
    b_adders_[thread_id] += scale * J.transpose() * e;
    chi_adders_[thread_id] = chi;
  }
}

void MADicp::updateState() {
  for (size_t i = 0; i < num_threads_; ++i) {
    H_adder_ += H_adders_[i];
    b_adder_ += b_adders_[i];
    chi_adder_ += chi_adders_[i];
  }

  Vector6d dx          = H_adder_.ldlt().solve(-b_adder_);
  Eigen::Isometry3d dX = Eigen::Isometry3d::Identity();

  dX.linear()      = expMapSO3(dx.tail(3));
  dX.translation() = dx.head(3);
  X_               = X_ * dX;
}