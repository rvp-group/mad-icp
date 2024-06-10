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

#include "vel_estimator.h"
#include <tools/constants.h>

VelEstimator::VelEstimator(double sensor_hz) {
  X_.setZero();
  ts_ = 1. / sensor_hz;
}

void VelEstimator::init(const Vector6d& velocity) {
  X_ = velocity;
}

void VelEstimator::setOdometry(const std::vector<Eigen::Isometry3d>& odometry) {
  odometry_ = odometry;
}

void VelEstimator::errorAndJacobian(Vector6d& e,
                                    Matrix6d& J,
                                    const Eigen::Isometry3d& T_now,
                                    const Eigen::Isometry3d& T_prev,
                                    const double delta_t) {
  Eigen::Isometry3d T_now_to_prev = T_prev.inverse() * T_now;
  e.block<3, 1>(0, 0)             = delta_t * X_.block<3, 1>(0, 0) - T_now_to_prev.translation();

  Eigen::Vector3d angles;
  angles(0)           = atan2(-T_now_to_prev.linear()(1, 2), T_now_to_prev.linear()(2, 2));
  angles(1)           = asin(T_now_to_prev.linear()(0, 2));
  angles(2)           = atan2(-T_now_to_prev.linear()(0, 1), T_now_to_prev.linear()(0, 0));
  e.block<3, 1>(3, 0) = delta_t * X_.block<3, 1>(3, 0) - angles;

  J = Matrix6d::Identity() * delta_t;
}

void VelEstimator::update(const Eigen::Isometry3d& T_now,
                          const Eigen::Isometry3d& T_prev,
                          const double delta_t,
                          const double weight) {
  Vector6d e;
  Matrix6d J;
  errorAndJacobian(e, J, T_now, T_prev, delta_t);

  double scale      = 1.;
  const double chi2 = e.squaredNorm();
  const double chi  = sqrt(chi2);
  if (chi > E_THRESHOLD_VEL) {
    scale = E_THRESHOLD_VEL / chi;
  }

  H_adder_ += scale * weight * J.transpose() * J;
  b_adder_ += scale * weight * J.transpose() * e;
}

void VelEstimator::oneRound() {
  H_adder_.setZero();
  b_adder_.setZero();

  const Eigen::Isometry3d T_now = odometry_.back();

  for (size_t i = 0; i < odometry_.size() - 1; ++i) {
    const Eigen::Isometry3d T_prev = odometry_[i];
    const double delta_t           = (odometry_.size() - 1 - i) * ts_;
    const double weight            = 1.f - double(odometry_.size() - 2 - i) / double(odometry_.size() - 1);

    update(T_now, T_prev, delta_t, weight);
  }

  Vector6d dx = H_adder_.ldlt().solve(-b_adder_);
  X_ += dx;
}
