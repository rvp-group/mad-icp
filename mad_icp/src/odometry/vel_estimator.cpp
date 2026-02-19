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
  
  // Linear error: predicted displacement vs actual displacement
  e.block<3, 1>(0, 0) = delta_t * X_.block<3, 1>(0, 0) - T_now_to_prev.translation();

  // Angular error: use axis-angle representation (logMapSO3) instead of Euler angles
  // This avoids gimbal lock singularities and provides a continuous error surface
  const Eigen::Vector3d angles = logMapSO3(T_now_to_prev.linear());
  e.block<3, 1>(3, 0) = delta_t * X_.block<3, 1>(3, 0) - angles;

  // Jacobian: diagonal structure since both linear and angular velocities
  // have linear relationship with their respective errors (for local linearization)
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
  const size_t n = odometry_.size();

  for (size_t i = 0; i < n - 1; ++i) {
    const Eigen::Isometry3d T_prev = odometry_[i];
    // Time difference from frame i to the current frame (last in window)
    const double delta_t = (n - 1 - i) * ts_;
    // Weight: linearly increases with frame recency
    // Newer frames (higher i) get higher weight to prioritize recent motion
    const double weight = double(i + 1) / double(n);

    update(T_now, T_prev, delta_t, weight);
  }

  Vector6d dx = H_adder_.ldlt().solve(-b_adder_);
  X_ += dx;
}
