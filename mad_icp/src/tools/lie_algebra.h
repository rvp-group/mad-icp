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

#include <Eigen/Core>

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return S;
}

inline Eigen::Matrix3d expMapSO3(const Eigen::Vector3d& omega) {
  Eigen::Matrix3d R;
  const double theta_square = omega.dot(omega);
  if (theta_square < 1e-8) {
    R = Eigen::Matrix3d::Identity() + skew(omega);
    return R;
  }
  const double theta        = sqrt(theta_square);
  const Eigen::Matrix3d W   = skew(omega);
  const Eigen::Matrix3d K   = W / theta;
  const double one_minus_cos = 2.0 * sin(theta / 2.0) * sin(theta / 2.0);
  R = Eigen::Matrix3d::Identity() + sin(theta) * K + one_minus_cos * K * K;
  return R;
}

inline Eigen::Vector3d logMapSO3(const Eigen::Matrix3d& R) {
  const double &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const double &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const double &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // get trace(R)
  const double tr = R.trace();
  const double pi(M_PI);
  const double two(2);

  Eigen::Vector3d omega;
  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-10) {
    if (abs(R33 + 1.0) > 1e-5) {
      omega = (pi / sqrt(two + two * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
    } else if (abs(R22 + 1.0) > 1e-5) {
      omega = (pi / sqrt(two + two * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
    } else {
      omega = (pi / sqrt(two + two * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
    }
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / two);
      magnitude    = theta / (two * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      magnitude = 0.5 - tr_3 * tr_3 / 12.0;
    }
    omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
  }
  return omega;
}