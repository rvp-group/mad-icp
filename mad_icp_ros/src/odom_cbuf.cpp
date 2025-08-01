#include "mad_icp_ros/odom_cbuf.h"

#include <iostream>

void mad_icp_ros::OdomCircularBuffer::insert(const ItemT& item) {
  data_[end_] = item;
  end_ = incr(end_);

  if (full_) {
    start_ = incr(start_);
  } else if (end_ == start_) {
    full_ = true;
  }
}

size_t mad_icp_ros::OdomCircularBuffer::find(TimeT t) const {
  std::pair<ItemT, ItemT> out;
  if (!full_ && end_ - start_ < 2) {
    return std::numeric_limits<size_t>::quiet_NaN();
  }

  // I could do binary search but
  // I just start from the end and go backwards
  size_t i = decr(end_);
  if (t >= data_[i].first || t <= data_[start_].first) {
    return std::numeric_limits<size_t>::quiet_NaN();
  }
  while (i != start_) {
    // assume that t is less than the last element, I have to check the previous
    // one
    i = decr(i);
    if (t >= data_[i].first) {
      // found it
      return i;
    }
  }
  // not found
  return std::numeric_limits<size_t>::quiet_NaN();
}

mad_icp_ros::OdomCircularBuffer::PoseT mad_icp_ros::OdomCircularBuffer::interp(
    const mad_icp_ros::OdomCircularBuffer::PoseT& A,
    const mad_icp_ros::OdomCircularBuffer::PoseT& B, double lambda) const {
  Eigen::Vector3d translated =
      (1 - lambda) * A.translation() + lambda * B.translation();

  Eigen::Quaterniond quatA(A.rotation());
  Eigen::Quaterniond quatB(B.rotation());

  if (quatA.dot(quatB) < 0.0) {
    quatB = Eigen::Quaterniond(-1.0 * quatB.coeffs());
  }

  // Spherical interpolation
  Eigen::Quaterniond quatInterp = quatA.slerp(lambda, quatB);

  // Combine the interpolated translation and rotation into an Isometry3d
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = translated;
  result.linear() = quatInterp.toRotationMatrix();

  return result;
}

mad_icp_ros::OdomCircularBuffer::ItemT mad_icp_ros::OdomCircularBuffer::query(
    TimeT t) const {
  size_t closest = find(t);
  if (std::isnan(closest)) {
    return invalid_item_;
  }

  auto& i1 = data_[closest];
  auto& i2 = data_[incr(closest)];

  double lambda = (t - i1.first) / (i2.first - i1.first);

  return std::make_pair(t, interp(i1.second, i2.second, lambda));
}