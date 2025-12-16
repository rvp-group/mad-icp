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

#include <tools/mad_tree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <nanoflann.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace mad_icp_ros {

/**
 * @brief Represents a map pose with its associated MADtree file
 */
struct MapPose {
  uint32_t pose_id;
  double timestamp;
  Eigen::Isometry3d pose;
  std::string tree_filepath;  // Relative path within map directory
};

/**
 * @brief Point cloud adaptor for nanoflann k-d tree
 */
struct PoseCloud {
  std::vector<MapPose> poses;

  inline size_t kdtree_get_point_count() const { return poses.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return poses[idx].pose.translation()(dim);
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;  // Optional bounding box not provided
  }
};

// Type aliases for nanoflann k-d tree
using PoseKDTree =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PoseCloud>,
                                        PoseCloud,
                                        3 /* dims */>;

/**
 * @brief Spatial index for map poses with lazy-loaded MADtrees
 *
 * Provides O(log N) nearest neighbor search for pose positions using nanoflann.
 * All MADtrees are loaded at startup into a hash table for fast access.
 */
class PoseIndex {
public:
  PoseIndex();
  ~PoseIndex();

  /**
   * @brief Load manifest file and all associated MADtrees
   *
   * Manifest format (per line):
   * timestamp tx ty tz qx qy qz qw tree_relative_path
   *
   * @param map_dir Path to directory containing manifest.txt and trees/
   * @return true on success, false on failure
   */
  bool loadMap(const std::string& map_dir);

  /**
   * @brief Find N nearest poses to a query position
   *
   * @param position 3D query position
   * @param n Number of nearest neighbors to find
   * @return Vector of pose IDs sorted by distance (closest first)
   */
  std::vector<uint32_t> findNearestPoses(const Eigen::Vector3d& position, size_t n) const;

  /**
   * @brief Get MapPose by ID
   *
   * @param pose_id Pose ID (0-indexed)
   * @return Reference to MapPose
   * @throws std::out_of_range if pose_id is invalid
   */
  const MapPose& getPose(uint32_t pose_id) const;

  /**
   * @brief Get MADtree pointer by pose ID
   *
   * @param pose_id Pose ID
   * @return Pointer to MADtree (owned by PoseIndex), or nullptr if not found
   */
  MADtree* getTree(uint32_t pose_id) const;

  /**
   * @brief Get full filepath for a tree
   *
   * @param pose_id Pose ID
   * @return Full path to tree binary file
   */
  std::string getTreePath(uint32_t pose_id) const;

  /**
   * @brief Get number of poses in the map
   */
  size_t size() const { return cloud_.poses.size(); }

  /**
   * @brief Check if map is loaded
   */
  bool isLoaded() const { return index_ != nullptr; }

  /**
   * @brief Get the map directory path
   */
  const std::string& getMapDir() const { return map_dir_; }

private:
  std::string map_dir_;
  PoseCloud cloud_;
  std::unique_ptr<PoseKDTree> index_;
  std::unordered_map<uint32_t, std::unique_ptr<MADtree>> trees_;
};

}  // namespace mad_icp_ros
