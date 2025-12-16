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
#include <memory>
#include <string>

namespace mad_icp_ros {

/**
 * @brief Binary file header for MADtree serialization
 *
 * Format: 16 bytes total
 * - magic: "MADT" (4 bytes)
 * - version: uint32 (4 bytes) - currently 1
 * - num_nodes: uint32 (4 bytes)
 * - reserved: uint32 (4 bytes)
 */
struct BinaryTreeHeader {
  char magic[4];
  uint32_t version;
  uint32_t num_nodes;
  uint32_t reserved;
};

/**
 * @brief Deserialize a MADtree from a binary file
 *
 * The binary format stores nodes in breadth-first order.
 * Each node (126 bytes):
 * - mean: 3 x double (24 bytes)
 * - bbox: 3 x double (24 bytes)
 * - eigenvectors: 9 x double (72 bytes)
 * - num_points: int32 (4 bytes)
 * - flags: uint8 (1 byte) - bit 0: has_left, bit 1: has_right
 * - padding: uint8 (1 byte)
 *
 * @param filepath Path to the binary tree file
 * @return Unique pointer to deserialized MADtree, or nullptr on failure
 */
std::unique_ptr<MADtree> deserializeTree(const std::string& filepath);

/**
 * @brief Serialize a MADtree to a binary file
 *
 * @param tree Pointer to the MADtree to serialize
 * @param filepath Path to the output file
 * @return true on success, false on failure
 */
bool serializeTree(const MADtree* tree, const std::string& filepath);

}  // namespace mad_icp_ros
