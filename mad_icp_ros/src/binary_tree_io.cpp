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

#include "mad_icp_ros/binary_tree_io.h"

#include <cstring>
#include <fstream>
#include <queue>
#include <vector>

namespace mad_icp_ros {

std::unique_ptr<MADtree> deserializeTree(const std::string& filepath) {
  std::ifstream file(filepath, std::ios::binary);
  if (!file) {
    return nullptr;
  }

  // Read header
  BinaryTreeHeader header;
  file.read(reinterpret_cast<char*>(&header), sizeof(header));

  if (!file || std::strncmp(header.magic, "MADT", 4) != 0) {
    return nullptr;
  }

  if (header.version != 1) {
    return nullptr;
  }

  if (header.num_nodes == 0) {
    return nullptr;
  }

  // Read all nodes into a vector (BFS order)
  std::vector<MADtree*> nodes;
  nodes.reserve(header.num_nodes);

  // Track which nodes need children and what children they expect
  // Queue entries: (node_ptr, needs_left, needs_right)
  std::queue<std::tuple<MADtree*, bool, bool>> pending_parents;

  for (uint32_t i = 0; i < header.num_nodes; ++i) {
    MADtree* node = new MADtree();

    // Read node data
    file.read(reinterpret_cast<char*>(node->mean_.data()), 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(node->bbox_.data()), 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(node->eigenvectors_.data()), 9 * sizeof(double));

    int32_t num_points;
    file.read(reinterpret_cast<char*>(&num_points), sizeof(num_points));
    node->num_points_ = num_points;

    uint8_t flags, padding;
    file.read(reinterpret_cast<char*>(&flags), sizeof(flags));
    file.read(reinterpret_cast<char*>(&padding), sizeof(padding));

    if (!file) {
      // Read error - clean up allocated nodes
      for (auto* n : nodes) {
        // Only delete nodes that aren't connected as children
        // to avoid double-free (parent destructor deletes children)
        if (n->parent_ == nullptr && n != nodes[0]) {
          delete n;
        }
      }
      if (nodes.empty() || node != nodes[0]) {
        delete node;
      }
      if (!nodes.empty()) {
        delete nodes[0];  // This will cascade delete children
      }
      return nullptr;
    }

    bool has_left = flags & 1;
    bool has_right = flags & 2;

    nodes.push_back(node);

    // If this is not the root (i > 0), assign it as child of its parent
    if (i > 0 && !pending_parents.empty()) {
      auto& [parent, needs_left, needs_right] = pending_parents.front();

      if (needs_left) {
        parent->left_ = node;
        node->parent_ = parent;

        if (needs_right) {
          // Still need right child - update the entry
          std::get<1>(pending_parents.front()) = false;
        } else {
          // Parent is complete
          pending_parents.pop();
        }
      } else if (needs_right) {
        parent->right_ = node;
        node->parent_ = parent;
        pending_parents.pop();
      }
    }

    // Queue this node if it has children
    if (has_left || has_right) {
      pending_parents.push({node, has_left, has_right});
    }
  }

  // Transfer ownership of root
  std::unique_ptr<MADtree> root(nodes[0]);
  return root;
}

bool serializeTree(const MADtree* tree, const std::string& filepath) {
  if (!tree) {
    return false;
  }

  std::ofstream file(filepath, std::ios::binary);
  if (!file) {
    return false;
  }

  // Count nodes
  std::function<uint32_t(const MADtree*)> countNodes = [&](const MADtree* node) -> uint32_t {
    if (!node) return 0;
    return 1 + countNodes(node->left_) + countNodes(node->right_);
  };

  uint32_t num_nodes = countNodes(tree);

  // Write header
  BinaryTreeHeader header;
  std::memcpy(header.magic, "MADT", 4);
  header.version = 1;
  header.num_nodes = num_nodes;
  header.reserved = 0;

  file.write(reinterpret_cast<const char*>(&header), sizeof(header));

  // Write nodes in breadth-first order
  std::queue<const MADtree*> queue;
  queue.push(tree);

  while (!queue.empty()) {
    const MADtree* node = queue.front();
    queue.pop();

    // Write node data
    file.write(reinterpret_cast<const char*>(node->mean_.data()), 3 * sizeof(double));
    file.write(reinterpret_cast<const char*>(node->bbox_.data()), 3 * sizeof(double));
    file.write(reinterpret_cast<const char*>(node->eigenvectors_.data()), 9 * sizeof(double));

    int32_t num_points = node->num_points_;
    file.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

    uint8_t flags = 0;
    if (node->left_) flags |= 1;
    if (node->right_) flags |= 2;
    file.write(reinterpret_cast<const char*>(&flags), sizeof(flags));

    uint8_t padding = 0;
    file.write(reinterpret_cast<const char*>(&padding), sizeof(padding));

    // Enqueue children
    if (node->left_) queue.push(node->left_);
    if (node->right_) queue.push(node->right_);
  }

  return file.good();
}

}  // namespace mad_icp_ros
