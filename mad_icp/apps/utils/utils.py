# Copyright 2024 R(obots) V(ision) and P(erception) group
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion (w, x, y, z).
    
    Uses scipy's Rotation class which is more robust to numerical errors.
    """
    rot = Rotation.from_matrix(R)
    # scipy returns quaternion as [x, y, z, w], we need [w, x, y, z]
    quat = rot.as_quat()  # [x, y, z, w]
    return quat[3], quat[0], quat[1], quat[2]  # [w, x, y, z]


def write_transformed_pose(estimate_file, lidar_to_world, lidar_to_base, timestamp=None, output_format="kitti"):
    """
    Write pose to file in specified format.
    
    Args:
        estimate_file: File handle to write to
        lidar_to_world: 4x4 transformation matrix from lidar to world
        lidar_to_base: 4x4 transformation matrix from lidar to base
        timestamp: Timestamp for TUM format (required for TUM)
        output_format: "kitti" or "tum"
    """
    base_to_lidar = np.linalg.inv(lidar_to_base)
    base_to_world = np.dot(lidar_to_base, np.dot(lidar_to_world, base_to_lidar))
    
    if output_format == "kitti":
        # KITTI format: 12 values (3x4 matrix row-major)
        np.savetxt(estimate_file, base_to_world[:3].reshape(-1, 12))
    elif output_format == "tum":
        # TUM format: timestamp tx ty tz qx qy qz qw
        tx, ty, tz = base_to_world[:3, 3]
        qw, qx, qy, qz = rotation_matrix_to_quaternion(base_to_world[:3, :3])
        estimate_file.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    else:
        raise ValueError(f"Unknown output format: {output_format}. Supported: 'kitti', 'tum'")
