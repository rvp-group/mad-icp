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

"""TUM trajectory format parser.

TUM format: timestamp tx ty tz qx qy qz qw
- timestamp: float seconds
- tx, ty, tz: translation
- qx, qy, qz, qw: quaternion (Hamilton convention, scalar last)
"""

from pathlib import Path
from typing import List, Tuple, Dict
import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix.

    Args:
        qx, qy, qz, qw: Quaternion components (scalar last)

    Returns:
        3x3 rotation matrix
    """
    rot = Rotation.from_quat([qx, qy, qz, qw])
    return rot.as_matrix()


def parse_tum_trajectory(filepath: Path) -> List[Tuple[float, np.ndarray]]:
    """Parse TUM format trajectory file.

    Args:
        filepath: Path to TUM trajectory file

    Returns:
        List of (timestamp, 4x4 pose matrix) tuples
    """
    trajectory = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            # Skip empty lines and comments
            if not line or line.startswith('#'):
                continue

            parts = line.split()
            if len(parts) != 8:
                raise ValueError(f"Invalid TUM line (expected 8 values): {line}")

            timestamp = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])

            # Build 4x4 homogeneous transformation matrix
            pose = np.eye(4)
            pose[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
            pose[:3, 3] = [tx, ty, tz]

            trajectory.append((timestamp, pose))

    return trajectory


def trajectory_to_dict(trajectory: List[Tuple[float, np.ndarray]],
                       tolerance: float = 1e-6) -> Dict[float, np.ndarray]:
    """Convert trajectory list to timestamp-indexed dictionary.

    Args:
        trajectory: List of (timestamp, pose) tuples
        tolerance: Timestamp precision for matching (default 1 microsecond)

    Returns:
        Dictionary mapping timestamps to poses
    """
    # Round timestamps to the tolerance level for exact matching
    return {round(t, 6): pose for t, pose in trajectory}


def find_matching_pose(trajectory_dict: Dict[float, np.ndarray],
                       query_timestamp: float,
                       tolerance: float = 1e-6) -> Tuple[bool, np.ndarray]:
    """Find pose with matching timestamp.

    Args:
        trajectory_dict: Dictionary of timestamp -> pose
        query_timestamp: Timestamp to search for
        tolerance: Maximum allowed difference (default 1 microsecond)

    Returns:
        Tuple of (found, pose). If not found, pose is identity.
    """
    rounded_query = round(query_timestamp, 6)

    if rounded_query in trajectory_dict:
        return True, trajectory_dict[rounded_query]

    # Try to find within tolerance
    for t, pose in trajectory_dict.items():
        if abs(t - query_timestamp) < tolerance:
            return True, pose

    return False, np.eye(4)
