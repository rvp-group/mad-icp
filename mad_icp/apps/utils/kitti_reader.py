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

from pathlib import Path
from typing import Tuple
import natsort
import pickle
import numpy as np

class KittiReader:
    def __init__(self, data_dir: Path, min_range = 0, 
                 max_range = 200, *args, **kwargs):
        """
        :param data_dir: Directory containing rosbags or path to a rosbag file
        :param topics: None for kitti
        :param min_range: minimum range for the points
        :param max_range: maximum range for the points
        :param args:
        :param kwargs:
        """
        sensor_hz = kwargs.pop('sensor_hz')
        self.file_names = natsort.natsorted([file for file in list(data_dir.glob("*.bin"))])

        self.min_range = min_range
        self.max_range = max_range
        self.time = 0.
        self.time_inc = 1./sensor_hz
        self.file_index = 0
        self.data_dir = data_dir
        self.cdtype = np.float32
        if (self.data_dir / ".dtype.pkl").exists():
            f = open(self.data_dir / ".dtype.pkl", "rb")
            self.cdtype = pickle.load(f)

    def __len__(self):
        return len(self.file_names)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return

    def __getitem__(self, item) -> Tuple[float, Tuple[np.ndarray, np.ndarray]]:
        cloud_np = np.fromfile(self.file_names[self.file_index], dtype=self.cdtype)
        # with open(self.file_names[self.file_index], 'rb') as f:
        #     buffer = f.read()
        #     cloud_np = np.frombuffer(buffer, dtype=self.cdtype)
        cloud_np = cloud_np.reshape(-1, 4)[:, :3]
        # if (point.norm() < min_range || point.norm() > max_range || std::isnan(point.x()) || std::isnan(point.y()) ||
        #   std::isnan(point.z()))
        norms = np.linalg.norm(cloud_np, axis=1)
        mask = (norms >= self.min_range) & (norms <= self.max_range)
        # Use the mask to filter the points
        filtered_points = cloud_np[mask]
        self.time += self.time_inc
        self.file_index += 1
        return self.time, filtered_points