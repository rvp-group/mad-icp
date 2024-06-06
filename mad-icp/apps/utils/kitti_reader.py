import os
import sys
from pathlib import Path
from typing import Tuple, Union
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

    def __len__(self):
        return len(self.file_names)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return

    def __getitem__(self, item) -> Tuple[float, Tuple[np.ndarray, np.ndarray]]:
        
        with open(self.data_dir / ".dtype.pkl", "rb") as f:
            cdtype = pickle.load(f)

        cloud_np = np.fromfile(self.file_names[self.file_index], dtype=cdtype)
        pts = np.stack([
        cloud_np["x"],
        cloud_np["y"],
        cloud_np["z"],
        cloud_np["intensity"]], axis=1)
        # if (point.norm() < min_range || point.norm() > max_range || std::isnan(point.x()) || std::isnan(point.y()) ||
        #   std::isnan(point.z()))
        self.time += self.time_inc
        self.file_index += 1
        return self.time, cloud_np