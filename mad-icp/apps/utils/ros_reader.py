import os
import sys
from pathlib import Path
from typing import Tuple
import natsort
from utils.point_cloud2 import read_point_cloud
import numpy as np

class Ros1Reader:
    def __init__(self, data_dir: Path, min_range = 0, 
                 max_range = 200, *args, **kwargs):
        """
        :param data_dir: Directory containing rosbags or path to a rosbag file
        :param topics: Topic to read
        :param min_range: minimum range for the points
        :param max_range: maximum range for the points
        :param args:
        :param kwargs:
        """
        topic = kwargs.pop('topic')
        try:
            from rosbags.highlevel import AnyReader
        except ModuleNotFoundError:
            print("rosbags library not installed, run 'pip install -U rosbags'")
            sys.exit(-1)

        if data_dir.is_file():
            #self.sequence_id = os.path.basename(data_dir).split(".")[0]
            self.bag = AnyReader([data_dir])
        else:
            #self.sequence_id = os.path.basename(data_dir[0]).split(".")[0]
            self.bag = AnyReader(natsort.natsorted([bag for bag in list(data_dir.glob("*.bag"))]))
            print("Reading multiple .bag files in directory:")
            print("\n".join([path.name for path in self.bag.paths]))

        self.bag.open()
        connection = self.bag.connections

        if not topic:
            raise Exception("You have to specify a topic")

        print("Reading the following topic: ", topic)
        connection = [x for x in self.bag.connections if x.topic == topic]
        self.msgs = self.bag.messages(connections=connection)

        self.min_range = min_range
        self.max_range = max_range
        self.topic = topic
        self.num_messages = self.bag.topics[topic].msgcount

    def __len__(self):
        return self.num_messages

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if hasattr(self, "bag"):
            self.bag.close()

    def __getitem__(self, item) -> Tuple[float, Tuple[np.ndarray, np.ndarray]]:
        connection, timestamp, rawdata = next(self.msgs)
        msg = self.bag.deserialize(rawdata, connection.msgtype)
        cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points, _ = read_point_cloud(msg, min_range=self.min_range, max_range=self.max_range)
        return timestamp, points