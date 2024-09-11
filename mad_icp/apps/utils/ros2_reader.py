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

import os
import sys
from pathlib import Path
from typing import Tuple
import natsort
from mad_icp.apps.utils.point_cloud2 import read_point_cloud
import numpy as np


class Ros2Reader:
    def __init__(self, data_dir: Path, min_range=0,
                 max_range=200, *args, **kwargs):
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
            print("Rosbags library not installed, run 'pip install -U rosbags'")
            sys.exit(-1)


        self.bag = AnyReader([data_dir])

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
        points, _ = read_point_cloud(
            msg, min_range=self.min_range, max_range=self.max_range)
        return timestamp, points
