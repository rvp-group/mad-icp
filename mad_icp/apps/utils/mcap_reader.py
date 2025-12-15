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


class McapReader:
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
        self.topic = kwargs.pop('topic')
        
        if not self.topic:
            raise Exception("You have to specify a topic")
        
        try:
            from mcap.reader import make_reader
            from mcap_ros2.reader import read_ros2_messages
        except ModuleNotFoundError:
            print("mcap package not installed: run 'pip install -U mcap-ros2-support'")
            sys.exit(-1)

        print("Reading the following topic: ", self.topic)

        self.min_range = min_range
        self.max_range = max_range

        mcap_file_path = next(data_dir.glob("*.mcap"), None)
        if not mcap_file_path or not os.path.isfile(mcap_file_path):
            raise FileNotFoundError(f"mcap dataloader expects an existing MCAP file in {data_dir}")
        mcap_file = str(mcap_file_path)

        self.bag = make_reader(open(mcap_file, "rb"))
        self.summary = self.bag.get_summary()
        self.topic = self.check_topic(self.topic)
        self.num_messages = sum(
            count
            for (id, count) in self.summary.statistics.channel_message_counts.items()
            if self.summary.channels[id].topic == self.topic
        )
        self.msgs = read_ros2_messages(mcap_file, topics=[self.topic])
        self.read_point_cloud = read_point_cloud

    def __len__(self):
        return self.num_messages

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        return

    def __getitem__(self, item) -> Tuple[float, Tuple[np.ndarray, np.ndarray]]:
        msg = next(self.msgs).ros_msg
        points, _ = read_point_cloud(
            msg, min_range=self.min_range, max_range=self.max_range)
        cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return cloud_stamp, points
    
    def __del__(self):
        if hasattr(self, "bag"):
            del self.bag

    def check_topic(self, topic: str) -> str:
        # Extract schema id from the .mcap file that encodes the PointCloud2 msg
        schema_id = [
            schema.id
            for schema in self.summary.schemas.values()
            if schema.name == "sensor_msgs/msg/PointCloud2"
        ][0]

        point_cloud_topics = [
            channel.topic
            for channel in self.summary.channels.values()
            if channel.schema_id == schema_id
        ]

        def print_available_topics_and_exit():
            print(50 * "-")
            for t in point_cloud_topics:
                print(f"topic : {t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in point_cloud_topics:
            return topic
        # when user specified the topic check that exists
        if topic and topic not in point_cloud_topics:
            print(f"Error: Input bag does not contain any topic with this name: {topic}")
            print_available_topics_and_exit()