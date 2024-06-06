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

import os, sys
import yaml
import argparse
import numpy as np
from datetime import datetime
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from utils.point_cloud2 import read_point_cloud
from utils.utils import write_transformed_pose
from utils.visualizer import Visualizer

# binded odometry
sys.path.append("../build/src/odometry/")
from pypeline import Pipeline, VectorEigen3d

parser = argparse.ArgumentParser(description='mad-icp runner for rosbag')
parser.add_argument('--data_path', help='path containing one or more rosbags (folder path)', required=True)
parser.add_argument('--estimate_path', help='trajectory estimate output path (folder path)', required=True)
# dataset and mad-icp configurations
parser.add_argument('--dataset_config', help='dataset configuration file', required=True)
parser.add_argument('--mad_icp_config', help='parameters for mad icp', default="../configurations/params.cfg", required=True)
# machine/pc dependent parameters
parser.add_argument('--num_cores', type=int, help='how many threads to use for icp (suggest maximum num)', default=4)
parser.add_argument('--num_keyframes', type=int, help='max number of kf kept in the local map (suggest as num threads)', default=4)
parser.add_argument('--realtime', help='if true anytime realtime', action='store_true')
args = parser.parse_args()

# parse dataset configuration file
data_config_file = open(args.dataset_config, 'r')
data_cf = yaml.safe_load(data_config_file)
min_range = data_cf["min_range"] 
max_range = data_cf["max_range"] 
sensor_hz = data_cf["sensor_hz"] 
deskew = data_cf["deskew"] 
topic = data_cf["rosbag_topic"]
lidar_to_base = np.array(data_cf["lidar_to_base"])

# parse mad-icp configuration file
mad_icp_config_file = open(args.mad_icp_config, 'r')
mad_icp_cf = yaml.safe_load(mad_icp_config_file)

b_max = mad_icp_cf["b_max"]
b_min = mad_icp_cf["b_min"]
b_ratio = mad_icp_cf["b_ratio"]
p_th = mad_icp_cf["p_th"]
rho_ker = mad_icp_cf["rho_ker"]
n = mad_icp_cf["n"]

# check some params for machine
if(args.realtime):
	assert args.num_keyframes <= args.num_cores

data_path = Path(args.data_path)
assert data_path.is_dir()

# sort files in directory
files_in_directory = sorted([f for f in data_path.glob("*.bag")])

output_name = os.path.join(args.estimate_path, "estimate.txt")
estimate_file = open(output_name, 'w')

# setting up pipeline for odometry estimation
pipeline = Pipeline(sensor_hz, deskew, b_max, rho_ker, p_th, b_min, b_ratio, args.num_keyframes, args.num_cores, args.realtime)

typestore = get_typestore(Stores.ROS2_FOXY)

visualizer = Visualizer()

for filename in files_in_directory:
	with AnyReader([filename], default_typestore=typestore) as reader:
		connections = [x for x in reader.connections if x.topic == topic]
		for connection, timestamp, rawdata in reader.messages(connections=connections):
			msg = reader.deserialize(rawdata, connection.msgtype)
			cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
			
			# print("Cloud stamp:", cloud_stamp)
			print("Loading frame #", pipeline.currentID())
			# print("Keyframe is frame #", pipeline.keyframeID(), "\n")

			t_start = datetime.now()
			points, _ = read_point_cloud(msg, min_range=min_range, max_range=max_range)
			points = VectorEigen3d(points)
			t_end = datetime.now()
			t_delta = t_end - t_start
			print("Time for reading points [ms]:", t_delta.total_seconds() * 1000)

			t_start = datetime.now()
			pipeline.compute(cloud_stamp, points)
			t_end = datetime.now()
			t_delta = t_end - t_start
			print("Time for odometry estimation [ms]:", t_delta.total_seconds() * 1000)

			lidar_to_world = pipeline.currentPose()
			write_transformed_pose(estimate_file, lidar_to_world, lidar_to_base)

			t_start = datetime.now()
			if pipeline.isMapUpdated():
				visualizer.update(pipeline.currentLeaves(), pipeline.modelLeaves(), lidar_to_world, pipeline.keyframePose())
			else:
				visualizer.update(pipeline.currentLeaves(), None, lidar_to_world, None)
			t_end = datetime.now()
			t_delta = t_end - t_start
			print("Time for visualization in ms:", t_delta.total_seconds() * 1000, "\n")

estimate_file.close()

