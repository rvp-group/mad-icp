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

from enum import Enum

hilti_2021_conf = {
    "min_range": 0.7,
    "max_range": 100,
    "sensor_hz": 10,
    "deskew": False,
    "rosbag_topic": "/os_cloud_node/points",
    "lidar_to_base": [
        [1, 0.0025, -0.0065, 0.0100],
        [0.0025, -1, 0.0003, -0.0066],
        [-0.0065, -0.0003, -1, 0.0947],
        [0, 0, 0, 1]
    ]
}

kitti_conf = {
    "min_range": 0.7,
    "max_range": 120,
    "sensor_hz": 10,
    "deskew": False,
    "apply_correction": True,
    "lidar_to_base": [
        [4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02],
        [-7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02],
        [9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01],
        [0, 0, 0, 1]
    ]
}

mulran_conf = {
    "min_range": 0.7,
    "max_range": 120,
    "sensor_hz": 10,
    "deskew": True,
    "lidar_to_base": [
        [-1, -0.0058, 0, 1.7042],
        [0.0058, -1, 0, -0.0210],
        [0, 0, 1, 1.8047],
        [0, 0, 0, 1]
    ]
}

newer_college_os0_conf = {
    "min_range": 0.7,
    "max_range": 50,
    "sensor_hz": 10,
    "deskew": False,
    "rosbag_topic": "/os_cloud_node/points",
    "lidar_to_base": [
        [1, 0, 0, 0.001],
        [0, 1, 0, 0],
        [0, 0, 1, 0.090683],
        [0, 0, 0, 1]
    ]
}

newer_college_os1_conf = {
    "min_range": 0.7,
    "max_range": 120,
    "sensor_hz": 10,
    "deskew": False,
    "rosbag_topic": "/os1_cloud_node/points",
    "lidar_to_base": [
        [-0.7071, -0.7071, 0, -0.0843],
        [0.7071, -0.7071, 0, -0.0250],
        [0, 0, 1, 0.0502],
        [0, 0, 0, 1]
    ]
}

vbr_os0_conf = {
    "min_range": 0,
    "max_range": 50,
    "sensor_hz": 10,
    "deskew": False,
    "rosbag_topic": "/ouster/points",
    "lidar_to_base": [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
}

vbr_os1_conf = {
    "min_range": 1.3,
    "max_range": 120,
    "sensor_hz": 20,
    "deskew": True,
    "rosbag_topic": "/ouster/points",
    "lidar_to_base": [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
}

class DatasetConfiguration(str, Enum):
    hilti_2021 = "hilti_2021",
    kitti = "kitti",
    mulran = "mulran",
    newer_college_os0 = "newer_college_os0",
    newer_college_os1 = "newer_college_os1",
    vbr_os0 = "vbr_os0",
    vbr_os1 = "vbr_os1"
    # Can insert additional dataset configurations


DatasetConfiguration_lut = {
    DatasetConfiguration.hilti_2021: hilti_2021_conf,
    DatasetConfiguration.kitti: kitti_conf,
    DatasetConfiguration.mulran: mulran_conf,
    DatasetConfiguration.newer_college_os0: newer_college_os0_conf,
    DatasetConfiguration.newer_college_os1: newer_college_os1_conf,
    DatasetConfiguration.vbr_os0: vbr_os0_conf,
    DatasetConfiguration.vbr_os1: vbr_os1_conf
}