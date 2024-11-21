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
import typer
from typing_extensions import Annotated
from rich.progress import track
from rich.console import Console
from enum import Enum
import os
import sys
import yaml
import numpy as np
from datetime import datetime
from mad_icp.apps.utils.utils import write_transformed_pose
from mad_icp.apps.utils.ros_reader import Ros1Reader
from mad_icp.apps.utils.ros2_reader import Ros2Reader
from mad_icp.apps.utils.mcap_reader import McapReader
from mad_icp.apps.utils.kitti_reader import KittiReader
from mad_icp.apps.utils.visualizer import Visualizer
from mad_icp.configurations.datasets.dataset_configurations import DatasetConfiguration_lut
from mad_icp.configurations.mad_params import MADConfiguration_lut
# binded vectors and odometry
from mad_icp.src.pybind.pypeline import Pipeline, VectorEigen3d


console = Console()


class InputDataInterface(str, Enum):
    kitti = "kitti",
    ros1  = "ros1",
    ros2  = "ros2",
    mcap  = "mcap"
    # Can insert additional conversion formats


InputDataInterface_lut = {
    InputDataInterface.kitti: KittiReader,
    InputDataInterface.ros1: Ros1Reader,
    InputDataInterface.ros2: Ros2Reader,
    InputDataInterface.mcap: McapReader
}


def main(data_path: Annotated[
        Path, typer.Option(help="path containing one or more rosbags (folder path)", show_default=False)],
        estimate_path: Annotated[
        Path, typer.Option(help="trajectory estimate output path (folder path)", show_default=False)],
        dataset_config: Annotated[
        Path, typer.Option(help="dataset configuration name or path to config file", show_default=False)],
        mad_icp_params: Annotated[
        Path, typer.Option(help="parameters for mad icp", show_default=True)] = "default",
        num_cores: Annotated[
        int, typer.Option(help="how many threads to use for icp (suggest maximum num)", show_default=True)] = 4,
        num_keyframes: Annotated[
        int, typer.Option(help="max number of kf kept in the local map (suggest as num threads)", show_default=True)] = 4,
        realtime: Annotated[
        bool, typer.Option(help="if true anytime realtime", show_default=True)] = False,
        noviz: Annotated[
        bool, typer.Option(help="if true visualizer on", show_default=True)] = False) -> None:
    if not data_path.exists():
        console.print(f"[red] {data_path} does not exist!")
        sys.exit(-1)
    if not estimate_path.is_dir():
        console.print(
            f"[yellow] Output directory {estimate_path} does not exist. Creating new directory")
        estimate_path.mkdir(parents=True, exist_ok=True)

    visualizer = None
    if not noviz:
        visualizer = Visualizer()

    reader_type = InputDataInterface.kitti
    
    if len(list(data_path.glob("*.bag"))) != 0:
        console.print("[yellow] The dataset is in ros bag format")
        reader_type = InputDataInterface.ros1
    elif len(list(data_path.glob("*.db3"))) != 0:
        console.print("[yellow] The dataset is in ros2 db3 format")
        reader_type = InputDataInterface.ros2
    elif len(list(data_path.glob("*.mcap"))) != 0:
        console.print("[yellow] The dataset is in ros2 mcap format")
        reader_type = InputDataInterface.mcap
    else:
        console.print("[yellow] The dataset is in kitti format")


    console.print("[green] Parsing dataset configuration")
    if dataset_config.is_file():
        data_config_file = open(dataset_config, 'r')
        data_cf = yaml.safe_load(data_config_file)
    else:
        dataset_config_str = str(dataset_config)
        if dataset_config_str in DatasetConfiguration_lut:
            data_cf = DatasetConfiguration_lut[dataset_config_str]
        else:
            console.print(f"[red] Error: Dataset '{dataset_config_str}' not found in the configuration lookup table. Possible configurations: {', '.join(DatasetConfiguration_lut.keys())}")
            sys.exit(-1)
    min_range = data_cf["min_range"]
    max_range = data_cf["max_range"]
    sensor_hz = data_cf["sensor_hz"]
    deskew = data_cf["deskew"]
    # apply_correction = data_cf["apply_correction"]
    apply_correction = data_cf.get("apply_correction", False)
    topic = None
    if reader_type in [InputDataInterface.ros1, InputDataInterface.ros2, InputDataInterface.mcap]:
        topic = data_cf["rosbag_topic"]
    lidar_to_base = np.array(data_cf["lidar_to_base"])


    console.print("[green] Parsing mad-icp parameters")
    if mad_icp_params.is_file():
        mad_icp_params_file = open(mad_icp_params, 'r')
        mad_icp_cf = yaml.safe_load(mad_icp_params_file)
    else:
        mad_icp_params_str = str(mad_icp_params)
        if mad_icp_params_str in MADConfiguration_lut:
            mad_icp_cf = MADConfiguration_lut[mad_icp_params_str]
        else:
            console.print(f"[red] Error: mad-icp '{mad_icp_params_str}' not found in the configuration lookup table. Possible configurations: {', '.join(MADConfiguration_lut.keys())}")
            sys.exit(-1)
    b_max = mad_icp_cf["b_max"]
    b_min = mad_icp_cf["b_min"]
    b_ratio = mad_icp_cf["b_ratio"]
    p_th = mad_icp_cf["p_th"]
    rho_ker = mad_icp_cf["rho_ker"]
    n = mad_icp_cf["n"]


    # check some params for machine
    if (realtime and num_keyframes > num_cores):
        console.print(
            "[red] If you chose realtime option, we suggest to chose a num_cores at least >= than the num_keyframes")
        sys.exit(-1)

    console.print("[green] Setting up pipeline for odometry estimation")
    pipeline = Pipeline(sensor_hz, deskew, b_max, rho_ker,
                        p_th, b_min, b_ratio, num_keyframes, num_cores, realtime)

    estimate_file_name = estimate_path / "estimate.txt"
    estimate_file = open(estimate_file_name, 'a')
    estimate_file.truncate(0)

    with InputDataInterface_lut[reader_type](data_path, min_range, max_range, topic=topic, sensor_hz=sensor_hz, apply_correction=apply_correction) as reader:
        t_start = datetime.now()
        for ts, points in track(reader, description="processing..."):

            print("Loading frame #", pipeline.currentID())

            points = VectorEigen3d(points)
            t_end = datetime.now()
            t_delta = t_end - t_start
            print("Time for reading points [ms]: ",
                  t_delta.total_seconds() * 1000)

            t_start = datetime.now()
            pipeline.compute(ts, points)
            t_end = datetime.now()
            t_delta = t_end - t_start
            print(
                "Time for odometry estimation [ms]: ", t_delta.total_seconds() * 1000)

            lidar_to_world = pipeline.currentPose()
            write_transformed_pose(
                estimate_file, lidar_to_world, lidar_to_base)

            if not noviz:
                t_start = datetime.now()
                if pipeline.isMapUpdated():
                    visualizer.update(pipeline.currentLeaves(), pipeline.modelLeaves(
                    ), lidar_to_world, pipeline.keyframePose())
                else:
                    visualizer.update(pipeline.currentLeaves(),
                                      None, lidar_to_world, None)
                t_end = datetime.now()
                t_delta = t_end - t_start
                print("Time for visualization [ms]:",
                      t_delta.total_seconds() * 1000, "\n")

            t_start = datetime.now()

    estimate_file.close()


def run():
    typer.run(main)


if __name__ == '__main__':
    run()
