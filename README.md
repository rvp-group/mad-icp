# MAD-ICP

## It Is All About Matching Data -- Robust and Informed LiDAR Odometry

A minimal, robust, accurate, and real-time LiDAR odometry.
<div align="center">
    <img src="mad-icp.gif" width="720"/>
</div>

This version is mainly for reviewers. Soon, you can install this system via `pip`. Our <a href="paper_with_supplementary.pdf">preprint</a> is available for more details and results.


## Building ##

Building has been tested on Ubuntu 20.04 (with g++).

The following external dependencies are required.

| Dependency   | Version(s) known to work |
| ------------ | ------------------------ |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) | 3.3 |
| [OpenMP](https://www.openmp.org/) |  |
| [pybind11](https://pybind11.readthedocs.io/en/stable/) |  |
| [yaml](https://github.com/jbeder/yaml-cpp) (optional C++ apps) |  |


After obtaining all dependencies, the application can be built with CMake, for example, as follows:

```bash
cd mad-icp
mkdir build && cd build && cmake .. && make -j
```

## Running

A few other Python packages need to be installed for running. You can find the specific versions in the `requirements.txt`. We suggest to create a virtual env and run `pip3 install -r requirements.txt`.

We provide a Python launcher for both rosbags and bin formats. The configuration file is important for the sensor characteristics and extrinsic information (usually, ground truths are not in the LiDAR frame). The internal parameters are in `configurations/params.cfg`; all the experiments have been run with this same set.

How to run (make sure `estimate_path` and `data_path` point to a folder):
```bash
cd apps
python3 mad-icp.py --data_path /path_to_bag_folder/ --estimate_path /path_to_estimate_folder/ --dataset_config ../configurations/datasets/dataset_config_file --mad_icp_config ../configurations/params.cfg --num_cores 4 --num_keyframes 4 --realtime
```

Our runner directly saves the odometry estimate file in KITTI format (homogenous matrix row-major 12 components); in the near future, we will provide more available formats like TUM. 
Our pipeline is anytime realtime, therefore you can play with parameters `num_keyframes` and `num_cores`, if you have enough computation we suggest to increase these (we run demo with 16 and 16), if not you can leave it in the proposed way.

### (optional) Building and Running C++ Apps
If you want to avoid Python, we provide a C++ executable that works just with binary cloud format (KITTI, Mulran, etc.), this file called `bin_runner` can be found in `build/apps/cpp_runners`.
You can build this using
```bash
cd mad-icp
mkdir build && cd build && cmake -DCOMPILE_CPP_APPS=ON .. && make -j
```

And run
```bash
cd build/apps/cpp_runners
./bin_runner -data_path /path_to_bag_folder/ -estimate_path /path_to_estimate_folder/ -dataset_config ../../../configurations/datasets/dataset_config_file -mad_icp_config ../../../configurations/params.cfg -num_cores 4 -num_keyframes 4 -realtime
```
If running on the KITTI dataset, make sure to enable the flag `-kitti` for KITTI scan correction (not documented anywhere). We do not (currently) provide a viewer for this executable. 


## What is missing?
- `pip` package for easy install (coming soon)
- ROS/ROS2 optional dependencies
