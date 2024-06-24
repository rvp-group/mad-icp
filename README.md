<div align="center">
    <a href="https://github.com/rvp-group/mad-icp/actions/workflows/pythonbuild.yml"><img src="https://github.com/rvp-group/mad-icp/actions/workflows/pythonbuild.yml/badge.svg"/></a>
    <a href="https://github.com/rvp-group/mad-icp"><img alt="GitHub Repo stars" src="https://img.shields.io/github/stars/rvp-group/mad-icp"></a>
    <a href="https://pypi.org/project/mad-icp/"><img alt="PyPI - Version" src="https://img.shields.io/pypi/v/mad-icp"></a>
    <a href="https://pypi.org/project/mad-icp/"><img alt="PyPI - Downloads" src="https://img.shields.io/pypi/dm/mad-icp"></a>
    <h1>MAD-ICP</h1>
    <h3>It Is All About Matching Data -- Robust and Informed LiDAR Odometry</h3>
    <h3>
        <a href="https://github.com/rvp-group/mad-icp/blob/main/paper_with_supplementary.pdf">Preprint</a>
    </h3>
    <div align="center">
        <a href="https://github.com/rvp-group/mad-icp"><img src="https://github.com/rvp-group/mad-icp/blob/main/mad-icp.gif?raw=true"/></a>   
    </div>
    <br />   
</div>

# Install using `pip`
You can download/install MAD-ICP using `pip`
```bash
pip install mad-icp
```

# Usage

We provide a Python launcher for Rosbag1, Rosbag2, and KITTI binary formats. The dataset configuration is important for the sensor characteristics and extrinsic information (typically, ground truths are not expressed in the LiDAR frame). In `configurations/datasets/dataset_configurations.py` we provide configurations for many datasets.

The internal parameters (used by default) are stored in `configurations/mad_params.py`. All the experiments have been run with this same set.
You can specify a new set in `configurations/mad_params.py` and use it with the option `--mad-icp-params`.

Both the options `--dataset-config` and `--mad-icp-params` also accept `.cfg` files like those in `configurations`.

To run the pipeline, choose the appropriate dataset configuration (`kitti` for this example) and type:
```bash
mad_icp --data-path /input_dir/ \
        --estimate-path /output_dir/ \
        --dataset-config kitti
```
Our runner directly saves the odometry estimate file in KITTI format (homogenous matrix row-major 12 scalars); soon, we will provide more available formats like TUM.

Our pipeline is `anytime realtime`! You can play with parameters `num_keyframes` and `num_cores` and, if you have enough _computation capacity_, we suggest increasing these (we run demo/experiments with `num_keyframes=16` and `num_cores=16`).

____________________________________________________________________

# Building from source

Building is tested by our CI/CD pipeline for Ubuntu 20.04 and Ubuntu 22.04 (using g++).

The following external dependencies are required.
| Dependency   | Version(s) known to work |
| ------------ | ------------------------ |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) | 3.3 |
| [OpenMP](https://www.openmp.org/) |  |
| [pybind11](https://pybind11.readthedocs.io/en/stable/) |  |
| [yaml](https://github.com/jbeder/yaml-cpp) (optional for C++ apps) |  |

If your system lacks any dependency (except for `OpenMP`) we download local copies using `FetchContent`.
If you want to build and install the package, assuming you're inside the repository, you can use `pip` as follows:
```bash
pip install .
```
Moreover, you can build the C++ library (along with the pybinds) by typing:
```bash
mkdir build && cd build && cmake ../mad_icp && make -j8
```

## Building and Running C++ Apps \[Optional\]
If you want to avoid Python, we provide the `bin_runner` C++ executable (located in `mad_icp/apps/cpp_runners/bin_runner.cpp`) that accepts binary cloud format (KITTI, Mulran, etc.).
You can build the executable using
```bash
mkdir build && cd build && cmake -DCOMPILE_CPP_APPS=ON ../mad_icp && make -j
```
And run
```bash
cd build/apps/cpp_runners
./bin_runner -data_path /path_to_bag_folder/ \
             -estimate_path /path_to_estimate_folder/ \
             -dataset_config ../../../mad_icp/configurations/datasets/kitti.cfg \
             -mad_icp_config ../../../mad_icp/configurations/params.cfg 
```
>[!IMPORTANT]
 >If running on the KITTI dataset, enable the flag `-kitti` for KITTI scan correction (not documented anywhere). We do not (currently) provide a viewer for this executable. 

# What is missing?
- ROS/ROS2 optional dependencies

# Cite us
Our work is currently under review; in the meantime, if you use any of this code, here is the preprint information:

```
@article{ferrari2024mad,
  title={MAD-ICP: It Is All About Matching Data--Robust and Informed LiDAR Odometry},
  author={Ferrari, Simone and Di Giammarino, Luca and Brizi, Leonardo and Grisetti, Giorgio},
  journal={arXiv preprint arXiv:2405.05828},
  year={2024}
}
```
