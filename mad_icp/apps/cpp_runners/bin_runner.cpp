// Copyright 2024 R(obots) V(ision) and P(erception) group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iomanip>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <filesystem>
#include <fstream>
#include <sys/time.h>

#include "cpp_utils/parse_cmd_line.h"
#include <yaml-cpp/yaml.h>

#include <odometry/pipeline.h>
#include <tools/mad_tree.h>

std::string data_path;
std::string estimate_path;
std::string dataset_config;
std::string mad_icp_config = "../../../configurations/default.cfg";
int num_cores              = 4;
int num_keyframes          = 4;
bool realtime              = false;
bool kitti                 = false;

// for kitti scan correction (not documented)
constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;

// parsing cmd line stuff
void printHelp();
void parseCmdLine(int argc, char* argv[]);
Eigen::Matrix4d parseMatrix(const std::vector<std::vector<double>>& vec);

// write homogenous transformation in base frame
void writeTransformedPose(std::ofstream& estimate_file_buf,
                          const Eigen::Matrix4d& lidar_to_world,
                          const Eigen::Matrix4d& lidar_to_base);

int main(int argc, char* argv[]) {
  parseCmdLine(argc, argv);
  // srand(time(NULL));
  YAML::Node yaml_dataset_config, yaml_mad_icp_config;
  yaml_dataset_config = YAML::LoadFile(dataset_config);
  yaml_mad_icp_config = YAML::LoadFile(mad_icp_config);

  // parse data specific configurations
  const double min_range = yaml_dataset_config["min_range"].as<double>();
  const double max_range = yaml_dataset_config["max_range"].as<double>();
  const double sensor_hz = yaml_dataset_config["sensor_hz"].as<double>();
  const bool deskew      = yaml_dataset_config["deskew"].as<bool>();
  // parsing lidar in base homogenous transformation
  const auto lidar_to_base_vec        = yaml_dataset_config["lidar_to_base"].as<std::vector<std::vector<double>>>();
  const Eigen::Matrix4d lidar_to_base = parseMatrix(lidar_to_base_vec);

  // parse mad-icp configuration
  const double b_max   = yaml_mad_icp_config["b_max"].as<double>();
  const double b_min   = yaml_mad_icp_config["b_min"].as<double>();
  const double b_ratio = yaml_mad_icp_config["b_ratio"].as<double>();
  const double p_th    = yaml_mad_icp_config["p_th"].as<double>();
  const double rho_ker = yaml_mad_icp_config["rho_ker"].as<double>();
  const int n          = yaml_mad_icp_config["n"].as<int>();

  // check some params for machine
  if (realtime)
    if (num_keyframes > num_cores) {
      throw std::runtime_error("if realtime num_keyframes needs to be <= num_cores");
    }

  // parsing bin filenames in order
  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(data_path),
            std::filesystem::directory_iterator(),
            std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end());

  struct timeval t_start, t_end, t_delta;

  std::unique_ptr<Pipeline> pipeline =
    std::make_unique<Pipeline>(sensor_hz, deskew, b_max, rho_ker, p_th, b_min, b_ratio, num_keyframes, num_cores, realtime);

  double time            = 0.;
  const double time_incr = 1. / sensor_hz;

  std::ofstream os;
  std::string estimate_file = estimate_path + "/estimate.txt";
  os.open(estimate_file);
  os << std::fixed << std::setprecision(12);

  ContainerType cloud;
  Eigen::Matrix4d lidar_to_world;

  for (const std::string& filename : files_in_directory) {
    std::cout << "Loading frame # " << pipeline->currentID() << std::endl;

    // load bin point cloud - start
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    gettimeofday(&t_start, nullptr);
    int32_t num = 1000000;
    float* data = (float*) malloc(num * sizeof(float));

    // pointers
    float* px = data + 0;
    float* py = data + 1;
    float* pz = data + 2;
    float* pr = data + 3;

    cloud.clear();
    cloud.reserve(num);

    // load bin point cloud
    FILE* stream;
    stream = fopen(filename.c_str(), "rb");
    if (!stream) {
      std::cerr << "Failed to open file: " << filename << std::endl;
      free(data);
      continue;
    }
    num    = fread(data, sizeof(float), num, stream) / 4;
    for (int32_t i = 0; i < num; i++) {
      Eigen::Vector3f point(*px, *py, *pz);
      px += 4;
      py += 4;
      pz += 4;
      pr += 4;

      if (point.norm() < min_range || point.norm() > max_range || std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z()))
        continue;

      // apply kitti magic correction (not documented)
      const Eigen::Vector3d pointd = point.cast<double>();
      if (kitti) {
        const Eigen::Vector3d rotation_vector = pointd.cross(Eigen::Vector3d(0., 0., 1.));
        const Eigen::Vector3d corrected_point = Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotation_vector.normalized()) * pointd;
        cloud.push_back(corrected_point);
      } else {
        cloud.push_back(pointd);
      }
    }

    cloud.shrink_to_fit();
    fclose(stream);
    free(data);
    gettimeofday(&t_end, nullptr);
    timersub(&t_end, &t_start, &t_delta);
    std::cout << std::fixed << std::setprecision(4)
              << "Time for reading points [ms]: " << double(t_delta.tv_sec) * 1000. + 1e-3 * t_delta.tv_usec << std::endl;
    // load bin point cloud - end

    gettimeofday(&t_start, nullptr);
    pipeline->compute(time, cloud);
    gettimeofday(&t_end, nullptr);
    timersub(&t_end, &t_start, &t_delta);
    std::cout << std::fixed << std::setprecision(4)
              << "Time for odometry estimation [ms]: " << double(t_delta.tv_sec) * 1000. + 1e-3 * t_delta.tv_usec << std::endl;

    lidar_to_world = pipeline->currentPose();

    time += time_incr;

    // dump trajectory to txt file, no visualization in C++
    // use Python for viz
    writeTransformedPose(os, lidar_to_world, lidar_to_base);
    std::cout << std::endl;
  }

  os.close();
  return 0;
}

void printHelp() {
  // clang-format off
  std::cout << "mad-icp runner for bin (KITTI, Mulran format)\n";
  std::cout << "bin_runner -data_path <path-to-data> -estimate_path <path-to-estimate> -dataset_config <path-to-dataset-config>\n";
  std::cout << "  -data_path        [string] path containing multiple bin (folder path)\n";
  std::cout << "  -estimate_path    [string] trajectory estimate output path (folder path)\n";
  std::cout << "  -dataset_config   [string] dataset configuration file\n";
  std::cout << "  -mad_icp_config   [string] parameters for mad icp\n";
  std::cout << "  -num_cores        [int] how many threads to use for icp (suggest maximum num)\n";
  std::cout << "  -num_keyframes    [int] max number of kf kept in the local map\n";
  std::cout << "  -realtime         [flag] if true anytime realtime\n";
  std::cout << "  -kitti            [flag] if kitti, applies kitti correction\n";
  std::cout << "  -h                display this help and exit\n";
  // clang-format on
}

void parseCmdLine(int argc, char* argv[]) {
  InputParser input(argc, argv);
  // mandatory args
  if (!input.cmdOptionExists("-data_path") || !input.cmdOptionExists("-estimate_path") ||
      !input.cmdOptionExists("-dataset_config")) {
    printHelp();
    exit(0);
  }

  data_path      = input.getCmdOption("-data_path");
  estimate_path  = input.getCmdOption("-estimate_path");
  dataset_config = input.getCmdOption("-dataset_config");

  // default ok!
  if (input.cmdOptionExists("-mad_icp_config"))
    mad_icp_config = input.getCmdOption("-mad_icp_config");
  if (input.cmdOptionExists("-num_cores"))
    num_cores = input.getInt("-num_cores");
  if (input.cmdOptionExists("-num_keyframes"))
    num_keyframes = input.getInt("-num_keyframes");
  if (input.cmdOptionExists("-realtime"))
    realtime = true;
  if (input.cmdOptionExists("-kitti"))
    kitti = true;

  // help message
  if (input.cmdOptionExists("-h")) {
    printHelp();
    exit(0);
  }
}

Eigen::Matrix4d parseMatrix(const std::vector<std::vector<double>>& vec) {
  // this need to be done to respect config file <shit>
  std::vector<double> mat_vec;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      mat_vec.push_back(vec[r][c]);
    }
  }
  return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(mat_vec.data());
}

void writeTransformedPose(std::ofstream& estimate_file_buf,
                          const Eigen::Matrix4d& lidar_to_world,
                          const Eigen::Matrix4d& lidar_to_base) {
  const Eigen::Matrix4d base_to_world = lidar_to_base * lidar_to_world * lidar_to_base.inverse();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      estimate_file_buf << base_to_world(row, col);
      if (col < 3) {
        estimate_file_buf << " ";
      }
    }
    if (row < 2) {
      estimate_file_buf << " ";
    }
  }
  estimate_file_buf << std::endl;
}