#include "mad_icp_ros/map_server.h"
#include <fstream>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>

namespace mad_icp_ros {
MapServer::MapServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mad_map_publisher", options) {
  init_params();

  init_publishers();

  load_map();

  pub_map_->publish(map_msg_);
  pub_map_lowres_->publish(map_lowres_msg_);
}

void MapServer::init_params() {
  map_filename_ = this->declare_parameter<std::string>("map_filename", "");
  map_format_ = this->declare_parameter<std::string>("map_format", "ply");
}
void MapServer::init_publishers() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                 .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", qos);
  pub_map_lowres_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("map_lowres", qos);
}

using PointType = pcl::PointXYZ;
using CloudType = pcl::PointCloud<PointType>;
using CloudPtr = typename CloudType::Ptr;

CloudPtr loadBINFile(const std::string &filename) {
  CloudPtr cloud(new pcl::PointCloud<PointType>);

  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("Failed to open file " + filename);
  }
  cloud->clear();
  while (ifs.good()) {
    float data[4];
    ifs.read(reinterpret_cast<char *>(data), sizeof(float) * 4);
    if (!ifs.good()) {
      break;
    }
    PointType p;
    p.x = data[0];
    p.y = data[1];
    p.z = data[2];
    cloud->push_back(p);
  }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

void MapServer::load_map() {
  CloudPtr cloud(new pcl::PointCloud<PointType>);

  if (map_format_ == "pcd") {
    int ret = pcl::io::loadPCDFile(map_filename_, *cloud);
    if (ret) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD file: %s (err=%d)",
                   map_filename_.c_str(), ret);
      throw std::runtime_error("PCD load error");
    }
  } else if (map_format_ == "ply") {
    int ret = pcl::io::loadPLYFile(map_filename_, *cloud);
    if (ret) {
      RCLCPP_ERROR(get_logger(), "Failed to load PLY file: %s (err=%d)",
                   map_filename_.c_str(), ret);
      throw std::runtime_error("PLY load error");
    }
  } else if (map_format_ == "bin") {
    cloud = loadBINFile(map_filename_);
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported map format: %s",
                 map_format_.c_str());
    throw std::runtime_error("Unsupported map format");
  }

  RCLCPP_INFO(get_logger(), "Loaded map '%s' with %zu points",
              map_filename_.c_str(), cloud->points.size());

  // Voxeize for low-res visualization map
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.2f, 0.2f, 0.2f);
  CloudPtr cloud_lowres(new pcl::PointCloud<PointType>);
  sor.filter(*cloud_lowres);

  pcl::toROSMsg(*cloud, map_msg_);
  map_msg_.header.stamp = this->now();
  map_msg_.header.frame_id = "map";

  pcl::toROSMsg(*cloud_lowres, map_lowres_msg_);
  map_lowres_msg_.header.stamp = map_msg_.header.stamp;
  map_lowres_msg_.header.frame_id = map_msg_.header.frame_id;
}
} // namespace mad_icp_ros
RCLCPP_COMPONENTS_REGISTER_NODE(mad_icp_ros::MapServer)
