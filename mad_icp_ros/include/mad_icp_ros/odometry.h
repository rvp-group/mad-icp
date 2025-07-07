#include <Eigen/Dense>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "odometry/mad_icp.h"

namespace mad_icp_ros {

class Odometry : public rclcpp::Node {
 public:
  Odometry(const rclcpp::NodeOptions& options);

 protected:
  // mad_icp odometry parameters. They are explained in the
  // config/mad_icp/jackal.yaml.
  // Overriding these parameters at runtime has no effect
  double min_range_{0};
  double max_range_{0};
  double b_max_{0};
  double b_min_{0};
  double b_ratio_{0};
  double p_th_{0};
  double rho_ker_{0};
  int n_{0};
  double sensor_hz_{0};
  bool deskew_{0};
  double intensity_thr_{0};
  Eigen::Matrix4d lidar_in_base_;

  int num_threads_{0};

  std::string base_frame_{"base_link"};
  bool use_wheels_{true};
  // std::string lidar_frame_{"os0_sensor"};

  std::unique_ptr<MADicp> icp_;

  // odometry state
  Eigen::Isometry3d frame_to_map_;
  Eigen::Isometry3d keyframe_to_map_;
  ContainerType pc_container_;  // Intermediate container used to filter points
  rclcpp::Time stamp_;
  bool initialized_;
  size_t seq_;  // progressively increasing counter for frames
  bool map_updated_;
  std::deque<Frame*> keyframes_;  // each scan is registered to these scans
  void reset();

  // ROS2 Subscribers:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  void init_subscribers();

  // ROS2 callbacks
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 private:
  // Declares and initializes parameters.
  void init_params();

  // on receiving the first frame, initialize the odometry
  void initialize(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  int max_parallel_levels_;
};
}  // namespace mad_icp_ros