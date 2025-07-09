#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
  double b_max_{0};
  double b_min_{0};
  double b_ratio_{0};
  double p_th_{0};
  double rho_ker_{0};
  int n_{0};
  size_t max_icp_its_{0};
  double delta_chi_threshold_{0};
  size_t frame_window_{0};
  double sensor_hz_{0};
  bool deskew_{0};
  double min_range_{0};
  double max_range_{0};
  double intensity_thr_{0};
  Eigen::Isometry3d lidar_in_base_;

  int num_threads_{0};
  int num_keyframes_{0};

  std::string base_frame_{"base_link"};
  bool use_wheels_{true};

  bool publish_odom_{false};
  bool publish_tf_{false};
  // std::string lidar_frame_{"os0_sensor"};
  //

  // odometry state
  std::unique_ptr<MADicp> icp_;
  Eigen::Isometry3d frame_to_map_;
  Eigen::Isometry3d wheel_to_map_;  // store odom transformation here
  Eigen::Isometry3d diff_;
  // Eigen::Isometry3d keyframe_to_map_;
  ContainerType pc_container_;  // Intermediate container used to filter points
  rclcpp::Time pc_stamp_;
  rclcpp::Time odom_stamp_;
  bool initialized_;
  size_t seq_;  // progressively increasing counter for frames
  bool map_updated_;
  std::deque<Frame*> keyframes_;  // each scan is registered to these scans
  std::deque<Frame*>
      frames_;  // store recent frames to choose the best keyframe
  void reset();
  //

  // ROS2 Subscribers:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void init_subscribers();
  //

  // ROS2 Publishers:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  void init_publishers();
  //

  // Declares and initializes parameters.
  void init_params();

  // on receiving the first frame, initialize the odometry
  void initialize(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // on receiving subsequent frames, compute the odometry
  void compute(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void publish_odom_tf(const Eigen::Isometry3d& pose,
                       const rclcpp::Time& stamp);

  int max_parallel_levels_;
};
}  // namespace mad_icp_ros