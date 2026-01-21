"""Launch file for pose-graph based MAD-ICP localizer."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = FindPackageShare(package="mad_icp_ros").find("mad_icp_ros")
    default_config = os.path.join(
        package_dir, "config", "pose_graph_localizer", "default.yaml"
    )

    # Launch Arguments

    arg_map_dir = DeclareLaunchArgument(
        "map_dir",
        default_value="",
        description="Path to map directory containing manifest.txt and trees/",
    )

    arg_config = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Localizer parameters file (yaml)",
    )

    # Topics
    arg_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/cloud_in",
        description="Input 3D cloud topic (sensor_msgs/PointCloud2)",
    )

    arg_initialpose_topic = DeclareLaunchArgument(
        "initialpose_topic",
        default_value="/initialpose",
        description="Initial pose topic (geometry_msgs/PoseWithCovarianceStamped)",
    )

    arg_wheel_odom_topic = DeclareLaunchArgument(
        "wheel_odom_topic",
        default_value="/wheel_odom",
        description="Optional wheel odometry topic (nav_msgs/Odometry)",
    )

    arg_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localizer/odom",
        description="Output odometry topic (nav_msgs/Odometry)",
    )

    arg_pose_topic = DeclareLaunchArgument(
        "pose_topic",
        default_value="/localizer/pose",
        description="Output pose topic (geometry_msgs/PoseWithCovarianceStamped)",
    )

    arg_debug_viz = DeclareLaunchArgument(
        "debug_viz",
        default_value="true",
        description="Publish debug point clouds (local_map, current_cloud) for RViz",
    )

    # LaunchConfigurations
    map_dir = LaunchConfiguration("map_dir")
    config = LaunchConfiguration("config")
    cloud_topic = LaunchConfiguration("cloud_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    debug_viz = LaunchConfiguration("debug_viz")

    # Node
    localizer_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_pose_graph_localizer",
        name="pose_graph_localizer",
        output="screen",
        parameters=[
            config,
            {"map_dir": map_dir},
            {"publish_debug_clouds": debug_viz},
        ],
        remappings=[
            # Input topics
            ("/cloud_in", cloud_topic),
            ("/initialpose", initialpose_topic),
            ("/wheel_odom", wheel_odom_topic),
            # Output topics
            ("/odom", odom_topic),
            ("/pose", pose_topic),
        ],
    )

    return LaunchDescription(
        [
            arg_map_dir,
            arg_config,
            arg_cloud_topic,
            arg_initialpose_topic,
            arg_wheel_odom_topic,
            arg_odom_topic,
            arg_pose_topic,
            arg_debug_viz,
            localizer_node,
        ]
    )
