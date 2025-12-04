import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = FindPackageShare(package="mad_icp_ros").find("mad_icp_ros")
    default_localizer_cfg = os.path.join(
        package_dir, "config", "localizer", "default.yaml"
    )

    # Launch Arguments

    arg_map_filename = DeclareLaunchArgument(
        "map_filename",
        default_value="",
        description="Map filename (absolute path or relative to map directory)",
    )
    arg_map_format = DeclareLaunchArgument(
        "map_format",
        default_value="pcd",
        description="Format of the map file (e.g. pcd, ply, etc.)",
    )

    # Localizer YAML
    arg_localizer_cfg = DeclareLaunchArgument(
        "localizer_config",
        default_value=default_localizer_cfg,
        description="Localizer parameters file (yaml)",
    )

    # Topics (the parts that change per robot)
    arg_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/cloud_in",
        description="Input 3D cloud topic with sensor_msgs::msg::PointCloud2 messages.",
    )

    arg_initialpose_topic = DeclareLaunchArgument(
        "initialpose_topic",
        default_value="/initialpose",
        description="Initial pose topic",
    )

    arg_map_topic = DeclareLaunchArgument(
        "map_topic",
        default_value="/map",
        description="Map 3D cloud topic with sensor_msgs::msg::PointCloud2 messages."
        " Use with RMW_QOS_POLICY_RELIABILITY_RELIABLE and RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL",
    )

    arg_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localizer/odom",
        description="Output pose topic with nav_msgs::msg::Odometry messages",
    )

    arg_pose_topic = DeclareLaunchArgument(
        "pose_topic",
        default_value="/localizer/pose",
        description="Output pose topic with geometry_msgs::msg::PoseWithCovarianceStamped messages",
    )

    # --- LaunchConfigurations ---

    map_filename = LaunchConfiguration("map_filename")
    map_format = LaunchConfiguration("map_format")
    localizer_cfg = LaunchConfiguration("localizer_config")

    cloud_topic = LaunchConfiguration("cloud_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    map_topic = LaunchConfiguration("map_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    pose_topic = LaunchConfiguration("pose_topic")

    # --- Nodes ---

    map_server_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"map_filename": map_filename},
            {"map_format": map_format},
        ],
        remappings=[
            ("/map", map_topic),
            ("/map_lowres", "/map_lowres"),
        ],
    )

    localizer_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_localizer",
        name="localizer",
        output="screen",
        parameters=[localizer_cfg],
        remappings=[
            # Input topics
            ("/cloud_in", cloud_topic),
            ("/initialpose", initialpose_topic),
            ("/map", map_topic),
            # Output topics
            ("/odom", odom_topic),
            ("/pose", pose_topic),
        ],
    )

    return LaunchDescription(
        [
            arg_map_filename,
            arg_map_format,
            arg_localizer_cfg,
            arg_cloud_topic,
            arg_initialpose_topic,
            arg_map_topic,
            arg_odom_topic,
            arg_pose_topic,
            map_server_node,
            localizer_node,
        ]
    )
