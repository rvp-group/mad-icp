import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_dir = FindPackageShare(package="mad_icp_ros").find("mad_icp_ros")

    default_localizer_cfg = os.path.join(
       package_dir, "config", "localizer", "default.yaml" 
    )

    arg_map_filename = DeclareLaunchArgument(
        "map_filename",
        default_value="",
        description="Map filename"
    )
    arg_map_format = DeclareLaunchArgument(
        "map_format",
        default_value="pcd",
        description="Format of the map file"
    )

    arg_localizer_cfg = DeclareLaunchArgument(
        "localizer_config",
        default_value=default_localizer_cfg,
        description="Localizer parameters file (yaml)"
    )



    map_filename = LaunchConfiguration("map_filename")
    map_format = LaunchConfiguration("map_format")
    localizer_cfg = LaunchConfiguration("localizer_config")

    map_server_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"map_filename": map_filename},
            {"map_format": map_format}
        ],
        remappings=[
            ("/map", "/map"),
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
            # ("/cloud_in", "/cloud_in"),
            ("/cloud_in", "/utlidar/cloud_livox_mid360"),
            ("/initialpose", "/initialpose"),
            ("/map", "/map"),
            # Output topics
            ("/odom", "/localizer/odom"),
            ("/pose", "/localizer/pose"),
        ]
    )

    return LaunchDescription([
        arg_map_filename,
        arg_map_format,
        arg_localizer_cfg,
        map_server_node,
        localizer_node,
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                "/opt/ros/overlay_ws/src/mad-icp/aula_magna_record0_0.db3",
                "--topics",
                "/utlidar/cloud_livox_mid360"
            ]
        )
    ])

