import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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

    map_filename = LaunchConfiguration("map_filename")
    map_format = LaunchConfiguration("map_format")

    map_server_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"map_filename": map_filename},
            {"map_format": map_format}
        ]
    )

    return LaunchDescription([
        arg_map_filename,
        arg_map_format,
        map_server_node
    ])

