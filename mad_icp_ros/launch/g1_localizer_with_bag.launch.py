import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = FindPackageShare(package="mad_icp_ros").find("mad_icp_ros")

    # Defaults specific to this robot / dataset
    default_map = os.path.join(
        package_dir,
        "maps",
        "phdroom1.pcd",  # adjust to your structure
    )
    default_localizer_cfg = os.path.join(
        package_dir, "config", "localizer", "phdroom1.yaml"
    )
    default_bag = os.path.join(
        package_dir,
        "data",
        "recording_phdroom1",
        "recording_phdroom_1_0.db3",
    )

    # Arguments so you can still override if needed
    arg_map_filename = DeclareLaunchArgument(
        "map_filename",
        default_value=default_map,
        description="Map file",
    )
    arg_localizer_cfg = DeclareLaunchArgument(
        "localizer_config",
        default_value=default_localizer_cfg,
        description="Localizer config",
    )
    arg_bag_file = DeclareLaunchArgument(
        "bag_file",
        default_value=default_bag,
        description="Rosbag file to play",
    )

    map_filename = LaunchConfiguration("map_filename")
    localizer_cfg = LaunchConfiguration("localizer_config")
    bag_file = LaunchConfiguration("bag_file")

    # Include the generic localizer launch
    generic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "localizer.launch.py")
        ),
        launch_arguments={
            "map_filename": map_filename,
            "localizer_config": localizer_cfg,
            "map_format": "pcd",
            "cloud_topic": "/state/lidar/cloud_sync",
            "initialpose_topic": "/initialpose",
            "map_topic": "/map",
            "odom_topic": "/localizer/odom",
            "pose_topic": "/localizer/pose",
        }.items(),
    )

    # Bag playback specific to this dataset/robot
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_file,
            "--topics",
            "/state/lidar/cloud_sync",
            "/state/lidar/imu_sync",
            "/joint_states",
            "/tf",
            "/tf_static",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            arg_map_filename,
            arg_localizer_cfg,
            arg_bag_file,
            generic_launch,
            bag_play,
        ]
    )
