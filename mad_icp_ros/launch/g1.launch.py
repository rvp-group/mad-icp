import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = SetParameter(name="use_sim_time", value=False)

    package_dir = FindPackageShare(package="mad_icp_ros").find("mad_icp_ros")
    rviz_config_file = os.path.join(package_dir, "config", "rviz", "viz.rviz")
    # rviz_config = DeclareLaunchArgument(
    #     "rviz_config",
    #     default_value=rviz_config_file,
    #     description="Path to the RViz configuration file",
    # )

    mad_icp_config_filepath = os.path.join(
        package_dir, "config", "mad_icp", "unitree_g1.yaml"
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rviz_config")],
    # )

    mad_icp_ros_node = Node(
        package="mad_icp_ros",
        executable="mad_icp_odometry",
        name="mad_icp_odometry",
        output="screen",
        parameters=[mad_icp_config_filepath],
        remappings=[
            ("/points", "/utlidar/cloud_livox_mid360"),
            # ("/odom_init", "/j100_0819/platform/odom"),
        ],
    )

    # hardcoded shit to type less shit
    # bag_play_process = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "bag",
    #         "play",
    #         "/home/fabioscap/Desktop/mad-icp/bags/rosbag2_2025_06_23-17_01_59",
    #     ],
    #     output="screen",
    # )
    #

    # Configure CycloneDDS
    cyclonedds_config = os.path.join(package_dir, "config", "cyclonedds.xml")
    ld.add_action(
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="CYCLONEDDS_URI", value=f"file://{cyclonedds_config}"
        )
    )

    # Add actions to the LaunchDescription
    ld.add_action(use_sim_time)
    # ld.add_action(rviz_config)
    ld.add_action(mad_icp_ros_node)
    # ld.add_action(rviz_node)
    # ld.add_action(bag_play_process)

    return ld
