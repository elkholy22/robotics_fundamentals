import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laser_filter_config = os.path.join(
        get_package_share_directory("create_fundamentals"),
        "laser_config.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial device for Hokuyo URG laser",
            ),
            DeclareLaunchArgument(
                "serial_baud",
                default_value="115200",
                description="Baud rate for Hokuyo URG laser serial connection",
            ),
            Node(
                package="urg_node",
                executable="urg_node_driver",
                name="urg_node",
                output="screen",
                parameters=[
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {"serial_baud": LaunchConfiguration("serial_baud")},
                ],
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="laser_filter",
                output="screen",
                parameters=[laser_filter_config, {"use_sim_time": False}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="robot_to_laser",
                arguments=[
                    "0.12",
                    "0",
                    "0.16",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "laser",
                ],
            ),
        ]
    )
