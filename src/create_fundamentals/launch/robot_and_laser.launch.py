import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("create_fundamentals")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    robot_port = LaunchConfiguration("robot_port")

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
            DeclareLaunchArgument(
                "robot_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for iRobot Create",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "laser.launch.py")
                ),
                launch_arguments={
                    "serial_port": serial_port,
                    "serial_baud": serial_baud,
                }.items(),
            ),
            Node(
                package="create_fundamentals",
                executable="driver.py",
                name="create_driver",
                output="screen",
                parameters=[{"port": robot_port}],
            ),
        ]
    )
