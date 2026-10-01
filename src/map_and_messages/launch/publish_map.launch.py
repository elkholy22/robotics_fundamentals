from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="map_and_messages",
                executable="publish_map.py",
                name="example_map_publisher",
                output="screen",
            )
        ]
    )
