import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg      = get_package_share_directory('red_fundamentals')
    planning = os.path.join(pkg, 'launch', 'planning.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gold_file',   default_value='gold.txt'),
        DeclareLaunchArgument('pickup_file', default_value='pickup.txt'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning)
        ),

        Node(
            package='red_fundamentals',
            executable='challenge',
            name='challenge',
            output='screen',
            arguments=[
                LaunchConfiguration('gold_file'),
                LaunchConfiguration('pickup_file'),
            ],
        ),
    ])