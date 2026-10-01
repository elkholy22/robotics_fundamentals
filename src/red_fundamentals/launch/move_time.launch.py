from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_lidar',
            arguments=['0.14', '0.0', '0.0', '0', '0', '0', 'base_link', 'lidar_link']
        ),
        
        Node(
            package='red_fundamentals',
            executable='conversion',
            name='conversion'
        ),
        
        Node(
            package='red_fundamentals',
            executable='line_detection',
            name='line_detection',
            arguments=["30", "0.25", "0.05", "300"]
        ),
        Node(
            package='red_fundamentals',
            executable='movement',
            name='movement',
            arguments=["1", "6", "20.0", "50", "2.4", "1", "15", "-1.3", "4", "4"]
        ),
        
        Node(
            package='red_fundamentals',
            executable='align',
            name='align',
            arguments=['3']
        ),
        
        Node(
            package='red_fundamentals',
            executable='grid_fast',
            name='grid_mover',
            arguments=['1']
        )
    ])
