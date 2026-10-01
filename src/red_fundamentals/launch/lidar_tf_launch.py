from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_lidar',
            arguments=['0.14', '0.0', '0.0', '0', '0', '0', 'base_link', 'lidar_link'] # 18cm weiter vorne, Hoehe hat keine Relevanz
        )
    ])