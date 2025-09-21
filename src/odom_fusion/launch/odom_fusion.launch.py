from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='odom_fusion', executable='ekf_node', output='screen')
    ])
