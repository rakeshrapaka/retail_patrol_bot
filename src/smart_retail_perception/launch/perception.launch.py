from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='smart_retail_perception', executable='person_detector', output='screen')
    ])
