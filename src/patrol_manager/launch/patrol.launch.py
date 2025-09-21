from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 🔹 Camera driver (Pi Camera)
        Node(
            package='raspicam2',
            executable='raspicam2_node',
            name='raspicam2_node',
            output='screen',
            parameters=[{
                'image_size': [640, 480],
                'framerate': 15.0,
            }],
            remappings=[
                ('image_raw', '/raspicam2/image_raw'),
                ('camera_info', '/raspicam2/camera_info'),
            ]
        ),

        # 🔹 Lidar driver (YDLIDAR)
        Node(
            package='ydlidar',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen'
        ),

        # 🔹 Patrol manager (movement + obstacle avoidance)
        Node(
            package='patrol_manager',
            executable='patrol_node.py',
            name='patrol_manager',
            output='screen'
        ),

        # 🔹 Person detector (subscribe to Pi Camera)
        Node(
            package='smart_retail_perception',
            executable='person_detector.py',
            name='person_detector',
            output='screen',
            remappings=[
                ('/camera/image_raw', '/raspicam2/image_raw')
            ]
        ),

        # 🔹 Odometry fusion (IMU + wheels)
        Node(
            package='odom_fusion',
            executable='odom_fusion_node.py',
            name='odom_fusion',
            output='screen'
        ),

        # 🔹 Dashboard (Flask web UI)
        Node(
            package='dashboard',
            executable='dashboard_node.py',
            name='dashboard',
            output='screen'
        ),
    ])
