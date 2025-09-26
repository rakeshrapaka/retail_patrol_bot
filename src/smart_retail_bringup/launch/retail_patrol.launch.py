import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros


def generate_launch_description():
    # ------------------------------
    # Paths to dependent packages
    # ------------------------------
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')

    navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
    rviz_launch_dir = os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
    ydlidar_launch_dir = os.path.join(get_package_share_directory('ydlidar'), 'launch')
    cartographer_launch_dir = os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')

    prefix_address = get_package_share_directory('tortoisebot_slam')
    default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot_simple.xacro')
    params_file = os.path.join(prefix_address, 'config', 'nav2_params.yaml')
    map_file = LaunchConfiguration('map')
    map_directory = os.path.join(
        get_package_share_directory('tortoisebot_bringup'), 'maps', 'room2.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('tortoisebot_description'),
        'rviz',
        'tortoisebot_sensor_display.rviz'
    )

    # ------------------------------
    # Include TortoiseBot Launches
    # ------------------------------
    state_publisher_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')
        ),
        launch_arguments={'params_file': params_file}.items()
    )

    cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'slam': exploration,
            'use_sim_time': use_sim_time
        }.items()
    )

    ydlidar_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x2_ydlidar_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------
    # Smart Retail Patrol custom nodes
    # ------------------------------
    patrol_manager_node = Node(
        package='patrol_manager',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        remappings=[
            ('/ydlidar_node/scan', '/scan'),   # remap ydlidar scan to /scan
            ('/scan', '/scan')                 # ensure patrol listens on /scan
        ]
    )

    perception_node = Node(
        package='smart_retail_perception',
        executable='person_detector',
        name='person_detector',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/raspicam2/image_raw')
        ]
    )

    odom_fusion_node = Node(
        package='odom_fusion',
        executable='odom_fusion_node',
        name='odom_fusion',
        output='screen'
    )

    dashboard_node = Node(
        package='dashboard',
        executable='dashboard_node',
        name='dashboard',
        output='screen'
    )

    camera_node = Node(
        package='raspicam2',
        executable='raspicam2_node',
        name='raspicam2_node',
        output='screen',
        parameters=[{
            'image_size': [640, 480],
            'framerate': 15.0,
        }]
    )

    raspicam2_node = Node(
        package='raspicam2',
        executable='raspicam2_node',
        name='raspicam2_node',
        output='screen',
        remappings=[
            ('/raspicam2/image_raw', '/camera/image_raw'),  # remap for detector
            ('/raspicam2/camera_info', '/camera/camera_info')
        ]
    )

    person_detector_node = Node(
        package='smart_retail_perception',
        executable='person_detector',
        name='person_detector',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/camera/image_raw')  # ensures it matches
        ]
    )

    firmware_node = Node(
        package='tortoisebot_firmware',
        executable='differential.py',
        name='differential_drive_publisher',
        output='screen'
    )

    # ------------------------------
    # Final LaunchDescription
    # ------------------------------
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='exploration',
            default_value='True',
            description='Enable SLAM exploration'
        ),
        DeclareLaunchArgument(
            name='map',
            default_value=map_directory,
            description='Map to be used'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),

        # TortoiseBot base system
        state_publisher_launch_cmd,
        #gazebo_launch_cmd,
        navigation_launch_cmd,
        cartographer_launch_cmd,
        ydlidar_launch_cmd,

        
        # Camera
        raspicam2_node,
        camera_node,

        # Smart Retail Patrol custom system
        patrol_manager_node,
        person_detector_node,
        perception_node,
        odom_fusion_node,
        dashboard_node,
        firmware_node,
    ])
