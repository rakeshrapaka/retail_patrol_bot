from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to TortoiseBot bringup
    tortoisebot_bringup_dir = get_package_share_directory('tortoisebot_bringup')
    tortoisebot_launch = os.path.join(tortoisebot_bringup_dir, 'launch', 'bringup.launch.py')

    # Path to Smart Retail bringup
    smart_retail_bringup_dir = get_package_share_directory('smart_retail_bringup')
    retail_patrol_launch = os.path.join(smart_retail_bringup_dir, 'launch', 'retail_patrol.launch.py')

    return LaunchDescription([
        # Include TortoiseBot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tortoisebot_launch)
        ),

        # Include Smart Retail bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(retail_patrol_launch)
        ),
    ])
