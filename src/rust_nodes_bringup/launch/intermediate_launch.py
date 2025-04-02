from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Look up the share directory of the launch package.
    package_share = get_package_share_directory('rust_nodes_bringup')
    
    publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'publisher_launch.py')
        )
    )
    
    subscriber_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'subscriber_launch.py')
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(publisher_launch)
    ld.add_action(subscriber_launch)
    return ld
