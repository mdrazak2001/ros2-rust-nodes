from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Look up the share directory of the launch package.
    package_share = get_package_share_directory('rust_nodes_bringup')
    
    intermediate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'intermediate_launch.py')
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(intermediate_launch)
    return ld
