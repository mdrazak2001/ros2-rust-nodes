from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('rust_nodes_bringup')
    
    publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'publisher_launch.py')
        ),
        launch_arguments={
            'topic': LaunchConfiguration('topic'),
            'publish_rate_ms': LaunchConfiguration('publish_rate_ms'),
            'prefix': LaunchConfiguration('prefix'),
            'suffix': LaunchConfiguration('suffix'),
            'count_start': LaunchConfiguration('count_start')
        }.items()
    )
    
    subscriber_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'subscriber_launch.py')
        ),
        launch_arguments={
            'topic': LaunchConfiguration('topic'),
            'output_file': LaunchConfiguration('output_file')
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(publisher_launch)
    ld.add_action(subscriber_launch)
    return ld
