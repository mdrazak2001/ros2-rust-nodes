from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Top-level launch arguments
    topic_arg = DeclareLaunchArgument('topic', default_value='/custom_topic')
    publish_rate_arg = DeclareLaunchArgument('publish_rate_ms', default_value='1000')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='Hello')
    suffix_arg = DeclareLaunchArgument('suffix', default_value='World')
    count_start_arg = DeclareLaunchArgument('count_start', default_value='0')
    output_file_arg = DeclareLaunchArgument('output_file', default_value='subscriber_output.txt')
    
    # Get the share directory of this launch package
    package_share = get_package_share_directory('rust_nodes_bringup')
    intermediate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'intermediate_launch.py')
        ),
        launch_arguments={
            'topic': LaunchConfiguration('topic'),
            'publish_rate_ms': LaunchConfiguration('publish_rate_ms'),
            'prefix': LaunchConfiguration('prefix'),
            'suffix': LaunchConfiguration('suffix'),
            'count_start': LaunchConfiguration('count_start'),
            'output_file': LaunchConfiguration('output_file')
        }.items()
    )
    
    ld = LaunchDescription([
        topic_arg, publish_rate_arg, prefix_arg, suffix_arg, count_start_arg, output_file_arg,
        intermediate_launch
    ])
    return ld
