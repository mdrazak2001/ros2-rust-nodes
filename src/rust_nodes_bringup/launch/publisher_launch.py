from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for dynamic configuration.
    topic_arg = DeclareLaunchArgument('topic', default_value='default_topic')
    rate_arg = DeclareLaunchArgument('publish_rate_ms', default_value='1000')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='Hello')
    suffix_arg = DeclareLaunchArgument('suffix', default_value='World')
    count_start_arg = DeclareLaunchArgument('count_start', default_value='0')

    # Use LaunchConfiguration to read the arguments.
    topic = LaunchConfiguration('topic')
    publish_rate = LaunchConfiguration('publish_rate_ms')
    prefix = LaunchConfiguration('prefix')
    suffix = LaunchConfiguration('suffix')
    count_start = LaunchConfiguration('count_start')

    # Define the publisher node. The parameters are passed via the 'parameters' field.
    publisher_node = Node(
        package='rust_nodes',
        executable='simple_publisher',
        name='publisher_node',
        output='screen',
        parameters=[{
            'topic': topic,
            'publish_rate_ms': publish_rate,
            'prefix': prefix,
            'suffix': suffix,
            'count_start': count_start
        }]
    )

    return LaunchDescription([
        topic_arg,
        rate_arg,
        prefix_arg,
        suffix_arg,
        count_start_arg,
        publisher_node
    ])
