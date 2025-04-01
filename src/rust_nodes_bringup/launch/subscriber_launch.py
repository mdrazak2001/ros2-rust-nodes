from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments.
    topic_arg = DeclareLaunchArgument('topic', default_value='custom_topic')
    output_file_arg = DeclareLaunchArgument('output_file', default_value='subscriber_output.txt')

    # Use LaunchConfiguration to read the arguments.
    topic = LaunchConfiguration('topic')
    output_file = LaunchConfiguration('output_file')

    # Define the subscriber node. Parameters are passed so the node can read them at runtime.
    subscriber_node = Node(
        package='rust_nodes',
        executable='simple_subscriber',
        name='subscriber_node',
        output='screen',
        parameters=[{'topic': topic, 'output_file': output_file}]
    )

    return LaunchDescription([
        topic_arg,
        output_file_arg,
        subscriber_node
    ])
