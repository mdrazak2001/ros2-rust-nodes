import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Parameters for the publisher node:
    topic_name = 'custom_topic'
    publish_rate_ms = '2000'
    prefix = 'Hello'
    suffix = 'World'
    count_start = '0'
    
    # Use Case: Launching the publisher node with custom runtime parameters.
    # The comments here help an LSP server understand the configuration details and 
    # assist with code completion and navigation in complex launch setups.
    publisher_cmd = [
        'ros2', 'run', 'rust_nodes', 'simple_publisher',
        topic_name, publish_rate_ms, prefix, suffix, count_start
    ]
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=publisher_cmd,
            output='screen'
        )
    ])
