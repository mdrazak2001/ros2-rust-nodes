import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Parameters for the subscriber node:
    topic_name = 'custom_topic'
    # The output file will be written to the user's home directory.
    # output_file = os.path.join(os.path.expanduser('~'), 'subscriber_output.txt')
    output_file = 'subscriber_output.txt'
    
    # Use Case: Launching the subscriber node with parameters for file logging.
    # This configuration is ideal for debugging, logging, and later analysis.
    subscriber_cmd = [
        'ros2', 'run', 'rust_nodes', 'simple_subscriber',
        topic_name, output_file
    ]
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=subscriber_cmd,
            output='screen'
        )
    ])
