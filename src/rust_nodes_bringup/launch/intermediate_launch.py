import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Intermediate launch file.
    # Use Case: Splits the overall launch into modular parts.
    # This helps the LSP server by providing modular launch units for tasks such as selective debugging
    # or launching subsets of the application.
    
    # Include publisher launch file
    publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'publisher_launch.py')
        )
    )
    ld.add_action(publisher_launch)
    
    # Include subscriber launch file
    subscriber_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'subscriber_launch.py')
        )
    )
    ld.add_action(subscriber_launch)
    
    return ld
