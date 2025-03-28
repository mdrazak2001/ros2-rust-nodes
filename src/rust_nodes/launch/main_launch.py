import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Top-level launch file.
    # Use Case: This is the single entry point for the entire application.
    # It is especially useful when using an LSP server to inspect the launch structure
    # and to support advanced features like code navigation and launch debugging.
    
    # Include the intermediate scaffolding launch file
    intermediate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'intermediate_launch.py')
        )
    )
    ld.add_action(intermediate_launch)
    
    return ld
