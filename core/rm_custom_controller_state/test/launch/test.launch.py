from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the configuration file
    config_file_path = Path(get_package_share_directory('rm_custom_controller_state')) / 'test' / 'config' / 'params.yaml'

    # Define the node with the configuration file
    test_node = Node(
        package='rm_custom_controller_state',
        executable='rm_custom_controller_state',
        name='rm_custom_controller_state',
        output='screen',
        parameters=[str(config_file_path)]
    )

    # Create and return the launch description
    return LaunchDescription([test_node])