from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_button_action')
    params_file = os.path.join(pkg_share, 'params', 'example.yaml')

    button_action_node = Node(
        package='rm_button_action',
        executable='rm_button_action',
        name='rm_button_action',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        button_action_node,
    ])
