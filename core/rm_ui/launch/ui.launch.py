from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    rm_ui_debugger = Node(
        package='rm_ui',
        executable='rm_ui',
        name='rm_ui',
        output='screen',
        parameters=[{
            "sender_topic" : "rm_manager/send_message",
            "sender_hz": 15.0,
            "sender_target" : 2,
            "sender_id": 2,
            "receiver_id": 0x0102,
        }]
    )

    return LaunchDescription([
        rm_ui_debugger,
    ])
