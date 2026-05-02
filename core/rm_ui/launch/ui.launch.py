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
            "sender_topic" : "send_message",
            "sender_hz": 15.0,
            "sender_target" : 2,
            "sender_id": 1,
            "receiver_id": 0x0101,
        }]
    )

    return LaunchDescription([
        rm_ui_debugger,
    ])
