from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    image_topic = LaunchConfiguration('image_topic')
    frame_id = LaunchConfiguration('frame_id')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    protocol_width = LaunchConfiguration('protocol_width')
    protocol_height = LaunchConfiguration('protocol_height')
    publish_hz = LaunchConfiguration('publish_hz')
    draw_names = LaunchConfiguration('draw_names')

    rm_ui_debugger = Node(
        package='rm_ui',
        executable='rm_ui_debugger',
        name='rm_ui_debugger',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'image_topic': image_topic,
            'frame_id': frame_id,
            'image_width': image_width,
            'image_height': image_height,
            'protocol_width': protocol_width,
            'protocol_height': protocol_height,
            'publish_hz': publish_hz,
            'draw_names': draw_names,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='send_message',
            description='SendMessage topic subscribed by the UI debugger.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='ui_debug/image',
            description='Debug image topic published as sensor_msgs/msg/Image.'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='rm_ui_debug',
            description='Frame id used in the published debug image header.'),
        DeclareLaunchArgument(
            'image_width',
            default_value='1920',
            description='Published debug image width in pixels.'),
        DeclareLaunchArgument(
            'image_height',
            default_value='1080',
            description='Published debug image height in pixels.'),
        DeclareLaunchArgument(
            'protocol_width',
            default_value='1920',
            description='Referee UI protocol coordinate width.'),
        DeclareLaunchArgument(
            'protocol_height',
            default_value='1080',
            description='Referee UI protocol coordinate height.'),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='30.0',
            description='Debug image publish frequency.'),
        DeclareLaunchArgument(
            'draw_names',
            default_value='true',
            description='Draw three-byte figure names next to figures.'),
        rm_ui_debugger,
    ])
