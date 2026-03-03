from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('rm_custom_controller_imu')
    
    # 参数文件路径
    params_file = os.path.join(pkg_share, 'test', 'config', 'params.yaml')

    # 创建节点
    rm_custom_controller_imu_node = Node(
        package='rm_custom_controller_imu',
        executable='rm_custom_controller_imu',
        name='rm_custom_controller_imu',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        rm_custom_controller_imu_node,
    ])
