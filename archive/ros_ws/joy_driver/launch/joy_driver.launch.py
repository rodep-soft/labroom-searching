import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージのパスを取得
    pkg_share = get_package_share_directory('joy_driver')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')

    return LaunchDescription([
        # joy_node (ジョイスティック入力を/joyトピックにpublish)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[config_file]
        ),

        # joy_driver_node (joyトピックを受け取ってモーターRPMに変換)
        Node(
            package='joy_driver',
            executable='joy_driver_node',
            name='joy_driver_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
