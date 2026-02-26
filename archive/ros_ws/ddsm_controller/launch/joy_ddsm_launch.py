import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 各パッケージの設定ファイルを取得
    ddsm_config = os.path.join(
        get_package_share_directory('ddsm_controller'),
        'config',
        'config.yaml'
    )
    
    joy_config = os.path.join(
        get_package_share_directory('joy_driver'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        # ジョイスティックノード (joyパッケージ) - 最初に起動
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config]
        ),

        # ジョイドライバーノード
        Node(
            package='joy_driver',
            executable='joy_driver_node',
            name='joy_driver_node',
            output='screen',
            parameters=[joy_config]
        ),

        # DDSMコントローラーノード - 最後に起動
        Node(
            package='ddsm_controller',
            executable='ddsm_controller_node',
            name='ddsm_controller_node',
            output='screen',
            parameters=[ddsm_config]
        ),
    ])