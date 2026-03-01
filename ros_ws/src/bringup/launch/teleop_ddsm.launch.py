from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ddsm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ddsm_controller'),
                'launch',
                'launch.py'
            )
        )
    )

    keyboard_teleop_node = Node(
        package='keyboard_teleop',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        ddsm_launch,
        keyboard_teleop_node,
    ])
