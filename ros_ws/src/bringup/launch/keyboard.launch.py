from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    keyboard_teleop_node = Node(
        package='keyboard_teleop',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        keyboard_teleop_node,
    ])
