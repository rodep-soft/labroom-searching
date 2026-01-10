import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ddsm_controller'),
        'config',
        'config.yaml'
    )
    
    ddsm_controller_node = Node(
        package='ddsm_controller',
        executable='ddsm_controller_node',
        name='ddsm_controller_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        ddsm_controller_node
    ])