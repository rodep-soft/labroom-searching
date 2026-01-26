from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('bringup'),'config','rviz2_config_odom.rviz')],
        parameters=[{'use_sim_time':True}]
    )

    return LaunchDescription([
        rviz
    ])

