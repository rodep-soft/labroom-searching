from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('urg_node2'),
                'launch',
                'urg_node2.launch.py'
            )
        )
    )


    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(get_package_share_directory('bringup'),'config','slam_params.yaml'),
        }.items()
    )
    
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rf2o_laser_odometry'),
                'launch',
                'rf2o_laser_odometry.launch.py',
            )
        ),
        #launch_arguments={
        #    'use_sim_time': 'true',
        #}.items()
    )

    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        #parameters=[{'use_sim_time': True}]
    )


    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #parameters=[{'use_sim_time': True}]
    )
 
    return LaunchDescription([
        urg_launch,
        base_to_laser,
        slam_launch,
#        odom_to_base,
    ])

