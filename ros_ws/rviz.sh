cd ~/labroom-searching/ros_ws

source ./install/setup.bash

export ROS_DISCOVERY_SERVER=100.126.180.124:11811

ros2 launch bringup rviz2.launch.py
